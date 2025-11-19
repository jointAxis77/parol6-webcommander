/**
 * Forward and Inverse Kinematics for PAROL6 Robot
 *
 * Based on DH parameters from PAROL6_ROBOT.py:
 * - Joint angles input/output are in degrees
 * - Poses are in millimeters for position (X, Y, Z)
 * - Rotations are in degrees (RX, RY, RZ) - Euler angles
 */

import * as THREE from 'three';
import { JointAngles, CartesianPose, IkAxisMask, Tool } from './types';
import { STANDBY_POSITION, ORIENTATION_CONFIG, JOINT_ANGLE_OFFSETS, JOINT_LIMITS } from './constants';
import { useTimelineStore } from './store';
import { calculateTcpPoseFromUrdf } from './tcpCalculations';
import { threeJsToRobot } from './coordinateTransform';
import { applyJointAnglesToUrdf } from './urdfHelpers';

// DH Parameters from PAROL6_ROBOT.py (converted to mm)
const DH_PARAMS = {
  a1: 110.50,   // mm
  a2: 23.42,
  a3: 180.0,
  a4: 43.5,
  a5: 176.35,
  a6: 62.8,
  a7: 45.25,
  // alpha values (radians)
  alpha: [-Math.PI / 2, Math.PI, Math.PI / 2, -Math.PI / 2, Math.PI / 2, Math.PI]
};

import { logger } from './logger';
export interface IKResult {
  success: boolean;
  jointAngles: JointAngles | null;
  iterations?: number; // Number of iterations taken
  finalError?: number; // Final position error in mm
  error?: {
    reason: 'out_of_reach' | 'singular' | 'invalid_input' | 'no_urdf';
    message: string;
    distance?: number;
  };
}

/**
 * Numerical Inverse Kinematics using URDF's accurate FK
 * Uses damped least squares (Levenberg-Marquardt style) with finite difference Jacobian
 * Supports full 6-DOF (position + orientation) with selective axis masking
 *
 * @param targetPose - Target TCP pose (position in mm, orientation in degrees)
 * @param currentJoints - Starting joint configuration (seed)
 * @param urdfRobot - URDF robot model for accurate FK
 * @param tcpOffset - TCP offset from L6 frame (in mm)
 * @param axisMask - Which axes to solve for (defaults to position-only)
 * @param maxIterations - Maximum solver iterations
 * @param tolerance - Convergence tolerance (mm for position, degrees for orientation)
 * @returns IK result with success status and computed joint angles
 */
export function numericalIK(
  targetPose: CartesianPose,
  currentJoints: JointAngles,
  urdfRobot: any,
  tool: Tool,
  axisMask: IkAxisMask = { X: true, Y: true, Z: true, RX: false, RY: false, RZ: false },
  maxIterations: number = 30,
  tolerance: number = 1.0
): IKResult {
  const activeAxes = Object.entries(axisMask).filter(([_, enabled]) => enabled).map(([axis]) => axis);

  if (!urdfRobot) {
    logger.error('FAILED: URDF robot not loaded', 'numericalIK');
    return {
      success: false,
      jointAngles: null,
      error: {
        reason: 'no_urdf',
        message: 'URDF robot model not loaded'
      }
    };
  }

  // Validate input pose for NaN values
  const poseValues = [targetPose.X, targetPose.Y, targetPose.Z, targetPose.RX, targetPose.RY, targetPose.RZ];
  if (poseValues.some(v => isNaN(v))) {
    logger.error('FAILED: Input pose contains NaN', 'numericalIK', { targetPose });
    return {
      success: false,
      jointAngles: null,
      error: {
        reason: 'invalid_input',
        message: `Input pose contains NaN values: X=${targetPose.X}, Y=${targetPose.Y}, Z=${targetPose.Z}, RX=${targetPose.RX}, RY=${targetPose.RY}, RZ=${targetPose.RZ}`
      }
    };
  }

  // Start from current joint configuration
  let joints = { ...currentJoints };
  const lambda = 0.1; // Damping factor for numerical stability

  for (let iter = 0; iter < maxIterations; iter++) {
    // Get current TCP pose using URDF FK
    const currentTcp = getURDFTcpPose(urdfRobot, joints, tool);
    if (!currentTcp) {
      logger.error(`FAILED: FK computation failed at iteration ${iter}`, 'numericalIK');
      return {
        success: false,
        jointAngles: null,
        error: {
          reason: 'invalid_input',
          message: 'Failed to compute FK during IK iteration'
        }
      };
    }

    // Compute 6-DOF error vector, applying axis mask
    // For disabled axes, set error to 0 (already satisfied)
    const errors = [
      axisMask.X ? (targetPose.X - currentTcp.X) : 0,
      axisMask.Y ? (targetPose.Y - currentTcp.Y) : 0,
      axisMask.Z ? (targetPose.Z - currentTcp.Z) : 0,
      axisMask.RX ? normalizeAngleDelta(targetPose.RX - currentTcp.RX) : 0,
      axisMask.RY ? normalizeAngleDelta(targetPose.RY - currentTcp.RY) : 0,
      axisMask.RZ ? normalizeAngleDelta(targetPose.RZ - currentTcp.RZ) : 0
    ];

    const errorVec = new Matrix(errors.map(e => [e]), 6, 1);

    const errorNorm = Math.sqrt(errors.reduce((sum, e) => sum + e * e, 0));

    // Check convergence
    if (errorNorm < tolerance) {
      return {
        success: true,
        jointAngles: joints,
        iterations: iter + 1,
        finalError: errorNorm
      };
    }

    // Compute Jacobian
    const jacobian = computeJacobian(urdfRobot, joints, tool);
    if (!jacobian) {
      logger.error(`FAILED: Jacobian computation failed at iteration ${iter}`, 'numericalIK');
      return {
        success: false,
        jointAngles: null,
        error: {
          reason: 'singular',
          message: 'Failed to compute Jacobian'
        }
      };
    }

    // Solve for joint angle changes using damped least squares
    // ΔJ = J^T(JJ^T + λI)^-1 * error
    let deltaJoints: Matrix;
    try {
      const pseudoInv = jacobian.dampedPseudoInverse(lambda);
      deltaJoints = pseudoInv.multiply(errorVec);
    } catch (e) {
      logger.error(`FAILED: Matrix inversion failed at iteration ${iter}`, 'numericalIK', e);
      return {
        success: false,
        jointAngles: null,
        error: {
          reason: 'singular',
          message: 'Matrix inversion failed - singular configuration'
        }
      };
    }

    // Update joint angles with step limiting for stability
    const maxStep = 5.0; // degrees
    const jointNames = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const;
    jointNames.forEach((joint, i) => {
      let delta = deltaJoints.data[i][0];
      // Limit step size
      delta = Math.max(-maxStep, Math.min(maxStep, delta));
      joints[joint] += delta;
    });

    // Apply joint limits from constants
    jointNames.forEach((joint) => {
      const limits = JOINT_LIMITS[joint];
      joints[joint] = Math.max(limits.min, Math.min(limits.max, joints[joint]));
    });
  }

  // Max iterations reached
  const finalTcp = getURDFTcpPose(urdfRobot, joints, tool);
  const finalError = finalTcp
    ? Math.sqrt(
        (axisMask.X ? (targetPose.X - finalTcp.X) ** 2 : 0) +
        (axisMask.Y ? (targetPose.Y - finalTcp.Y) ** 2 : 0) +
        (axisMask.Z ? (targetPose.Z - finalTcp.Z) ** 2 : 0) +
        (axisMask.RX ? normalizeAngleDelta(targetPose.RX - finalTcp.RX) ** 2 : 0) +
        (axisMask.RY ? normalizeAngleDelta(targetPose.RY - finalTcp.RY) ** 2 : 0) +
        (axisMask.RZ ? normalizeAngleDelta(targetPose.RZ - finalTcp.RZ) ** 2 : 0)
      )
    : Infinity;

  return {
    success: false,
    jointAngles: joints, // Return best attempt
    iterations: maxIterations,
    finalError,
    error: {
      reason: 'out_of_reach',
      message: `Failed to converge after ${maxIterations} iterations (error: ${isNaN(finalError) ? 'NaN' : finalError.toFixed(1)}mm)`,
      distance: finalError
    }
  };
}

/**
 * Inverse Kinematics - Using numerical solver with URDF FK
 * Supports full 6-DOF (position + orientation) with axis masking
 *
 * @param targetPose - Target Cartesian pose (full 6-DOF)
 * @param currentJoints - Current joint configuration (for seed)
 * @param urdfRobot - URDF robot model (get from kinematicsStore.computationRobotRef)
 * @param tool - Tool configuration with TCP offset
 * @param axisMask - Which axes to solve for (optional, defaults to position-only)
 * @returns IK result with success status
 */
export function inverseKinematicsDetailed(
  targetPose: CartesianPose,
  currentJoints: JointAngles,
  urdfRobot: any,
  tool: Tool,
  axisMask?: IkAxisMask
): IKResult {
  // Use numerical IK solver with full pose and axis mask
  return numericalIK(
    targetPose,
    currentJoints,
    urdfRobot,
    tool,
    axisMask
  );
}

/**
 * Simplified IK wrapper that returns joint angles or null (backwards compatible)
 */
export function inverseKinematics(
  targetPose: CartesianPose,
  currentJoints: JointAngles
): JointAngles | null {
  const result = inverseKinematicsDetailed(targetPose, currentJoints);
  return result.success ? result.jointAngles : null;
}

/**
 * Get Cartesian interpolation between two poses
 * Used for cartesian keyframe interpolation
 */
export function interpolateCartesian(
  poseA: CartesianPose,
  poseB: CartesianPose,
  t: number // 0 to 1
): CartesianPose {
  const lerp = (a: number, b: number) => a + (b - a) * t;

  return {
    X: lerp(poseA.X, poseB.X),
    Y: lerp(poseA.Y, poseB.Y),
    Z: lerp(poseA.Z, poseB.Z),
    RX: lerp(poseA.RX, poseB.RX),
    RY: lerp(poseA.RY, poseB.RY),
    RZ: lerp(poseA.RZ, poseB.RZ),
  };
}

// ============================================================================
// NUMERICAL IK USING URDF'S ACCURATE FK
// ============================================================================

/**
 * Matrix utilities for numerical IK
 */
class Matrix {
  constructor(public data: number[][], public rows: number, public cols: number) {}

  static zeros(rows: number, cols: number): Matrix {
    return new Matrix(Array(rows).fill(0).map(() => Array(cols).fill(0)), rows, cols);
  }

  static identity(size: number): Matrix {
    const data = Array(size).fill(0).map(() => Array(size).fill(0));
    for (let i = 0; i < size; i++) data[i][i] = 1;
    return new Matrix(data, size, size);
  }

  transpose(): Matrix {
    const result = Matrix.zeros(this.cols, this.rows);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.cols; j++) {
        result.data[j][i] = this.data[i][j];
      }
    }
    return result;
  }

  multiply(other: Matrix): Matrix {
    if (this.cols !== other.rows) throw new Error('Matrix dimensions mismatch');
    const result = Matrix.zeros(this.rows, other.cols);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < other.cols; j++) {
        let sum = 0;
        for (let k = 0; k < this.cols; k++) {
          sum += this.data[i][k] * other.data[k][j];
        }
        result.data[i][j] = sum;
      }
    }
    return result;
  }

  add(other: Matrix): Matrix {
    const result = Matrix.zeros(this.rows, this.cols);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.cols; j++) {
        result.data[i][j] = this.data[i][j] + other.data[i][j];
      }
    }
    return result;
  }

  scale(scalar: number): Matrix {
    const result = Matrix.zeros(this.rows, this.cols);
    for (let i = 0; i < this.rows; i++) {
      for (let j = 0; j < this.cols; j++) {
        result.data[i][j] = this.data[i][j] * scalar;
      }
    }
    return result;
  }

  // Damped pseudoinverse for numerical stability
  dampedPseudoInverse(lambda: number = 0.01): Matrix {
    // For damped least squares: J^T(JJ^T + λI)^-1
    const JT = this.transpose();
    const JJT = this.multiply(JT);
    const lambdaI = Matrix.identity(JJT.rows).scale(lambda);
    const JJT_damped = JJT.add(lambdaI);

    // Matrix inversion (handles 3x3, 6x6, or general NxN)
    let inv: Matrix;
    if (JJT_damped.rows === 3 && JJT_damped.cols === 3) {
      inv = this.invert3x3(JJT_damped);
    } else if (JJT_damped.rows === 6 && JJT_damped.cols === 6) {
      inv = this.invert6x6(JJT_damped);
    } else {
      throw new Error(`Matrix inversion not supported for size ${JJT_damped.rows}x${JJT_damped.cols}`);
    }

    return JT.multiply(inv);
  }

  private invert3x3(m: Matrix): Matrix {
    if (m.rows !== 3 || m.cols !== 3) throw new Error('Only 3x3 inversion supported');
    const d = m.data;

    // Calculate determinant
    const det =
      d[0][0] * (d[1][1] * d[2][2] - d[1][2] * d[2][1]) -
      d[0][1] * (d[1][0] * d[2][2] - d[1][2] * d[2][0]) +
      d[0][2] * (d[1][0] * d[2][1] - d[1][1] * d[2][0]);

    if (Math.abs(det) < 1e-10) throw new Error('Matrix is singular');

    const invDet = 1 / det;
    const result = Matrix.zeros(3, 3);

    result.data[0][0] = (d[1][1] * d[2][2] - d[1][2] * d[2][1]) * invDet;
    result.data[0][1] = (d[0][2] * d[2][1] - d[0][1] * d[2][2]) * invDet;
    result.data[0][2] = (d[0][1] * d[1][2] - d[0][2] * d[1][1]) * invDet;
    result.data[1][0] = (d[1][2] * d[2][0] - d[1][0] * d[2][2]) * invDet;
    result.data[1][1] = (d[0][0] * d[2][2] - d[0][2] * d[2][0]) * invDet;
    result.data[1][2] = (d[0][2] * d[1][0] - d[0][0] * d[1][2]) * invDet;
    result.data[2][0] = (d[1][0] * d[2][1] - d[1][1] * d[2][0]) * invDet;
    result.data[2][1] = (d[0][1] * d[2][0] - d[0][0] * d[2][1]) * invDet;
    result.data[2][2] = (d[0][0] * d[1][1] - d[0][1] * d[1][0]) * invDet;

    return result;
  }

  private invert6x6(m: Matrix): Matrix {
    if (m.rows !== 6 || m.cols !== 6) throw new Error('Only 6x6 inversion supported');

    // Gaussian elimination with partial pivoting
    // Create augmented matrix [m | I]
    const aug = Matrix.zeros(6, 12);
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        aug.data[i][j] = m.data[i][j];
        aug.data[i][j + 6] = i === j ? 1 : 0; // Identity on right side
      }
    }

    // Forward elimination with pivoting
    for (let col = 0; col < 6; col++) {
      // Find pivot (largest absolute value in column)
      let maxRow = col;
      let maxVal = Math.abs(aug.data[col][col]);
      for (let row = col + 1; row < 6; row++) {
        const val = Math.abs(aug.data[row][col]);
        if (val > maxVal) {
          maxVal = val;
          maxRow = row;
        }
      }

      if (maxVal < 1e-10) throw new Error('Matrix is singular (6x6)');

      // Swap rows if needed
      if (maxRow !== col) {
        [aug.data[col], aug.data[maxRow]] = [aug.data[maxRow], aug.data[col]];
      }

      // Scale pivot row
      const pivot = aug.data[col][col];
      for (let j = 0; j < 12; j++) {
        aug.data[col][j] /= pivot;
      }

      // Eliminate column
      for (let row = 0; row < 6; row++) {
        if (row !== col) {
          const factor = aug.data[row][col];
          for (let j = 0; j < 12; j++) {
            aug.data[row][j] -= factor * aug.data[col][j];
          }
        }
      }
    }

    // Extract inverse from right side of augmented matrix
    const result = Matrix.zeros(6, 6);
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        result.data[i][j] = aug.data[i][j + 6];
      }
    }

    return result;
  }
}

/**
 * Get TCP pose (position + orientation) from URDF model in ROBOT COORDINATES
 * This is our ACCURATE FK that includes all 6 joints and TCP offset
 * Returns full 6-DOF pose in robot coordinates (Z-up): position (X, Y, Z in mm) and orientation (RX, RY, RZ in degrees)
 *
 * IMPORTANT: Converts from Three.js coordinates (Y-up) to robot coordinates (Z-up)
 * so the IK solver works entirely in robot coordinate space.
 */
function getURDFTcpPose(
  urdfRobot: any,
  jointAngles: JointAngles,
  tool: Tool
): CartesianPose | null {
  if (!urdfRobot) return null;

  try {
    // Apply joint angles to URDF using centralized helper
    applyJointAnglesToUrdf(urdfRobot, jointAngles);

    // Calculate TCP pose from URDF (returns Three.js coordinates)
    const threeJsPose = calculateTcpPoseFromUrdf(urdfRobot, tool.tcp_offset);
    if (!threeJsPose) return null;

    // Convert Three.js coordinates (Y-up) to robot coordinates (Z-up)
    // This ensures IK solver works entirely in robot coordinate space
    return threeJsToRobot(threeJsPose);
  } catch (e) {
    return null;
  }
}

/**
 * Compute Jacobian matrix via finite differences
 * Returns 6x6 matrix where each column is ∂Pose/∂Joint_i
 * Rows 0-2: position derivatives (∂X/∂J, ∂Y/∂J, ∂Z/∂J)
 * Rows 3-5: orientation derivatives (∂RX/∂J, ∂RY/∂J, ∂RZ/∂J)
 */
function computeJacobian(
  urdfRobot: any,
  jointAngles: JointAngles,
  tool: Tool,
  delta: number = 0.01 // degrees
): Matrix | null {
  const currentTcp = getURDFTcpPose(urdfRobot, jointAngles, tool);
  if (!currentTcp) return null;

  const jacobian = Matrix.zeros(6, 6);
  const jointNames = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const;

  for (let i = 0; i < 6; i++) {
    const joint = jointNames[i];
    const perturbedAngles = { ...jointAngles };
    perturbedAngles[joint] += delta;

    const perturbedTcp = getURDFTcpPose(urdfRobot, perturbedAngles, tool);
    if (!perturbedTcp) return null;

    // Compute derivative: ∂Pose/∂Joint_i
    jacobian.data[0][i] = (perturbedTcp.X - currentTcp.X) / delta;
    jacobian.data[1][i] = (perturbedTcp.Y - currentTcp.Y) / delta;
    jacobian.data[2][i] = (perturbedTcp.Z - currentTcp.Z) / delta;

    // Orientation derivatives (handle angle wrapping for ±180° discontinuities)
    jacobian.data[3][i] = normalizeAngleDelta(perturbedTcp.RX - currentTcp.RX) / delta;
    jacobian.data[4][i] = normalizeAngleDelta(perturbedTcp.RY - currentTcp.RY) / delta;
    jacobian.data[5][i] = normalizeAngleDelta(perturbedTcp.RZ - currentTcp.RZ) / delta;
  }

  return jacobian;
}

/**
 * Normalize angle delta to [-180, 180] range to handle wrapping
 */
function normalizeAngleDelta(delta: number): number {
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;
  return delta;
}
