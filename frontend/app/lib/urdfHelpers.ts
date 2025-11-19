/**
 * URDF Helper Utilities
 *
 * Centralized functions for URDF robot manipulation to maintain DRY principle.
 * All joint angle transformations happen here in one place.
 */

import { JointAngles } from './types';
import { JOINT_ANGLE_OFFSETS } from './constants';

/**
 * Apply joint angles to URDF robot model with proper transformations
 *
 * This is the SINGLE source of truth for how joint angles map to URDF joint values.
 * Used by all visualization and computation code to ensure consistency.
 *
 * @param urdfRobot - The URDF robot instance to update
 * @param jointAngles - Joint angles in degrees {J1, J2, J3, J4, J5, J6}
 *
 * Transformations applied:
 * - angleSigns: Per-joint sign multipliers (for axis direction differences)
 * - JOINT_ANGLE_OFFSETS: Per-joint offset additions (for zero position differences)
 * - Degree to radian conversion
 * - Joint name mapping: J1→L1, J2→L2, etc.
 */
export function applyJointAnglesToUrdf(
  urdfRobot: any,
  jointAngles: JointAngles
): void {
  if (!urdfRobot) return;

  // Angle signs for each joint (calibrated for current URDF)
  // OLD URDF values: [1, 1, -1, -1, -1, -1]
  // NEW URDF values: [1, 1, 1, 1, 1, 1] (identity - no inversions)
  const angleSigns = [1, 1, 1, 1, 1, 1];

  const jointNames = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const;

  jointNames.forEach((joint, index) => {
    const angleDeg = jointAngles[joint];
    const sign = angleSigns[index];
    const offset = JOINT_ANGLE_OFFSETS[index] || 0;

    // Apply transformations: (angle * sign) + offset
    const correctedAngleDeg = angleDeg * sign + offset;

    // Convert to radians
    const angleRad = (correctedAngleDeg * Math.PI) / 180;

    // Map to URDF link name: J1→L1, J2→L2, etc.
    const linkName = `L${index + 1}`;

    try {
      urdfRobot.setJointValue(linkName, angleRad);
    } catch (e) {
      // Silently ignore joint not found errors
      // This can happen during URDF loading or if joint doesn't exist
    }
  });
}

/**
 * Get current joint angles from URDF robot
 *
 * Inverse of applyJointAnglesToUrdf - extracts joint angles from URDF state
 * and reverses the transformations to get back to user-space angles.
 *
 * @param urdfRobot - The URDF robot instance to read from
 * @returns Joint angles in degrees {J1, J2, J3, J4, J5, J6} or null if failed
 */
export function getJointAnglesFromUrdf(urdfRobot: any): JointAngles | null {
  if (!urdfRobot) return null;

  const angleSigns = [1, 1, 1, 1, 1, 1];
  const jointNames = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const;
  const result: any = {};

  try {
    jointNames.forEach((joint, index) => {
      const linkName = `L${index + 1}`;
      const angleRad = urdfRobot.joints[linkName]?.angle || 0;
      const angleDeg = (angleRad * 180) / Math.PI;

      // Reverse transformations: (angle - offset) / sign
      const offset = JOINT_ANGLE_OFFSETS[index] || 0;
      const sign = angleSigns[index];
      result[joint] = (angleDeg - offset) / sign;
    });

    return result as JointAngles;
  } catch (e) {
    return null;
  }
}
