/**
 * Cartesian Motion Planner
 *
 * Generates Cartesian waypoints for straight-line motion between two poses.
 * Used for pre-computing trajectories that will be sent to batch IK solver.
 *
 * Architecture:
 * 1. Frontend generates Cartesian waypoints (this file)
 * 2. Backend batch IK solves all waypoints ONCE
 * 3. ExecuteTrajectory executes joint trajectory at 100Hz
 */

import { CartesianPose } from './types';

export interface CartesianPlannerOptions {
  duration: number;           // seconds
  interpolationType?: 'linear' | 'slerp';  // Future: support SLERP for orientation
}

/**
 * Calculate the number of waypoints for a given duration.
 *
 * Formula: num_waypoints = int(duration / INTERVAL_S)
 * Where INTERVAL_S = 0.01 (100Hz execution rate)
 *
 * Examples:
 * - 1.0s duration → 100 waypoints
 * - 2.5s duration → 250 waypoints
 * - 0.5s duration → 50 waypoints
 */
export function calculateWaypointCount(duration: number): number {
  const INTERVAL_S = 0.01;
  return Math.floor(duration / INTERVAL_S);
}

/**
 * Linear interpolation between two values.
 */
function lerp(start: number, end: number, t: number): number {
  return start + (end - start) * t;
}

/**
 * Normalize angle to [-180, 180] range to handle wrapping.
 */
function normalizeAngle(angle: number): number {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

/**
 * Interpolate angle considering wrapping (shortest path).
 *
 * Example: Interpolating from 170° to -170°
 * - Naive: 170 → 0 → -170 (340° rotation)
 * - Smart: 170 → 180/-180 → -170 (20° rotation) ✓
 */
function lerpAngle(start: number, end: number, t: number): number {
  // Normalize both angles to [-180, 180]
  start = normalizeAngle(start);
  end = normalizeAngle(end);

  // Calculate shortest angular distance
  let diff = end - start;
  if (diff > 180) {
    diff -= 360;
  } else if (diff < -180) {
    diff += 360;
  }

  return normalizeAngle(start + diff * t);
}

/**
 * Generate Cartesian waypoints for straight-line motion.
 *
 * @param startPose - Starting Cartesian pose [X, Y, Z, RX, RY, RZ]
 * @param endPose - Ending Cartesian pose [X, Y, Z, RX, RY, RZ]
 * @param options - Planning options (duration, interpolation type)
 * @returns Array of Cartesian waypoints
 *
 * @example
 * ```ts
 * const waypoints = generateCartesianWaypoints(
 *   { X: 200, Y: 0, Z: 300, RX: 0, RY: 0, RZ: 0 },
 *   { X: 300, Y: 100, Z: 300, RX: 0, RY: 45, RZ: 0 },
 *   { duration: 2.0 }
 * );
 * // Returns 200 waypoints for 2.0s duration at 100Hz
 * ```
 */
export function generateCartesianWaypoints(
  startPose: CartesianPose,
  endPose: CartesianPose,
  options: CartesianPlannerOptions
): CartesianPose[] {
  const { duration, interpolationType = 'linear' } = options;

  // Calculate number of waypoints
  const numWaypoints = calculateWaypointCount(duration);

  if (numWaypoints <= 0) {
    throw new Error(`Invalid duration ${duration}s - must be > 0.01s`);
  }

  const waypoints: CartesianPose[] = [];

  // Generate waypoints using linear interpolation
  for (let i = 0; i < numWaypoints; i++) {
    // Interpolation parameter: 0.0 → 1.0
    const t = i / (numWaypoints - 1);

    // Linear interpolation for position
    const X = lerp(startPose.X, endPose.X, t);
    const Y = lerp(startPose.Y, endPose.Y, t);
    const Z = lerp(startPose.Z, endPose.Z, t);

    // Angular interpolation for orientation (handles wrapping)
    const RX = lerpAngle(startPose.RX, endPose.RX, t);
    const RY = lerpAngle(startPose.RY, endPose.RY, t);
    const RZ = lerpAngle(startPose.RZ, endPose.RZ, t);

    waypoints.push({ X, Y, Z, RX, RY, RZ });
  }

  return waypoints;
}

/**
 * Generate Cartesian waypoints for path VISUALIZATION (not execution).
 *
 * Unlike generateCartesianWaypoints(), this generates a FIXED number of samples
 * based on the geometric distance, making the path shape independent of timing.
 * This is appropriate for preview/visualization purposes.
 *
 * Sampling: 1 sample per 10mm of linear distance
 * Min samples: 5 (for very short movements)
 * Max samples: 100 (to prevent performance issues)
 *
 * @param startPose - Starting Cartesian pose [X, Y, Z, RX, RY, RZ]
 * @param endPose - Ending Cartesian pose [X, Y, Z, RX, RY, RZ]
 * @returns Array of Cartesian waypoints for visualization
 *
 * @example
 * ```ts
 * const waypoints = generateCartesianPathForVisualization(
 *   { X: 200, Y: 0, Z: 300, RX: 0, RY: 0, RZ: 0 },
 *   { X: 300, Y: 100, Z: 300, RX: 0, RY: 45, RZ: 0 }
 * );
 * // Returns ~14 waypoints for ~141mm distance (sqrt(100^2 + 100^2))
 * ```
 */
export function generateCartesianPathForVisualization(
  startPose: CartesianPose,
  endPose: CartesianPose
): CartesianPose[] {
  // Calculate Cartesian distance (in mm)
  const dx = endPose.X - startPose.X;
  const dy = endPose.Y - startPose.Y;
  const dz = endPose.Z - startPose.Z;
  const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

  // Calculate number of samples based on distance (1 per 10mm)
  const MIN_SAMPLES = 5;
  const MAX_SAMPLES = 100;
  const numSamples = Math.max(MIN_SAMPLES, Math.min(MAX_SAMPLES, Math.ceil(distance / 10)));

  const waypoints: CartesianPose[] = [];

  // Generate waypoints using linear interpolation
  for (let i = 0; i <= numSamples; i++) {
    // Interpolation parameter: 0.0 → 1.0
    const t = i / numSamples;

    // Linear interpolation for position
    const X = lerp(startPose.X, endPose.X, t);
    const Y = lerp(startPose.Y, endPose.Y, t);
    const Z = lerp(startPose.Z, endPose.Z, t);

    // Angular interpolation for orientation (handles wrapping)
    const RX = lerpAngle(startPose.RX, endPose.RX, t);
    const RY = lerpAngle(startPose.RY, endPose.RY, t);
    const RZ = lerpAngle(startPose.RZ, endPose.RZ, t);

    waypoints.push({ X, Y, Z, RX, RY, RZ });
  }

  return waypoints;
}

/**
 * Convert CartesianPose to array format expected by backend.
 *
 * @param pose - Cartesian pose object
 * @returns Array [X, Y, Z, RX, RY, RZ]
 */
export function poseToArray(pose: CartesianPose): number[] {
  return [pose.X, pose.Y, pose.Z, pose.RX, pose.RY, pose.RZ];
}

/**
 * Convert array format to CartesianPose object.
 *
 * @param array - Array [X, Y, Z, RX, RY, RZ]
 * @returns Cartesian pose object
 */
export function arrayToPose(array: number[]): CartesianPose {
  if (array.length !== 6) {
    throw new Error(`Expected 6 elements, got ${array.length}`);
  }
  return {
    X: array[0],
    Y: array[1],
    Z: array[2],
    RX: array[3],
    RY: array[4],
    RZ: array[5],
  };
}

/**
 * Validate that waypoint count matches expected duration.
 *
 * Used to verify backend trajectory before execution.
 *
 * @param waypoints - Generated waypoints
 * @param expectedDuration - Expected duration in seconds
 * @param toleranceWaypoints - Allowed difference in waypoint count (default: 5)
 * @returns true if valid, false otherwise
 */
export function validateWaypointCount(
  waypoints: any[],
  expectedDuration: number,
  toleranceWaypoints: number = 5
): boolean {
  const expectedCount = calculateWaypointCount(expectedDuration);
  const actualCount = waypoints.length;
  const diff = Math.abs(expectedCount - actualCount);

  return diff <= toleranceWaypoints;
}
