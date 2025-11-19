/**
 * Coordinate System Transformations
 *
 * The PAROL6 system uses two coordinate systems:
 *
 * 1. ROBOT COORDINATES (Z-up, industry standard):
 *    - X: Forward from robot base
 *    - Y: Left (right-hand rule)
 *    - Z: Up from mounting surface
 *    - Used by: Backend, stores, IK solver, API communication
 *
 * 2. THREE.JS DISPLAY COORDINATES (Y-up, 3D graphics standard):
 *    - X: Same as robot X
 *    - Y: Up (was robot Z)
 *    - Z: Backward (was robot -Y)
 *    - Used by: URDF visualization, Three.js rendering
 *
 * This file provides transformation functions between these coordinate systems.
 * Transformations should ONLY be applied at the display boundary - all data
 * layer (stores, IK, API) works in robot coordinates.
 */

import type { CartesianPose } from './types';

/**
 * Convert robot coordinates (Z-up) to Three.js display coordinates (Y-up)
 *
 * Use this when:
 * - Displaying poses from stores in UI labels
 * - Positioning gizmos from robot coordinate poses
 * - Any time you need to show robot coordinates in Three.js space
 *
 * @param robotPose - Pose in robot coordinates (Z-up)
 * @returns Pose in Three.js coordinates (Y-up)
 *
 * @example
 * const robotPose = { X: 200, Y: 0, Z: 300, ... }; // Robot coords
 * const displayPose = robotToThreeJs(robotPose);
 * // displayPose = { X: 200, Y: 300, Z: 0, ... } for Three.js display
 */
export function robotToThreeJs(robotPose: CartesianPose): CartesianPose {
  return {
    X: robotPose.X,         // X stays the same
    Y: robotPose.Z,         // Robot Z → Display Y (up becomes up)
    Z: -robotPose.Y,        // Robot Y → Display -Z (left becomes backward)
    RX: robotPose.RX,       // Rotation around X-axis stays same
    RY: robotPose.RY,       // Rotation around Y-axis stays same
    RZ: robotPose.RZ        // Rotation around Z-axis stays same
  };
}

/**
 * Convert Three.js display coordinates (Y-up) to robot coordinates (Z-up)
 *
 * Use this when:
 * - User inputs cartesian coordinates in UI
 * - Reading positions from Three.js gizmos
 * - Converting display coordinates before sending to API or IK solver
 *
 * @param displayPose - Pose in Three.js coordinates (Y-up)
 * @returns Pose in robot coordinates (Z-up)
 *
 * @example
 * const displayPose = { X: 200, Y: 300, Z: 0, ... }; // Three.js coords
 * const robotPose = threeJsToRobot(displayPose);
 * // robotPose = { X: 200, Y: 0, Z: 300, ... } for robot commands
 */
export function threeJsToRobot(displayPose: CartesianPose): CartesianPose {
  return {
    X: displayPose.X,       // X stays the same
    Y: -displayPose.Z,      // Display Z → Robot -Y (backward becomes left)
    Z: displayPose.Y,       // Display Y → Robot Z (up becomes up)
    RX: displayPose.RX,     // Rotation around X-axis stays same
    RY: displayPose.RY,     // Rotation around Y-axis stays same
    RZ: displayPose.RZ      // Rotation around Z-axis stays same
  };
}

/**
 * Check if transformation is working correctly (for testing)
 * Should return the original pose after round-trip transformation
 */
export function testRoundTrip(robotPose: CartesianPose): boolean {
  const display = robotToThreeJs(robotPose);
  const backToRobot = threeJsToRobot(display);

  const tolerance = 0.001;
  return (
    Math.abs(robotPose.X - backToRobot.X) < tolerance &&
    Math.abs(robotPose.Y - backToRobot.Y) < tolerance &&
    Math.abs(robotPose.Z - backToRobot.Z) < tolerance &&
    Math.abs(robotPose.RX - backToRobot.RX) < tolerance &&
    Math.abs(robotPose.RY - backToRobot.RY) < tolerance &&
    Math.abs(robotPose.RZ - backToRobot.RZ) < tolerance
  );
}
