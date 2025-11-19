import { Keyframe, JointAngles, JointName, CartesianPose } from './types';
import { JOINT_NAMES } from './constants';
import { getHomePosition } from './positions';

/**
 * Linear interpolation between two values
 */
function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

/**
 * Get interpolated joint angles at a specific time
 * Single keyframe model: each keyframe contains all 6 joint angles
 * All joints interpolate together as a coordinated pose
 */
export function getJointAnglesAtTime(
  keyframes: Keyframe[],
  time: number
): JointAngles {
  // Sort keyframes by time
  const sortedKeyframes = keyframes.slice().sort((a, b) => a.time - b.time);

  // If no keyframes, return home position
  if (sortedKeyframes.length === 0) {
    return getHomePosition();
  }

  // If before first keyframe, return first keyframe's joint angles
  if (time <= sortedKeyframes[0].time) {
    return sortedKeyframes[0].jointAngles;
  }

  // If after last keyframe, return last keyframe's joint angles
  if (time >= sortedKeyframes[sortedKeyframes.length - 1].time) {
    return sortedKeyframes[sortedKeyframes.length - 1].jointAngles;
  }

  // Find surrounding keyframes
  let before = sortedKeyframes[0];
  let after = sortedKeyframes[1];

  for (let i = 0; i < sortedKeyframes.length - 1; i++) {
    if (time >= sortedKeyframes[i].time && time <= sortedKeyframes[i + 1].time) {
      before = sortedKeyframes[i];
      after = sortedKeyframes[i + 1];
      break;
    }
  }

  // Calculate interpolation factor (0 to 1)
  const duration = after.time - before.time;
  const elapsed = time - before.time;
  const t = duration > 0 ? elapsed / duration : 0;

  // Interpolate all joints together
  const result: JointAngles = {} as JointAngles;
  JOINT_NAMES.forEach((joint) => {
    result[joint] = lerp(before.jointAngles[joint], after.jointAngles[joint], t);
  });

  return result;
}

/**
 * Calculate total path length (sum of angular distances)
 * Single keyframe model: sum all joint angle changes between consecutive keyframes
 */
export function calculatePathLength(keyframes: Keyframe[]): number {
  if (keyframes.length < 2) return 0;

  const sortedKeyframes = keyframes.slice().sort((a, b) => a.time - b.time);
  let totalDistance = 0;

  // Calculate distance between consecutive keyframes
  for (let i = 0; i < sortedKeyframes.length - 1; i++) {
    const kf1 = sortedKeyframes[i];
    const kf2 = sortedKeyframes[i + 1];

    // Sum angular changes across all joints
    JOINT_NAMES.forEach((joint) => {
      totalDistance += Math.abs(kf2.jointAngles[joint] - kf1.jointAngles[joint]);
    });
  }

  return totalDistance;
}

/**
 * Get interpolated cartesian pose at a specific time using keyframe cartesianPose data
 * Used when interpolating between keyframes where the NEXT keyframe has motionType === 'cartesian'
 */
export function getCartesianPoseAtTime(
  keyframes: Keyframe[],
  time: number
): CartesianPose | null {
  // Sort keyframes by time
  const sortedKeyframes = keyframes.slice().sort((a, b) => a.time - b.time);

  // If no keyframes, return null
  if (sortedKeyframes.length === 0) {
    return null;
  }

  // If before first keyframe, return null (no cartesian data yet)
  if (time <= sortedKeyframes[0].time) {
    return sortedKeyframes[0].cartesianPose || null;
  }

  // If after last keyframe, return last keyframe's cartesian pose
  if (time >= sortedKeyframes[sortedKeyframes.length - 1].time) {
    return sortedKeyframes[sortedKeyframes.length - 1].cartesianPose || null;
  }

  // Find surrounding keyframes
  let before = sortedKeyframes[0];
  let after = sortedKeyframes[1];

  for (let i = 0; i < sortedKeyframes.length - 1; i++) {
    if (time >= sortedKeyframes[i].time && time <= sortedKeyframes[i + 1].time) {
      before = sortedKeyframes[i];
      after = sortedKeyframes[i + 1];
      break;
    }
  }

  // If either keyframe doesn't have cartesian pose, return null
  if (!before.cartesianPose || !after.cartesianPose) {
    return null;
  }

  // Calculate interpolation factor (0 to 1)
  const duration = after.time - before.time;
  const elapsed = time - before.time;
  const t = duration > 0 ? elapsed / duration : 0;

  // Interpolate all cartesian values together
  return {
    X: lerp(before.cartesianPose.X, after.cartesianPose.X, t),
    Y: lerp(before.cartesianPose.Y, after.cartesianPose.Y, t),
    Z: lerp(before.cartesianPose.Z, after.cartesianPose.Z, t),
    RX: lerp(before.cartesianPose.RX, after.cartesianPose.RX, t),
    RY: lerp(before.cartesianPose.RY, after.cartesianPose.RY, t),
    RZ: lerp(before.cartesianPose.RZ, after.cartesianPose.RZ, t)
  };
}

/**
 * Determine if we should use cartesian interpolation at the given time
 * Returns true if we're interpolating TO a cartesian keyframe
 */
export function shouldUseCartesianInterpolation(
  keyframes: Keyframe[],
  time: number
): boolean {
  // Sort keyframes by time
  const sortedKeyframes = keyframes.slice().sort((a, b) => a.time - b.time);

  // Find the next keyframe (the one we're moving TO)
  for (let i = 0; i < sortedKeyframes.length - 1; i++) {
    if (time >= sortedKeyframes[i].time && time <= sortedKeyframes[i + 1].time) {
      // We're between keyframes[i] and keyframes[i+1]
      // Check if the NEXT keyframe (i+1) is cartesian
      return sortedKeyframes[i + 1].motionType === 'cartesian';
    }
  }

  // Not between keyframes, use joint by default
  return false;
}
