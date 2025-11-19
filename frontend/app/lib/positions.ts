/**
 * Position utilities - Single source of truth for robot positions
 * All positions are loaded from config.yaml via configStore
 */

import { useConfigStore } from './configStore';
import type { JointAngles } from './types';

/**
 * Convert saved position from config (array) to JointAngles object
 */
export function arrayToJointAngles(joints: number[]): JointAngles {
  return {
    J1: joints[0] ?? 0,
    J2: joints[1] ?? 0,
    J3: joints[2] ?? 0,
    J4: joints[3] ?? 0,
    J5: joints[4] ?? 0,
    J6: joints[5] ?? 0,
  };
}

/**
 * Get a saved position by name from config
 * Returns null if position not found
 */
export function getPositionByName(name: string): JointAngles | null {
  const config = useConfigStore.getState().config;
  if (!config?.ui?.saved_positions) {
    return null;
  }

  const position = config.ui.saved_positions.find(
    (pos) => pos.name.toLowerCase() === name.toLowerCase()
  );

  if (!position) {
    return null;
  }

  return arrayToJointAngles(position.joints);
}

/**
 * Get all saved positions from config
 */
export function getAllPositions(): Array<{ name: string; joints: JointAngles }> {
  const config = useConfigStore.getState().config;
  if (!config?.ui?.saved_positions) {
    return [];
  }

  return config.ui.saved_positions.map((pos) => ({
    name: pos.name,
    joints: arrayToJointAngles(pos.joints),
  }));
}

/**
 * Get Home position (primary default position)
 * Falls back to all zeros if not found in config
 */
export function getHomePosition(): JointAngles {
  const homePos = getPositionByName('Home');
  if (homePos) {
    return homePos;
  }

  // Fallback if Home not found
  return {
    J1: 90,
    J2: -90,
    J3: 180,
    J4: 0,
    J5: 0,
    J6: 180,
  };
}

/**
 * Get Park position
 * Falls back to Home if not found in config
 */
export function getParkPosition(): JointAngles {
  const parkPos = getPositionByName('Park');
  if (parkPos) {
    return parkPos;
  }

  // Fallback to Home if Park not found
  return getHomePosition();
}

/**
 * Get Ready position
 * Falls back to Home if not found in config
 */
export function getReadyPosition(): JointAngles {
  const readyPos = getPositionByName('Ready');
  if (readyPos) {
    return readyPos;
  }

  // Fallback to Home if Ready not found
  return getHomePosition();
}
