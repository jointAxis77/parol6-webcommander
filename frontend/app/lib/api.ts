/**
 * API client for PAROL6 backend services
 */

import { CartesianPose, JointAngles, IkAxisMask } from './types';
import { getApiBaseUrl } from './apiConfig';
import { ORIENTATION_CONFIG } from './constants';
import { logger } from './logger';

/**
 * Check if backend API is reachable
 */
export async function checkBackendConnection(): Promise<boolean> {
  try {
    const response = await fetch(`${getApiBaseUrl()}/health`, {
      method: 'GET',
      signal: AbortSignal.timeout(2000) // 2 second timeout
    });
    return response.ok;
  } catch {
    return false;
  }
}

/**
 * Move robot joints to specified angles
 */
export async function moveJoints(
  angles: JointAngles,
  speedPercentage?: number,
  duration?: number
): Promise<{ success: boolean; error?: string }> {
  try {
    const anglesArray = [angles.J1, angles.J2, angles.J3, angles.J4, angles.J5, angles.J6];

    // Build request body with either speed_percentage OR duration (mutually exclusive)
    const requestBody: any = {
      angles: anglesArray
    };

    if (duration !== undefined) {
      requestBody.duration = duration;
    } else if (speedPercentage !== undefined) {
      requestBody.speed_percentage = speedPercentage;
    } else {
      throw new Error('Must provide either speedPercentage or duration');
    }

    const response = await fetch(`${getApiBaseUrl()}/api/robot/move/joints`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
      throw new Error(`Move joints failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return { success: true };
  } catch (error) {
    logger.error('Move joints error', 'API', error);
    return {
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error'
    };
  }
}

// ============================================================================
// Camera API
// ============================================================================

export interface CameraDevice {
  device: string;
  name: string;
}

export interface CameraStatus {
  streaming: boolean;
  device: string | null;
  width: number | null;
  height: number | null;
  fps: number | null;
}

/**
 * Get list of available USB camera devices
 */
export async function getCameraDevices(): Promise<{
  success: boolean;
  devices?: CameraDevice[];
  error?: string
}> {
  try {
    const response = await fetch(`${getApiBaseUrl()}/api/camera/devices`, {
      method: 'GET'
    });

    if (!response.ok) {
      throw new Error(`Get camera devices failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return {
      success: true,
      devices: data.devices
    };
  } catch (error) {
    logger.error('Get camera devices error', 'API', error);
    return {
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error'
    };
  }
}

/**
 * Get current camera status
 */
export async function getCameraStatus(): Promise<{
  success: boolean;
  status?: CameraStatus;
  error?: string;
}> {
  try {
    const response = await fetch(`${getApiBaseUrl()}/api/camera/status`, {
      method: 'GET'
    });

    if (!response.ok) {
      throw new Error(`Get camera status failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return {
      success: true,
      status: data
    };
  } catch (error) {
    logger.error('Get camera status error', 'API', error);
    return {
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error'
    };
  }
}

/**
 * Start camera on specified device
 */
export async function startCamera(
  device: string,
  width?: number,
  height?: number,
  fps?: number
): Promise<{
  success: boolean;
  status?: CameraStatus;
  error?: string;
}> {
  try {
    const params = new URLSearchParams({ device });
    if (width !== undefined) params.append('width', width.toString());
    if (height !== undefined) params.append('height', height.toString());
    if (fps !== undefined) params.append('fps', fps.toString());

    const response = await fetch(`${getApiBaseUrl()}/api/camera/start?${params}`, {
      method: 'POST'
    });

    if (!response.ok) {
      throw new Error(`Start camera failed: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    return {
      success: true,
      status: data.status
    };
  } catch (error) {
    logger.error('Start camera error', 'API', error);
    return {
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error'
    };
  }
}

/**
 * Stop camera capture
 */
export async function stopCamera(): Promise<{
  success: boolean;
  error?: string;
}> {
  try {
    const response = await fetch(`${getApiBaseUrl()}/api/camera/stop`, {
      method: 'POST'
    });

    if (!response.ok) {
      throw new Error(`Stop camera failed: ${response.status} ${response.statusText}`);
    }

    return { success: true };
  } catch (error) {
    logger.error('Stop camera error', 'API', error);
    return {
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error'
    };
  }
}

/**
 * Get camera stream URL
 */
export function getCameraStreamUrl(): string {
  return `${getApiBaseUrl()}/api/camera/stream`;
}

export interface ExecuteTrajectoryRequest {
  trajectory: number[][];  // Array of [J1-J6] in degrees
  duration?: number;  // Optional duration for validation
  wait_for_ack?: boolean;
  timeout?: number;
}

export interface CommandResponse {
  success: boolean;
  command_id?: string;
  message?: string;
  status?: string;
  details?: string;
}

/**
 * Execute a pre-computed joint trajectory at 100Hz.
 *
 * This achieves 100Hz execution (same as MoveJoint) with Cartesian straight-line
 * motion by using pre-computed joint positions (no real-time IK overhead).
 *
 * @param request - Trajectory execution request
 * @returns Command response with status
 *
 * @example
 * ```ts
 * // First, solve IK for waypoints using frontend kinematics
 * const jointTrajectory = [
 *   [0, -45, 90, 0, 45, 0],
 *   [5, -40, 85, 0, 40, 5],
 *   // ... more waypoints
 * ];
 *
 * // Then execute the trajectory
 * const execResult = await executeTrajectory({
 *   trajectory: jointTrajectory,
 *   duration: 2.0,
 *   wait_for_ack: true
 * });
 *
 * if (execResult.success) {
 *   logger.debug('Trajectory executing', 'API', { execResult });
 * }
 * ```
 */
export async function executeTrajectory(request: ExecuteTrajectoryRequest): Promise<CommandResponse> {
  const startTime = performance.now();

  try {
    const response = await fetch(`${getApiBaseUrl()}/api/robot/execute/trajectory`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`Execute trajectory failed: ${response.status} ${errorText}`);
    }

    const data: CommandResponse = await response.json();
    const elapsedMs = performance.now() - startTime;

    logger.info(
      `Execute trajectory ${data.success ? 'sent' : 'failed'}: ` +
      `${request.trajectory.length} waypoints, ` +
      `command_id=${data.command_id || 'N/A'} ` +
      `(${elapsedMs.toFixed(0)}ms)`,
      'api'
    );

    return data;
  } catch (error) {
    const elapsedMs = performance.now() - startTime;
    logger.error(`Execute trajectory error after ${elapsedMs.toFixed(0)}ms: ${error}`, 'api');

    return {
      success: false,
      message: error instanceof Error ? error.message : 'Unknown error',
    };
  }
}

