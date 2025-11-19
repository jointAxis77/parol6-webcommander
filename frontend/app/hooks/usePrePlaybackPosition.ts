/**
 * usePrePlaybackPosition Hook
 *
 * Ensures robot is at t=0 keyframe position before starting execute playback.
 * Moves robot to start position at 50% speed if not already there.
 */

import { useState, useRef } from 'react';
import { useTimelineStore } from '../lib/stores/timelineStore';
import { useHardwareStore } from '../lib/stores/hardwareStore';
import { JointAngles } from '../lib/types';
import { moveJoints } from '../lib/api';
import { getJointAnglesAtTime } from '../lib/interpolation';
import { JOINT_NAMES } from '../lib/constants';
import { logger } from '../lib/logger';

const POSITION_TOLERANCE_DEGREES = 0.5; // 0.5 degree tolerance (accounts for sensor noise + WebSocket latency)
const MOVE_SPEED_PERCENTAGE = 50; // Fixed 50% speed for pre-move
const POLL_INTERVAL_MS = 100; // Check position every 100ms
const TIMEOUT_MS = 60000; // 60 second safety timeout

/**
 * Check if actual position matches target within tolerance
 */
function isAtPosition(actual: JointAngles, target: JointAngles, tolerance: number): boolean {
  return JOINT_NAMES.every(joint =>
    Math.abs(actual[joint] - target[joint]) <= tolerance
  );
}

export function usePrePlaybackPosition() {
  const [isMovingToStart, setIsMovingToStart] = useState(false);
  const [moveError, setMoveError] = useState<string | null>(null);

  const pollIntervalRef = useRef<NodeJS.Timeout | null>(null);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);

  const hardwareJointAngles = useHardwareStore((state) => state.hardwareJointAngles);
  const keyframes = useTimelineStore((state) => state.timeline.keyframes);
  const play = useTimelineStore((state) => state.play);

  /**
   * Clear all timers
   */
  const clearTimers = () => {
    if (pollIntervalRef.current) {
      clearInterval(pollIntervalRef.current);
      pollIntervalRef.current = null;
    }
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
      timeoutRef.current = null;
    }
  };

  /**
   * Move to start position and then start playback
   */
  const moveToStartAndPlay = async () => {
    // Check if robot is connected
    if (!hardwareJointAngles) {
      logger.error('Robot not connected - cannot execute playback', 'PrePlayback');
      setMoveError('Robot not connected. Please connect robot before executing playback.');
      return;
    }

    // Get t=0 target position
    if (keyframes.length === 0) {
      logger.error('No keyframes found', 'PrePlayback');
      setMoveError('No keyframes to play');
      return;
    }

    const targetPosition = getJointAnglesAtTime(keyframes, 0);

    // Check if already at position
    if (isAtPosition(hardwareJointAngles, targetPosition, POSITION_TOLERANCE_DEGREES)) {
      play(true);
      return;
    }

    // Start moving to position
    setIsMovingToStart(true);
    setMoveError(null);

    try {
      // Send move command at 50% speed
      const result = await moveJoints(targetPosition, MOVE_SPEED_PERCENTAGE);

      if (!result.success) {
        throw new Error(result.error || 'Move command failed');
      }

      // Immediate re-check: position may have updated while sending move command
      // This handles the common case where WebSocket data was stale but updated during the command
      const immediateCheck = useHardwareStore.getState().hardwareJointAngles;
      if (immediateCheck && isAtPosition(immediateCheck, targetPosition, POSITION_TOLERANCE_DEGREES)) {
        setIsMovingToStart(false);
        play(true);
        return;
      }

      // Poll for position arrival
      return new Promise<void>((resolve, reject) => {
        let startTime = Date.now();

        // Set timeout
        timeoutRef.current = setTimeout(() => {
          clearTimers();
          const error = 'Timeout waiting for robot to reach start position (60s)';
          logger.error('Error during pre-move', 'PrePlayback', error);
          setMoveError(error);
          setIsMovingToStart(false);
          reject(new Error(error));
        }, TIMEOUT_MS);

        // Poll actual position
        pollIntervalRef.current = setInterval(() => {
          const currentActual = useHardwareStore.getState().hardwareJointAngles;

          if (!currentActual) {
            clearTimers();
            const error = 'Lost connection to robot during pre-move';
            logger.error('Error during pre-move', 'PrePlayback', error);
            setMoveError(error);
            setIsMovingToStart(false);
            reject(new Error(error));
            return;
          }

          // Check if at position
          if (isAtPosition(currentActual, targetPosition, POSITION_TOLERANCE_DEGREES)) {
            clearTimers();
            setIsMovingToStart(false);

            // Start playback
            play(true);
            resolve();
          }
        }, POLL_INTERVAL_MS);
      });

    } catch (error) {
      clearTimers();
      const errorMsg = error instanceof Error ? error.message : 'Unknown error during pre-move';
      logger.error('Error', 'PrePlayback', { errorMsg });
      setMoveError(errorMsg);
      setIsMovingToStart(false);
    }
  };

  return {
    moveToStartAndPlay,
    isMovingToStart,
    moveError,
    clearError: () => setMoveError(null)
  };
}
