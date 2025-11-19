/**
 * useActualFollowsTarget Hook
 *
 * Implements "Live Control Mode" - automatically sends move commands to the robot
 * whenever the commanded joint angles are changed in the UI.
 *
 * Features:
 * - Debounced command sending (500ms) to avoid spamming the API
 * - Tracks change source to prevent feedback loops
 * - Only active when liveControlEnabled is true
 * - Uses speed from command store (no duration - robot calculates based on speed)
 */

import { useEffect, useRef, useState } from 'react';
import { useCommandStore, useTimelineStore } from '../lib/stores';
import { JointAngles } from '../lib/types';
import { getApiBaseUrl } from '../lib/apiConfig';
import { logger } from '../lib/logger';

export function useActualFollowsTarget() {
  const liveControlEnabled = useCommandStore((state) => state.liveControlEnabled);
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const speed = useCommandStore((state) => state.speed);
  const isPlaying = useTimelineStore((state) => state.playbackState.isPlaying);

  // Track the last angles we sent to avoid sending duplicates
  const lastSentAngles = useRef<JointAngles | null>(null);
  const debounceTimer = useRef<NodeJS.Timeout | null>(null);
  const [isSending, setIsSending] = useState(false);

  useEffect(() => {
    // Skip during playback - playback handles its own command sending
    if (isPlaying) {
      return;
    }

    // Only proceed if live control mode is enabled
    if (!liveControlEnabled) {
      return;
    }

    // Check if angles have actually changed
    if (lastSentAngles.current &&
        JSON.stringify(commandedJointAngles) === JSON.stringify(lastSentAngles.current)) {
      return;
    }

    // Clear any existing debounce timer
    if (debounceTimer.current) {
      clearTimeout(debounceTimer.current);
    }

    // Set up new debounced command
    debounceTimer.current = setTimeout(async () => {
      try {
        setIsSending(true);

        // Convert to array format for API
        const angles = [
          commandedJointAngles.J1,
          commandedJointAngles.J2,
          commandedJointAngles.J3,
          commandedJointAngles.J4,
          commandedJointAngles.J5,
          commandedJointAngles.J6,
        ];

        // Use speed from command store (no duration - let robot calculate based on speed)
        const speedPercentage = speed;

        // Send move command to API
        // Note: Only send speed_percentage, NOT duration
        // Duration would override speed and make the slider useless
        const response = await fetch(`${getApiBaseUrl()}/api/robot/move/joints`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            angles,
            speed_percentage: speedPercentage,
            wait_for_ack: false, // Non-blocking
            timeout: 10.0,
          }),
        });

        if (!response.ok) {
          const error = await response.json();
          logger.error('Move command failed', 'LiveControl', error);
          return;
        }

        const result = await response.json();

        // Update last sent angles
        lastSentAngles.current = { ...commandedJointAngles };

      } catch (error) {
        logger.error('Error sending move command', 'LiveControl', error);
      } finally {
        setIsSending(false);
      }
    }, 500); // 500ms debounce

    // Cleanup on unmount or when dependencies change
    return () => {
      if (debounceTimer.current) {
        clearTimeout(debounceTimer.current);
      }
    };
  }, [liveControlEnabled, commandedJointAngles, speed, isPlaying]);

  // Reset last sent angles when mode is disabled
  useEffect(() => {
    if (!liveControlEnabled) {
      lastSentAngles.current = null;
    }
  }, [liveControlEnabled]);

  return {
    isSending,
    isActive: liveControlEnabled,
  };
}
