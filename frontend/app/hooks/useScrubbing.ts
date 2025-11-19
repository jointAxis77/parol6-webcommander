import { useEffect } from 'react';
import { useTimelineStore } from '@/app/lib/stores/timelineStore';
import { useCommandStore } from '@/app/lib/stores/commandStore';
import { useRobotConfigStore } from '@/app/lib/stores/robotConfigStore';
import { useKinematicsStore } from '@/app/lib/stores/kinematicsStore';
import { useInputStore } from '@/app/lib/stores/inputStore';
import { getJointAnglesAtTime, getCartesianPoseAtTime, shouldUseCartesianInterpolation } from '@/app/lib/interpolation';

/**
 * Scrubbing hook - updates robot position when timeline playhead is dragged
 * Only active when NOT playing (playback handles updates during play)
 */
export function useScrubbing() {
  const isPlaying = useTimelineStore((state) => state.playbackState.isPlaying);
  const currentTime = useTimelineStore((state) => state.playbackState.currentTime);
  const motionMode = useTimelineStore((state) => state.timeline.mode);
  const keyframes = useTimelineStore((state) => state.timeline.keyframes);
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);
  const ikAxisMask = useRobotConfigStore((state) => state.ikAxisMask);

  useEffect(() => {
    // Skip if actively playing (usePlayback handles interpolation)
    if (isPlaying) return;

    // Per-keyframe motion type interpolation
    // Check if we should use cartesian interpolation (moving TO a cartesian keyframe)
    const useCartesian = shouldUseCartesianInterpolation(keyframes, currentTime);

    if (useCartesian) {
      // Cartesian scrubbing: Use pre-calculated cached trajectory (NO IK!)
      // Find which segment we're in
      const sortedKeyframes = [...keyframes].sort((a, b) => a.time - b.time);
      let targetSegment = null;

      for (let i = 1; i < sortedKeyframes.length; i++) {
        const prevKf = sortedKeyframes[i - 1];
        const currKf = sortedKeyframes[i];

        if (currentTime >= prevKf.time && currentTime <= currKf.time && currKf.motionType === 'cartesian') {
          targetSegment = { prev: prevKf, curr: currKf, index: i };
          break;
        }
      }

      if (targetSegment) {
        // Get cached trajectory for this segment
        const cacheKey = `${targetSegment.prev.id}_${targetSegment.curr.id}`;
        const cachedTrajectory = useTimelineStore.getState().getCachedTrajectory(cacheKey);

        if (cachedTrajectory && cachedTrajectory.waypointJoints) {
          // Interpolate within cached waypoints
          const segmentDuration = targetSegment.curr.time - targetSegment.prev.time;
          const segmentProgress = (currentTime - targetSegment.prev.time) / segmentDuration;
          const waypointIndex = Math.floor(segmentProgress * (cachedTrajectory.waypointJoints.length - 1));
          const clampedIndex = Math.max(0, Math.min(waypointIndex, cachedTrajectory.waypointJoints.length - 1));

          // Use pre-calculated joint angles from cache
          const waypointJoints = cachedTrajectory.waypointJoints[clampedIndex];
          useCommandStore.setState({ commandedJointAngles: waypointJoints });
        } else {
          // No cache - fall back to keyframe interpolation
          const interpolatedAngles = getJointAnglesAtTime(keyframes, currentTime);
          useCommandStore.setState({ commandedJointAngles: interpolatedAngles });
        }
      } else {
        // Not in a cartesian segment - use keyframe interpolation
        const interpolatedAngles = getJointAnglesAtTime(keyframes, currentTime);
        useCommandStore.setState({ commandedJointAngles: interpolatedAngles });
      }

      // Update cartesian pose for target visualizer
      const interpolatedPose = getCartesianPoseAtTime(keyframes, currentTime);
      if (interpolatedPose) {
        useInputStore.setState({
          inputCartesianPose: interpolatedPose
        });
      }
    } else {
      // Joint interpolation: Interpolate joint angles directly
      const interpolatedAngles = getJointAnglesAtTime(keyframes, currentTime);
      useCommandStore.setState({ commandedJointAngles: interpolatedAngles });
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [currentTime, keyframes, motionMode, isPlaying]);
}
