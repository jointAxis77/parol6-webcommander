import { useEffect, useRef } from 'react';
import { useTimelineStore } from '@/app/lib/stores/timelineStore';
import { useCommandStore } from '@/app/lib/stores/commandStore';
import { useRobotConfigStore } from '@/app/lib/stores/robotConfigStore';
import { useKinematicsStore } from '@/app/lib/stores/kinematicsStore';
import { useInputStore } from '@/app/lib/stores/inputStore';
import { getJointAnglesAtTime, getCartesianPoseAtTime, shouldUseCartesianInterpolation } from '@/app/lib/interpolation';
import { inverseKinematicsDetailed } from '@/app/lib/kinematics';
import { DEFAULT_FPS } from '@/app/lib/constants';
import { moveJoints, executeTrajectory } from '@/app/lib/api';
import { generateCartesianWaypoints, arrayToPose, poseToArray, calculateWaypointCount } from '@/app/lib/cartesianPlanner';
import { JointAngles, CartesianPose, Tool } from '@/app/lib/types';
import { logger } from '@/app/lib/logger';

/**
 * Solve IK with J1 sweep fallback strategy.
 *
 * If primary seed fails, systematically sweeps J1 (base rotation) in ±10° increments
 * to find alternative arm configurations that can reach the target pose.
 *
 * @param target - Target Cartesian pose
 * @param primarySeed - Primary seed for IK (for continuity)
 * @param robot - URDF robot reference
 * @param tool - Tool configuration
 * @param mask - IK axis mask
 * @returns IK result with metadata about seed used and attempts required
 */
function solveIKWithJ1Sweep(
  target: CartesianPose,
  primarySeed: JointAngles,
  robot: any,
  tool: Tool,
  mask: any
): { success: boolean; jointAngles?: JointAngles; error?: Error; seedUsed?: JointAngles; attemptsRequired?: number } {

  // Try primary seed first (for trajectory continuity)
  let result = inverseKinematicsDetailed(target, primarySeed, robot, tool, mask);
  if (result.success && result.jointAngles) {
    return {
      success: true,
      jointAngles: result.jointAngles,
      seedUsed: primarySeed,
      attemptsRequired: 1
    };
  }

  // J1 sweep: ±15° increments up to ±90° (8 attempts, fast with 30 iter limit)
  const J1_INCREMENT = 15;
  const MAX_J1_OFFSET = 90;
  let attempts = 1;

  for (let offset = J1_INCREMENT; offset <= MAX_J1_OFFSET; offset += J1_INCREMENT) {
    // Try +offset
    attempts++;
    const seedPlus: JointAngles = { ...primarySeed, J1: primarySeed.J1 + offset };
    result = inverseKinematicsDetailed(target, seedPlus, robot, tool, mask);
    if (result.success && result.jointAngles) {
      logger.info(`J1 sweep: +${offset}° (${attempts} attempts)`, 'J1Sweep');
      return {
        success: true,
        jointAngles: result.jointAngles,
        seedUsed: seedPlus,
        attemptsRequired: attempts
      };
    }

    // Try -offset
    attempts++;
    const seedMinus: JointAngles = { ...primarySeed, J1: primarySeed.J1 - offset };
    result = inverseKinematicsDetailed(target, seedMinus, robot, tool, mask);
    if (result.success && result.jointAngles) {
      logger.info(`J1 sweep: -${offset}° (${attempts} attempts)`, 'J1Sweep');
      return {
        success: true,
        jointAngles: result.jointAngles,
        seedUsed: seedMinus,
        attemptsRequired: attempts
      };
    }
  }

  // All J1 sweeps failed
  logger.warn(`J1 sweep failed: ${attempts} attempts exhausted`, 'J1Sweep');
  return {
    success: false,
    error: result.error,
    seedUsed: primarySeed,
    attemptsRequired: attempts
  };
}

/**
 * Pre-calculate Cartesian trajectory using frontend IK with proper seeding.
 *
 * This solves the "jumpy preview" problem by:
 * 1. Generating dense waypoints (100Hz)
 * 2. Solving frontend IK for each waypoint
 * 3. Using PREVIOUS solution as seed for NEXT waypoint (continuity!)
 * 4. Comprehensive logging for debugging
 *
 * @param startPose - Starting Cartesian pose
 * @param endPose - Ending Cartesian pose
 * @param startJoints - Starting joint configuration (seed for first waypoint)
 * @param duration - Segment duration in seconds
 * @param computationRobotRef - URDF computation robot reference
 * @param computationTool - Tool configuration with TCP offset
 * @param ikAxisMask - IK axis mask
 * @returns Joint trajectory or null if IK fails
 */
export async function preCalculateCartesianTrajectory(
  startPose: CartesianPose,
  endPose: CartesianPose,
  startJoints: JointAngles,
  duration: number,
  computationRobotRef: any,
  computationTool: Tool,
  ikAxisMask: any
): Promise<{
  success: boolean;
  trajectory?: number[][];
  failedAt?: number;
  error?: string;
  waypointPoses?: CartesianPose[];
  waypointJoints?: JointAngles[];
  ikValid?: boolean[];
}> {
  const startTime = performance.now();

  // Generate waypoints at 100Hz
  const waypoints = generateCartesianWaypoints(startPose, endPose, { duration });
  const numWaypoints = waypoints.length;

  // Initialize arrays
  const jointTrajectory: number[][] = [];
  const jointAnglesArray: JointAngles[] = [];
  const ikValidArray: boolean[] = [];

  logger.info(`Starting trajectory pre-calc: ${numWaypoints} waypoints`, 'preCalc');

  // Solve IK for each waypoint with progressive seeding and J1 sweep fallback
  let successCount = 0;
  let failureCount = 0;
  let recoveryCount = 0;

  for (let i = 0; i < numWaypoints; i++) {
    const waypoint = waypoints[i];

    // Progressive seeding: use previous solution for continuity
    const seed = i === 0 ? startJoints :
                 (ikValidArray[i-1] ? jointAnglesArray[i-1] : startJoints);

    // Update progress and yield to UI every 10 waypoints
    if (i % 10 === 0) {
      useTimelineStore.getState().setIKProgress({
        current: i,
        total: numWaypoints,
        isCalculating: true,
        recoveries: recoveryCount
      });

      // Yield to UI thread to prevent freezing
      await new Promise(resolve => setTimeout(resolve, 0));
    }

    // Solve with J1 sweep fallback
    const ikResult = solveIKWithJ1Sweep(waypoint, seed, computationRobotRef, computationTool, ikAxisMask);

    if (!ikResult.success || !ikResult.jointAngles) {
      // IK failed - mark as invalid but CONTINUE to process remaining waypoints
      failureCount++;
      logger.warn(`Waypoint ${i}/${numWaypoints} FAILED`, 'preCalc', {
        attempts: ikResult.attemptsRequired,
        error: ikResult.error?.message
      });

      ikValidArray.push(false);
      jointAnglesArray.push(seed);
      jointTrajectory.push([
        seed.J1,
        seed.J2,
        seed.J3,
        seed.J4,
        seed.J5,
        seed.J6
      ]);

      continue;
    }

    // IK succeeded - mark as valid
    successCount++;

    // Track J1 sweep recoveries
    if ((ikResult.attemptsRequired || 1) > 1) {
      recoveryCount++;
      logger.info(`Waypoint ${i} recovered`, 'preCalc', {
        attempts: ikResult.attemptsRequired,
        j1Offset: ikResult.seedUsed ? (ikResult.seedUsed.J1 - seed.J1).toFixed(1) : 0
      });
    }

    ikValidArray.push(true);

    // Store solution
    jointAnglesArray.push({ ...ikResult.jointAngles });
    jointTrajectory.push([
      ikResult.jointAngles.J1,
      ikResult.jointAngles.J2,
      ikResult.jointAngles.J3,
      ikResult.jointAngles.J4,
      ikResult.jointAngles.J5,
      ikResult.jointAngles.J6
    ]);
  }

  // Clear progress when done
  useTimelineStore.getState().setIKProgress({
    current: numWaypoints,
    total: numWaypoints,
    isCalculating: false,
    recoveries: recoveryCount
  });

  logger.info(`Pre-calc complete: ${successCount}/${numWaypoints} succeeded`, 'preCalc', {
    successRate: `${(successCount/numWaypoints*100).toFixed(1)}%`,
    recoveries: recoveryCount,
    recoveryRate: `${(recoveryCount/numWaypoints*100).toFixed(1)}%`,
    totalFailures: failureCount,
    duration: `${(performance.now() - startTime).toFixed(0)}ms`
  });

  // Log pattern of successes/failures to see distribution
  const pattern: string[] = [];
  let currentRun = { valid: ikValidArray[0], count: 0 };
  for (let i = 0; i < ikValidArray.length; i++) {
    if (ikValidArray[i] === currentRun.valid) {
      currentRun.count++;
    } else {
      pattern.push(`${currentRun.count}${currentRun.valid ? 'V' : 'X'}`);
      currentRun = { valid: ikValidArray[i], count: 1 };
    }
  }
  pattern.push(`${currentRun.count}${currentRun.valid ? 'V' : 'X'}`);
  const patternStr = pattern.join(' → ');
  logger.info(`Validity pattern: ${patternStr}`, 'preCalc');

  const elapsed = performance.now() - startTime;

  // Check if any waypoints failed
  const anyFailed = ikValidArray.some(valid => !valid);
  const firstFailedIndex = anyFailed ? ikValidArray.findIndex(valid => !valid) : -1;

  if (anyFailed) {
    logger.warn(`Trajectory complete with ${ikValidArray.filter(v => !v).length} failed waypoints`, 'preCalculateCartesianTrajectory', {
      firstFailed: firstFailedIndex,
      total: numWaypoints,
      elapsed: `${elapsed.toFixed(0)}ms`
    });
  } else {
    logger.info(`Trajectory complete - all ${numWaypoints} waypoints valid`, 'preCalculateCartesianTrajectory', {
      elapsed: `${elapsed.toFixed(0)}ms`
    });
  }

  return {
    success: !anyFailed,
    trajectory: jointTrajectory,
    failedAt: firstFailedIndex >= 0 ? firstFailedIndex : undefined,
    error: anyFailed ? `IK failed at waypoint ${firstFailedIndex}/${numWaypoints}` : undefined,
    waypointPoses: waypoints,
    waypointJoints: jointAnglesArray,
    ikValid: ikValidArray
  };
}

/**
 * Playback loop hook - runs at 60fps when playing
 */
export function usePlayback() {
  const isPlaying = useTimelineStore((state) => state.playbackState.isPlaying);
  const executeOnRobot = useTimelineStore((state) => state.playbackState.executeOnRobot);
  const currentTime = useTimelineStore ((state) => state.playbackState.currentTime);
  const startTime = useTimelineStore((state) => state.playbackState.startTime);
  const duration = useTimelineStore((state) => state.timeline.duration);
  const motionMode = useTimelineStore((state) => state.timeline.mode);
  const keyframes = useTimelineStore((state) => state.timeline.keyframes);
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);
  const ikAxisMask = useRobotConfigStore((state) => state.ikAxisMask);
  const setCurrentTime = useTimelineStore((state) => state.setCurrentTime);
  const pause = useTimelineStore((state) => state.pause);
  const stop = useTimelineStore((state) => state.stop);
  const setPlaybackError = useTimelineStore((state) => state.setPlaybackError);

  // Track last time for keyframe crossing detection
  const lastTime = useRef(0);

  useEffect(() => {
    if (!isPlaying || startTime === null) return;

    // Reset lastTime when playback starts
    lastTime.current = 0;

    const interval = setInterval(() => {
      const elapsed = (Date.now() - startTime) / 1000; // Convert to seconds

      // Calculate last keyframe time
      const lastKeyframeTime = keyframes.length > 0
        ? Math.max(...keyframes.map(kf => kf.time))
        : duration;

      // Pause playback when reaching the last keyframe (don't jump to t=0)
      if (elapsed >= lastKeyframeTime) {
        pause();
        return;
      }

      setCurrentTime(elapsed);

      // Keyframe crossing detection (handles both joint and cartesian execution)
      if (executeOnRobot) {
        // Find all unique keyframe times crossed since last frame
        const uniqueTimes = new Set<number>();
        keyframes.forEach(kf => {
          if (kf.time >= lastTime.current && kf.time <= elapsed) {
            uniqueTimes.add(kf.time);
          }
        });

        // Process each crossed keyframe time
        const sortedTimes = Array.from(uniqueTimes).sort((a, b) => a - b);
        sortedTimes.forEach(async (time) => {
          // Find next keyframe time after this one
          const allKeyframeTimes = [...new Set(keyframes.map(kf => kf.time))];
          const futureKeyframeTimes = allKeyframeTimes.filter(t => t > time);
          const nextKeyframeTime = futureKeyframeTimes.length > 0
            ? Math.min(...futureKeyframeTimes)
            : null;

          if (nextKeyframeTime !== null) {
            const commandDuration = nextKeyframeTime - time;

            // Find the next keyframe to check its motion type
            const nextKeyframe = keyframes.find(kf => kf.time === nextKeyframeTime);

            if (nextKeyframe?.motionType === 'cartesian' && nextKeyframe.cartesianPose) {
              // ========================================
              // CARTESIAN EXECUTION WITH FRONTEND IK
              // ========================================

              // Get start pose (current keyframe's cartesian pose)
              const currentKeyframe = keyframes.find(kf => kf.time === time);
              const startPose = currentKeyframe?.cartesianPose || getCartesianPoseAtTime(keyframes, time);
              const endPose = nextKeyframe.cartesianPose;

              if (!startPose || !computationRobotRef) {
                logger.error('Missing start pose or robot ref', 'usePlayback', { time, nextKeyframeTime });
                pause();
                setPlaybackError('Cartesian execution failed: Missing pose data');
                return;
              }

              // Get current joint angles (seed for IK)
              const startJoints = getJointAnglesAtTime(keyframes, time);

              // Check cache first to avoid redundant IK solving
              const cacheKey = `${currentKeyframe?.id}_${nextKeyframe.id}`;
              logger.debug(`Looking for cache with key: ${cacheKey}`, 'usePlayback');
              logger.debug(`currentKeyframe: ${currentKeyframe?.id} at time ${time}`, 'usePlayback');
              logger.debug(`nextKeyframe: ${nextKeyframe.id} at time ${nextKeyframeTime}`, 'usePlayback');

              const getCachedTrajectory = useTimelineStore.getState().getCachedTrajectory;
              const setCachedTrajectory = useTimelineStore.getState().setCachedTrajectory;

              // Debug: Show all cache keys
              const allCacheKeys = Array.from(useTimelineStore.getState().trajectoryCache.keys());
              logger.debug('All cached keys', 'usePlayback', { allCacheKeys });

              const cachedTrajectory = getCachedTrajectory(cacheKey);
              logger.debug(`Cache found: ${!!cachedTrajectory}`, 'usePlayback');

              // CRITICAL: Cache MUST exist - no fallbacks, no recomputing during playback!
              if (!cachedTrajectory) {
                logger.error('FATAL: No cached trajectory found!', 'usePlayback');
                logger.error(`Requested key: ${cacheKey}`, 'usePlayback');
                logger.error('Available keys', 'usePlayback', { allCacheKeys });
                logger.error('Missing cached trajectory for cartesian segment', 'usePlayback', {
                  cacheKey,
                  availableKeys: allCacheKeys,
                  currentKeyframe: currentKeyframe?.id,
                  nextKeyframe: nextKeyframe.id
                });
                pause();
                setPlaybackError(`FATAL: Cartesian trajectory not pre-calculated for segment ${cacheKey}. Trajectory must be cached before playback (toggle keyframe to cartesian first).`);
                return;
              }

              // Use cached trajectory - convert JointAngles[] to number[][]
              const trajectory = cachedTrajectory.waypointJoints.map(joints => [
                joints.J1, joints.J2, joints.J3, joints.J4, joints.J5, joints.J6
              ]);

              // Check if any waypoint failed IK during pre-calculation
              const anyFailed = cachedTrajectory.ikValid.some(valid => !valid);
              if (anyFailed) {
                const failedIndex = cachedTrajectory.ikValid.findIndex(valid => !valid);
                logger.error('Cached trajectory has failed IK', 'usePlayback', { failedAt: failedIndex });
                pause();
                setPlaybackError(`Cartesian execution failed at waypoint ${failedIndex}: IK solver could not reach target pose (pre-calculated)`);
                return;
              }

              const result = { success: true, trajectory };
              logger.info('Using cached trajectory', 'usePlayback', { cacheKey, waypoints: trajectory.length });

              if (!result.success || !result.trajectory) {
                logger.error('Pre-calculation failed', 'usePlayback', { error: result.error, failedAt: result.failedAt });
                pause();
                setPlaybackError(`Cartesian execution failed at waypoint ${result.failedAt}: ${result.error}`);
                return;
              }

              // Execute trajectory on robot
              try {
                const execResult = await executeTrajectory({
                  trajectory: result.trajectory,
                  duration: commandDuration,
                  wait_for_ack: false
                });

                if (!execResult.success) {
                  logger.error('Execute failed', 'usePlayback', { message: execResult.message });
                  pause();
                  setPlaybackError(`Trajectory execution failed: ${execResult.message}`);
                }
              } catch (error) {
                logger.error('Execute error', 'usePlayback', { error });
                pause();
                setPlaybackError(`Trajectory execution error: ${error}`);
              }

            } else {
              // ========================================
              // JOINT EXECUTION (EXISTING LOGIC)
              // ========================================
              const nextKeyframeAngles = getJointAnglesAtTime(keyframes, nextKeyframeTime);

              // Send move command to NEXT keyframe with duration
              moveJoints(nextKeyframeAngles, undefined, commandDuration).catch(error => {
                logger.error('Failed to send move command', 'usePlayback', { error });
              });
            }
          }
        });
      }

      // Update lastTime for next frame
      lastTime.current = elapsed;

      // Per-keyframe motion type interpolation
      // Check if we should use cartesian interpolation (moving TO a cartesian keyframe)
      const useCartesian = shouldUseCartesianInterpolation(keyframes, elapsed);

      if (useCartesian) {
        // Cartesian preview: Use pre-calculated cached trajectory (NO IK!)
        // Find which segment we're in
        const sortedKeyframes = [...keyframes].sort((a, b) => a.time - b.time);
        let targetSegment = null;

        for (let i = 1; i < sortedKeyframes.length; i++) {
          const prevKf = sortedKeyframes[i - 1];
          const currKf = sortedKeyframes[i];

          if (elapsed >= prevKf.time && elapsed <= currKf.time && currKf.motionType === 'cartesian') {
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
            const segmentProgress = (elapsed - targetSegment.prev.time) / segmentDuration;
            const waypointIndex = Math.floor(segmentProgress * (cachedTrajectory.waypointJoints.length - 1));
            const clampedIndex = Math.max(0, Math.min(waypointIndex, cachedTrajectory.waypointJoints.length - 1));

            // Use pre-calculated joint angles from cache
            const waypointJoints = cachedTrajectory.waypointJoints[clampedIndex];
            useCommandStore.setState({ commandedJointAngles: waypointJoints });
          } else {
            // No cache - fall back to keyframe interpolation
            const interpolatedAngles = getJointAnglesAtTime(keyframes, elapsed);
            useCommandStore.setState({ commandedJointAngles: interpolatedAngles });
          }
        } else {
          // Not in a cartesian segment - use keyframe interpolation
          const interpolatedAngles = getJointAnglesAtTime(keyframes, elapsed);
          useCommandStore.setState({ commandedJointAngles: interpolatedAngles });
        }

        // Update cartesian pose for target visualizer
        const interpolatedPose = getCartesianPoseAtTime(keyframes, elapsed);
        if (interpolatedPose) {
          useInputStore.setState({
            inputCartesianPose: interpolatedPose
          });
        }
      } else {
        // Joint interpolation: Interpolate joint angles directly
        const interpolatedAngles = getJointAnglesAtTime(keyframes, elapsed);
        useCommandStore.setState({ commandedJointAngles: interpolatedAngles });
      }
    }, 1000 / DEFAULT_FPS); // 60fps

    return () => {
      clearInterval(interval);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isPlaying, startTime, duration, motionMode, keyframes, setCurrentTime, stop, executeOnRobot]);
}
