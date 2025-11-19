/**
 * Timeline Store
 * Timeline editing and playback state
 * Manages keyframes, playback control, and timeline metadata
 */

import { create } from 'zustand';
import type { Keyframe, Timeline, PlaybackState, MotionMode, JointName, JointAngles, CartesianPose } from '../types';
import { DEFAULT_DURATION, JOINT_NAMES } from '../constants';
import { getHomePosition } from '../positions';
import { v4 as uuidv4 } from 'uuid';
import { useCommandStore } from './commandStore';
import { useInputStore } from './inputStore';

/**
 * Cached trajectory for a cartesian motion segment
 * Pre-computed to eliminate duplicate IK solving during preview and playback
 */
export interface CachedTrajectory {
  waypointPoses: CartesianPose[];     // All waypoints at 100Hz
  waypointJoints: JointAngles[];      // IK-solved joint angles for each waypoint
  ikValid: boolean[];                 // Success status for each waypoint
  dependencyHash: string;             // Hash of tcpOffset + ikAxisMask + poses + duration
  computedAt: number;                 // Timestamp of computation
}

/**
 * IK progress state for trajectory pre-calculation
 * Used to display progress bar during IK solving
 */
export interface IKProgressState {
  isCalculating: boolean;
  current: number;
  total: number;
  recoveries?: number;
}

export interface TimelineStore {
  // Timeline data
  timeline: Timeline;

  // Playback state
  playbackState: PlaybackState;

  // Trajectory cache (for cartesian moves)
  trajectoryCache: Map<string, CachedTrajectory>;

  // IK progress state (for trajectory pre-calculation)
  ikProgress: IKProgressState;

  // Actions - Keyframe management
  addKeyframe: (time: number, jointAngles: JointAngles, cartesianPose?: CartesianPose) => void;
  removeKeyframe: (id: string) => void;
  updateKeyframe: (id: string, updates: Partial<Keyframe>) => void;
  recordKeyframes: (jointAngles: JointAngles, cartesianPose?: CartesianPose) => void;

  // Actions - Motion type management
  toggleKeyframeMotionType: (keyframeIds: string[]) => void;
  updateKeyframeValues: (id: string, jointAngles: JointAngles, cartesianPose: CartesianPose) => void;

  // Actions - Playback control
  setCurrentTime: (time: number) => void;
  play: (executeOnRobot?: boolean) => void;
  pause: () => void;
  stop: () => void;
  setPlaybackError: (error: string | null) => void;
  clearPlaybackError: () => void;

  // Actions - Timeline management
  setMotionMode: (mode: MotionMode) => void;
  loadTimeline: (timeline: Timeline) => void;
  exportTimeline: () => string;

  // Actions - Trajectory cache management
  setCachedTrajectory: (key: string, trajectory: CachedTrajectory) => void;
  getCachedTrajectory: (key: string) => CachedTrajectory | null;
  invalidateTrajectoryCache: (keyframeId?: string) => void;

  // Actions - IK progress management
  setIKProgress: (progress: IKProgressState) => void;
}

export const useTimelineStore = create<TimelineStore>((set, get) => ({
  // Initial state
  timeline: {
    name: 'Untitled',
    mode: 'joint',
    keyframes: [],
    duration: DEFAULT_DURATION
  },

  playbackState: {
    isPlaying: false,
    currentTime: 0,
    startTime: null,
    loop: false,
    executeOnRobot: false,
    playbackError: null
  },

  trajectoryCache: new Map<string, CachedTrajectory>(),

  ikProgress: {
    isCalculating: false,
    current: 0,
    total: 0,
    recoveries: 0
  },

  // Keyframe management actions
  addKeyframe: (time, jointAngles, cartesianPose) => {
    set((state) => ({
      timeline: {
        ...state.timeline,
        keyframes: [
          ...state.timeline.keyframes,
          {
            id: uuidv4(),
            time,
            jointAngles,
            ...(cartesianPose && { cartesianPose })
          }
        ].sort((a, b) => a.time - b.time) // Keep sorted by time
      }
    }));
  },

  removeKeyframe: (id) => {
    set((state) => ({
      timeline: {
        ...state.timeline,
        keyframes: state.timeline.keyframes.filter((kf) => kf.id !== id)
      }
    }));
  },

  updateKeyframe: (id, updates) => {
    set((state) => ({
      timeline: {
        ...state.timeline,
        keyframes: state.timeline.keyframes
          .map((kf) => (kf.id === id ? { ...kf, ...updates } : kf))
          .sort((a, b) => a.time - b.time) // Re-sort if time was updated
      }
    }));
  },

  recordKeyframes: (jointAngles, cartesianPose) => {
    const state = get();
    const currentTime = state.playbackState.currentTime;

    // Get current tool state from commandStore
    const commandStore = useCommandStore.getState();
    const toolId = commandStore.commanderTool.id;
    const gripperState = commandStore.commandedGripperState;

    // Record or update a single keyframe (containing all 6 joint angles) at current time
    // If keyframe already exists at this time, update it instead of creating new
    const existingKeyframe = state.timeline.keyframes.find(
      (kf) => Math.abs(kf.time - currentTime) < 0.001
    );

    if (existingKeyframe) {
      // Update existing keyframe with new joint angles, cartesian pose, and tool state
      const updates: Partial<Keyframe> = {
        jointAngles,
        toolId,
        gripperState
      };
      if (cartesianPose) {
        updates.cartesianPose = cartesianPose;
      }
      state.updateKeyframe(existingKeyframe.id, updates);
    } else {
      // Create new keyframe with cartesianPose and tool state atomically
      state.addKeyframe(currentTime, jointAngles, cartesianPose);

      // Update the newly created keyframe with tool state
      const newKeyframe = state.timeline.keyframes.find(
        (kf) => Math.abs(kf.time - currentTime) < 0.001
      );
      if (newKeyframe) {
        state.updateKeyframe(newKeyframe.id, { toolId, gripperState });
      }
    }
  },

  // Motion type management actions
  toggleKeyframeMotionType: (keyframeIds) => {
    set((state) => ({
      timeline: {
        ...state.timeline,
        keyframes: state.timeline.keyframes.map(kf => {
          if (!keyframeIds.includes(kf.id)) return kf;

          // Toggle motion type
          const currentType = kf.motionType || 'joint';
          const newType = currentType === 'joint' ? 'cartesian' : 'joint';

          return {
            ...kf,
            motionType: newType
          };
        })
      }
    }));
  },

  updateKeyframeValues: (id, jointAngles, cartesianPose) => {
    set((state) => ({
      timeline: {
        ...state.timeline,
        keyframes: state.timeline.keyframes.map(kf =>
          kf.id === id
            ? { ...kf, jointAngles, cartesianPose }
            : kf
        )
      }
    }));

    // Invalidate cache for segments involving this keyframe
    get().invalidateTrajectoryCache(id);
  },

  // Playback control actions
  setCurrentTime: (time) => {
    set((state) => ({
      playbackState: {
        ...state.playbackState,
        currentTime: time
      }
    }));
  },

  play: (executeOnRobot = false) => {
    set((state) => ({
      playbackState: {
        ...state.playbackState,
        isPlaying: true,
        executeOnRobot,
        startTime: Date.now() - state.playbackState.currentTime * 1000,
        playbackError: null  // Clear any previous errors
      }
    }));
  },

  pause: () => {
    set((state) => ({
      playbackState: {
        ...state.playbackState,
        isPlaying: false
      }
    }));

    // Sync input gizmo to commanded TCP pose (same as "Copy From Robot TCP" button)
    const commandedTcpPose = useCommandStore.getState().commandedTcpPose;
    if (commandedTcpPose) {
      useInputStore.setState({
        inputCartesianPose: { ...commandedTcpPose }
      });
    }
  },

  stop: () => {
    set({
      playbackState: {
        isPlaying: false,
        currentTime: 0,
        startTime: null,
        loop: false,
        executeOnRobot: false,
        playbackError: null  // Clear any errors
      }
    });
  },

  setPlaybackError: (error) => {
    set((state) => ({
      playbackState: {
        ...state.playbackState,
        playbackError: error
      }
    }));
  },

  clearPlaybackError: () => {
    set((state) => ({
      playbackState: {
        ...state.playbackState,
        playbackError: null
      }
    }));
  },

  // Timeline management
  setMotionMode: (mode) => {
    const state = get();

    // Stop playback before mode switch
    if (state.playbackState.isPlaying) {
      state.stop();
    }

    // Mode is just a display preference - keyframes remain unchanged
    set((state) => ({
      timeline: {
        ...state.timeline,
        mode
      },
      playbackState: {
        isPlaying: false,
        currentTime: 0,
        startTime: null,
        loop: false,
        executeOnRobot: false
      }
    }));
  },

  loadTimeline: (timeline) => {
    // Migration: Remove cartesianKeyframes from legacy timeline files
    const { cartesianKeyframes, ...timelineWithoutCartesian } = timeline as any;

    // Migration: Convert old per-joint keyframes to new single keyframe format
    const keyframes = timelineWithoutCartesian.keyframes || [];

    // Check if this is old format (has 'joint' and 'value' fields)
    const isOldFormat = keyframes.length > 0 && 'joint' in keyframes[0] && 'value' in keyframes[0];

    if (isOldFormat) {
      // Group old keyframes by time
      const keyframesByTime = new Map<number, { id?: string; time: number; jointAngles: Partial<JointAngles> }>();

      keyframes.forEach((kf: any) => {
        const timeKey = Math.round(kf.time * 1000) / 1000; // Round to 3 decimals for grouping

        if (!keyframesByTime.has(timeKey)) {
          keyframesByTime.set(timeKey, {
            id: kf.id, // Use first keyframe's ID
            time: kf.time,
            jointAngles: {}
          });
        }

        const group = keyframesByTime.get(timeKey)!;
        group.jointAngles[kf.joint as JointName] = kf.value;
      });

      // Convert to new format, filling missing joints with home position
      const homePosition = getHomePosition();
      const newKeyframes: Keyframe[] = Array.from(keyframesByTime.values()).map(group => ({
        id: group.id || uuidv4(),
        time: group.time,
        jointAngles: {
          J1: group.jointAngles.J1 ?? homePosition.J1,
          J2: group.jointAngles.J2 ?? homePosition.J2,
          J3: group.jointAngles.J3 ?? homePosition.J3,
          J4: group.jointAngles.J4 ?? homePosition.J4,
          J5: group.jointAngles.J5 ?? homePosition.J5,
          J6: group.jointAngles.J6 ?? homePosition.J6
        }
      }));

      set({ timeline: { ...timelineWithoutCartesian, keyframes: newKeyframes } as Timeline });
    } else {
      // Already new format
      set({ timeline: timelineWithoutCartesian as Timeline });
    }
  },

  exportTimeline: () => {
    return JSON.stringify(get().timeline, null, 2);
  },

  // Trajectory cache management
  setCachedTrajectory: (key, trajectory) => {
    set((state) => {
      const newCache = new Map(state.trajectoryCache);
      newCache.set(key, trajectory);
      return { trajectoryCache: newCache };
    });
  },

  getCachedTrajectory: (key) => {
    const cache = get().trajectoryCache;
    return cache.get(key) || null;
  },

  invalidateTrajectoryCache: (keyframeId) => {
    set((state) => {
      if (!keyframeId) {
        // Clear entire cache
        return { trajectoryCache: new Map<string, CachedTrajectory>() };
      }

      // Clear cache entries involving this keyframe
      // Cache keys are in format: "startKeyframeId_endKeyframeId"
      const newCache = new Map(state.trajectoryCache);
      for (const key of newCache.keys()) {
        if (key.includes(keyframeId)) {
          newCache.delete(key);
        }
      }
      return { trajectoryCache: newCache };
    });
  },

  // IK progress management
  setIKProgress: (progress) => {
    set({ ikProgress: progress });
  }
}));
