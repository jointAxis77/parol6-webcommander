/**
 * Command Store
 * Commanded robot state - what we're telling the robot to do
 * This is the result of input + IK solving, ready to send to hardware
 */

import { create } from 'zustand';
import type { JointAngles, CartesianPose, JointName, Tool } from '../types';
import { getHomePosition } from '../positions';
import { getDefaultTool } from '../toolManager';

export interface CommandStore {
  // The joint angles we're commanding (either from input directly or from IK result)
  commandedJointAngles: JointAngles;

  // Target robot TCP pose (calculated via FK from commandedJointAngles using URDF)
  // This is what the target robot ACTUALLY achieved (not what user typed)
  commandedTcpPose: CartesianPose | null;

  // URDF reference for commander robot visual (colored robot showing commands)
  commanderRobotRef: any;

  // Tool attached to commander robot (for visualization)
  commanderTool: Tool;

  // Gripper state (for tools with gripper_config)
  commandedGripperState: 'open' | 'closed';

  // Movement parameters
  speed: number;  // Speed percentage (0-100)
  accel: number;  // Acceleration percentage (0-100)

  // Control modes (mutually exclusive)
  liveControlEnabled: boolean;  // Send commands when target changes (was: actualFollowsTarget)
  teachModeEnabled: boolean;    // Target mirrors hardware (was: targetFollowsActual)

  // Joint homing status (tracked for visualization)
  jointHomedStatus: Record<JointName, boolean>;

  // Actions
  setCommandedJointAngles: (angles: JointAngles) => void;
  setCommandedJointAngle: (joint: JointName, angle: number) => void;
  setCommandedTcpPose: (pose: CartesianPose | null) => void;
  setCommanderRobotRef: (ref: any) => void;
  setCommanderTool: (tool: Tool) => void;
  setCommandedGripperState: (state: 'open' | 'closed') => void;
  setSpeed: (speed: number) => void;
  setAccel: (accel: number) => void;
  setLiveControlEnabled: (enabled: boolean) => void;
  setTeachModeEnabled: (enabled: boolean) => void;
  setJointHomed: (joint: JointName, homed: boolean) => void;
}

export const useCommandStore = create<CommandStore>((set) => ({
  // Initial state
  commandedJointAngles: getHomePosition(),
  commandedTcpPose: null,
  commanderRobotRef: null,
  commanderTool: getDefaultTool([]), // Will be updated when config loads
  commandedGripperState: 'open', // Default gripper state

  speed: 80,
  accel: 60,

  liveControlEnabled: false,
  teachModeEnabled: false,

  jointHomedStatus: {
    J1: false,
    J2: false,
    J3: false,
    J4: false,
    J5: false,
    J6: false
  },

  // Actions
  setCommandedJointAngles: (angles) => set({ commandedJointAngles: angles }),

  setCommandedJointAngle: (joint, angle) => {
    set((state) => ({
      commandedJointAngles: {
        ...state.commandedJointAngles,
        [joint]: angle
      }
    }));
  },

  setCommandedTcpPose: (pose) => set({ commandedTcpPose: pose }),

  setCommanderRobotRef: (ref) => set({ commanderRobotRef: ref }),

  setCommanderTool: (tool) => set({ commanderTool: tool }),

  setCommandedGripperState: (state) => set({ commandedGripperState: state }),

  setSpeed: (speed) => set({ speed }),

  setAccel: (accel) => set({ accel }),

  setLiveControlEnabled: (enabled) => {
    set((state) => ({
      liveControlEnabled: enabled,
      // Disable teach mode if enabling live control (mutually exclusive)
      teachModeEnabled: enabled ? false : state.teachModeEnabled
    }));
  },

  setTeachModeEnabled: (enabled) => {
    set((state) => ({
      teachModeEnabled: enabled,
      // Disable live control if enabling teach mode (mutually exclusive)
      liveControlEnabled: enabled ? false : state.liveControlEnabled
    }));
  },

  setJointHomed: (joint, homed) => {
    set((state) => ({
      jointHomedStatus: {
        ...state.jointHomedStatus,
        [joint]: homed
      }
    }));
  }
}));
