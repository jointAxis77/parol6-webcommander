/**
 * User Input Store
 * Raw UI input state from sliders and keyboard controls
 * This is what the user directly manipulates, before IK or command processing
 */

import { create } from 'zustand';
import type { JointAngles, CartesianPose, JointName } from '../types';
import { getHomePosition } from '../positions';

export interface InputStore {
  // Joint mode: Direct joint values from sliders/keyboard
  inputJointAngles: JointAngles;

  // Cartesian mode: Slider values BEFORE IK is solved
  inputCartesianPose: CartesianPose;

  // UI state
  selectedJoint: JointName | null;
  stepAngle: number;
  cartesianPositionStep: number;
  showTargetRobot: boolean;
  showHardwareRobot: boolean;  // Renamed from showActualRobot
  showFirmwareCoordinates: boolean;
  showPath: boolean;

  // Actions
  setInputJointAngle: (joint: JointName, angle: number) => void;
  setInputCartesianValue: (axis: keyof CartesianPose, value: number) => void;
  setSelectedJoint: (joint: JointName | null) => void;
  setStepAngle: (angle: number) => void;
  setCartesianPositionStep: (step: number) => void;
  setShowTargetRobot: (show: boolean) => void;
  setShowHardwareRobot: (show: boolean) => void;
  setShowFirmwareCoordinates: (show: boolean) => void;
  setShowPath: (show: boolean) => void;
}

export const useInputStore = create<InputStore>((set) => ({
  // Initial state
  inputJointAngles: getHomePosition(),
  inputCartesianPose: {
    X: 0,
    Y: 0,
    Z: 300, // Default height above base
    RX: 0,
    RY: 0,
    RZ: 0
  },

  selectedJoint: null,
  stepAngle: 1.0,
  cartesianPositionStep: 1,
  showTargetRobot: true,
  showHardwareRobot: true,
  showFirmwareCoordinates: false,
  showPath: true,

  // Actions
  setInputJointAngle: (joint, angle) => {
    set((state) => ({
      inputJointAngles: {
        ...state.inputJointAngles,
        [joint]: angle
      },
      // Auto-enable target robot when slider is moved
      showTargetRobot: true
    }));
  },

  setInputCartesianValue: (axis, value) => {
    set((state) => ({
      inputCartesianPose: {
        ...state.inputCartesianPose,
        [axis]: value
      }
    }));
  },

  setSelectedJoint: (joint) => set({ selectedJoint: joint }),
  setStepAngle: (angle) => set({ stepAngle: angle }),
  setCartesianPositionStep: (step) => set({ cartesianPositionStep: step }),
  setShowTargetRobot: (show) => set({ showTargetRobot: show }),
  setShowHardwareRobot: (show) => set({ showHardwareRobot: show }),
  setShowFirmwareCoordinates: (show) => set({ showFirmwareCoordinates: show }),
  setShowPath: (show) => set({ showPath: show })
}));
