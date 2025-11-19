/**
 * Kinematics Store
 *
 * Manages the headless computation robot used for FK/IK calculations.
 * This robot is never rendered - it's purely for computation.
 *
 * Separation of concerns:
 * - Visual robots (commander, hardware) in RobotViewer
 * - Computation robot (FK/IK) here
 */

import { create } from 'zustand';
import { Tool } from '../types';
import { getDefaultTool } from '../toolManager';

interface KinematicsState {
  // Headless URDF robot for FK/IK computation (never rendered)
  computationRobotRef: any;

  // Tool configuration for FK/IK calculations
  // Usually matches commanderTool, but can be different for what-if scenarios
  computationTool: Tool;

  // Actions
  setComputationRobotRef: (robot: any) => void;
  setComputationTool: (tool: Tool) => void;
}

export const useKinematicsStore = create<KinematicsState>((set) => ({
  // Initial state
  computationRobotRef: null,
  computationTool: getDefaultTool([]), // Fallback until tools loaded

  // Actions
  setComputationRobotRef: (robot) => set({ computationRobotRef: robot }),
  setComputationTool: (tool) => set({ computationTool: tool }),
}));
