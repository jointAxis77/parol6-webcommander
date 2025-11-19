'use client';

import { useState, useEffect } from 'react';
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Slider } from '@/components/ui/slider';
import { useTimelineStore } from '@/app/lib/stores/timelineStore';
import { useKinematicsStore } from '@/app/lib/stores/kinematicsStore';
import { useRobotConfigStore } from '@/app/lib/stores/robotConfigStore';
import { JOINT_NAMES, JOINT_LIMITS, CARTESIAN_AXES, CARTESIAN_LIMITS } from '@/app/lib/constants';
import { JointAngles, CartesianPose, JointName, CartesianAxis } from '@/app/lib/types';
import { inverseKinematicsDetailed } from '@/app/lib/kinematics';
import { calculateTcpPoseFromUrdf } from '@/app/lib/tcpCalculations';
import { threeJsToRobot } from '@/app/lib/coordinateTransform';
import { applyJointAnglesToUrdf } from '@/app/lib/urdfHelpers';
import { ArrowLeftRight, Calculator, AlertCircle } from 'lucide-react';
import { logger } from '@/app/lib/logger';

interface KeyframeEditDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  keyframeId: string | null;
}

export default function KeyframeEditDialog({
  open,
  onOpenChange,
  keyframeId
}: KeyframeEditDialogProps) {
  const keyframes = useTimelineStore((state) => state.timeline.keyframes);
  const updateKeyframeValues = useTimelineStore((state) => state.updateKeyframeValues);
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);
  const tcpOffset = useRobotConfigStore((state) => state.tcpOffset);
  const ikAxisMask = useRobotConfigStore((state) => state.ikAxisMask);

  // Find the keyframe being edited
  const keyframe = keyframes.find(kf => kf.id === keyframeId);

  // Local state for editing
  const [localJointAngles, setLocalJointAngles] = useState<JointAngles | null>(null);
  const [localCartesianPose, setLocalCartesianPose] = useState<CartesianPose | null>(null);
  const [ikError, setIkError] = useState<string | null>(null);
  const [fkError, setFkError] = useState<string | null>(null);

  // Track input field values for typing
  const [inputValues, setInputValues] = useState<Record<string, string>>({});

  // Initialize local state when keyframe changes
  useEffect(() => {
    if (keyframe) {
      // Test frontend → backend logging
      logger.info(`Opening keyframe edit dialog for keyframe at ${keyframe.time.toFixed(2)}s`, 'KeyframeEditDialog');

      setLocalJointAngles(keyframe.jointAngles);

      // Use stored cartesian pose if available, otherwise will compute on FK button click
      setLocalCartesianPose(keyframe.cartesianPose || {
        X: 0, Y: 0, Z: 300, RX: 0, RY: 0, RZ: 0
      });

      setIkError(null);
      setFkError(null);
      setInputValues({});
    }
  }, [keyframe, computationRobotRef, computationTool]);

  if (!keyframe || !localJointAngles || !localCartesianPose) {
    return null;
  }

  // Handle joint angle changes
  const handleJointChange = (joint: JointName, value: number) => {
    const newJointAngles = { ...localJointAngles, [joint]: value };
    setLocalJointAngles(newJointAngles);
    setInputValues({ ...inputValues, [joint]: '' });
  };

  // Handle cartesian value changes
  const handleCartesianChange = (axis: CartesianAxis, value: number) => {
    const newCartesianPose = { ...localCartesianPose, [axis]: value };
    setLocalCartesianPose(newCartesianPose);
    setInputValues({ ...inputValues, [axis]: '' });
  };

  // Sync cartesian to joint (FK)
  const handleFK = () => {
    if (!computationRobotRef) {
      setFkError('Robot model not loaded');
      return;
    }

    setFkError(null);

    try {
      // Apply joint angles to computation robot using centralized helper
      applyJointAnglesToUrdf(computationRobotRef, localJointAngles);

      // Compute FK from updated robot pose
      const fkPoseThreeJs = calculateTcpPoseFromUrdf(computationRobotRef, computationTool.tcp_offset);
      if (fkPoseThreeJs) {
        // Convert from Three.js coordinates (Y-up) to robot coordinates (Z-up)
        const fkPoseRobot = threeJsToRobot(fkPoseThreeJs);
        setLocalCartesianPose(fkPoseRobot);
      } else {
        setFkError('FK computation failed');
      }
    } catch (error) {
      setFkError(error instanceof Error ? error.message : 'FK failed');
    }
  };

  // Sync joint to cartesian (IK)
  const handleIK = () => {
    if (!computationRobotRef) {
      setIkError('Robot model not loaded');
      return;
    }

    setIkError(null);

    const ikResult = inverseKinematicsDetailed(
      localCartesianPose,
      localJointAngles,
      computationRobotRef,
      computationTool,
      ikAxisMask
    );

    if (ikResult.success && ikResult.jointAngles) {
      setLocalJointAngles(ikResult.jointAngles);
    } else {
      setIkError(ikResult.error?.message || 'IK failed');
    }
  };

  // Save changes
  const handleSave = () => {
    updateKeyframeValues(keyframe.id, localJointAngles, localCartesianPose);
    onOpenChange(false);
  };

  // Cancel changes
  const handleCancel = () => {
    onOpenChange(false);
  };

  const getUnit = (axis: CartesianAxis) => {
    return ['X', 'Y', 'Z'].includes(axis) ? 'mm' : '°';
  };

  const handleInputChange = (key: string, value: string) => {
    setInputValues({ ...inputValues, [key]: value });
  };

  const handleInputBlur = (key: string, isJoint: boolean) => {
    const value = inputValues[key];
    if (value !== undefined && value !== '') {
      const numValue = parseFloat(value);
      if (!isNaN(numValue)) {
        if (isJoint) {
          const joint = key as JointName;
          const limits = JOINT_LIMITS[joint];
          const clampedValue = Math.max(limits.min, Math.min(limits.max, numValue));
          handleJointChange(joint, clampedValue);
        } else {
          const axis = key as CartesianAxis;
          const limits = CARTESIAN_LIMITS[axis];
          const clampedValue = Math.max(limits.min, Math.min(limits.max, numValue));
          handleCartesianChange(axis, clampedValue);
        }
      }
    }
    setInputValues({ ...inputValues, [key]: '' });
  };

  const handleInputKeyDown = (key: string, isJoint: boolean, e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      (e.target as HTMLInputElement).blur();
    }
  };

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
        <DialogHeader>
          <DialogTitle>Edit Keyframe at {keyframe.time.toFixed(2)}s</DialogTitle>
          <DialogDescription>
            Adjust joint angles or cartesian pose. Use IK/FK to sync between them.
          </DialogDescription>
        </DialogHeader>

        <div className="grid grid-cols-[1fr_auto_1fr] gap-6 mt-4">
          {/* Left Column: Joint Angles */}
          <div className="space-y-4">
            <h3 className="text-sm font-semibold mb-3">Joint Angles</h3>
            {JOINT_NAMES.map((joint) => {
              const limits = JOINT_LIMITS[joint];
              const currentValue = localJointAngles[joint];
              const displayValue = inputValues[joint] !== undefined && inputValues[joint] !== ''
                ? inputValues[joint]
                : currentValue.toFixed(1);

              return (
                <div key={joint} className="space-y-2">
                  <div className="flex justify-between items-center text-sm gap-2">
                    <span className="font-medium w-8">{joint}</span>
                    <div className="flex items-center gap-1">
                      <Input
                        type="text"
                        value={displayValue}
                        onChange={(e) => handleInputChange(joint, e.target.value)}
                        onBlur={() => handleInputBlur(joint, true)}
                        onKeyDown={(e) => handleInputKeyDown(joint, true, e)}
                        className="w-20 h-7 px-2 text-xs font-mono text-right"
                      />
                      <span className="text-xs text-muted-foreground w-4">°</span>
                    </div>
                  </div>
                  <Slider
                    min={limits.min}
                    max={limits.max}
                    step={0.1}
                    value={[currentValue]}
                    onValueChange={(values) => handleJointChange(joint, values[0])}
                    className="w-full"
                  />
                </div>
              );
            })}
          </div>

          {/* Middle Column: Sync Buttons */}
          <div className="flex flex-col justify-center items-center gap-4 min-w-[100px]">
            <Button
              onClick={handleFK}
              variant="outline"
              size="sm"
              className="w-full"
              title="Forward Kinematics: Compute cartesian pose from joint angles"
            >
              <ArrowLeftRight className="w-4 h-4 mr-2" />
              FK →
            </Button>

            <div className="text-xs text-muted-foreground text-center">
              Sync
            </div>

            <Button
              onClick={handleIK}
              variant="outline"
              size="sm"
              className="w-full"
              title="Inverse Kinematics: Compute joint angles from cartesian pose"
            >
              <Calculator className="w-4 h-4 mr-2" />
              ← IK
            </Button>

            {ikError && (
              <div className="bg-red-500/10 border border-red-500/50 rounded p-2 flex items-start gap-2">
                <AlertCircle className="w-3 h-3 text-red-500 mt-0.5 flex-shrink-0" />
                <div className="text-xs text-red-400">{ikError}</div>
              </div>
            )}

            {fkError && (
              <div className="bg-red-500/10 border border-red-500/50 rounded p-2 flex items-start gap-2">
                <AlertCircle className="w-3 h-3 text-red-500 mt-0.5 flex-shrink-0" />
                <div className="text-xs text-red-400">{fkError}</div>
              </div>
            )}
          </div>

          {/* Right Column: Cartesian Pose */}
          <div className="space-y-4">
            <h3 className="text-sm font-semibold mb-3">Cartesian Pose</h3>
            {CARTESIAN_AXES.map((axis) => {
              const limits = CARTESIAN_LIMITS[axis];
              const unit = getUnit(axis);
              const step = ['X', 'Y', 'Z'].includes(axis) ? 1 : 0.1;
              const currentValue = localCartesianPose[axis];
              const displayValue = inputValues[axis] !== undefined && inputValues[axis] !== ''
                ? inputValues[axis]
                : currentValue.toFixed(1);

              return (
                <div key={axis} className="space-y-2">
                  <div className="flex justify-between items-center text-sm gap-2">
                    <span className="font-medium w-8">{axis}</span>
                    <div className="flex items-center gap-1">
                      <Input
                        type="text"
                        value={displayValue}
                        onChange={(e) => handleInputChange(axis, e.target.value)}
                        onBlur={() => handleInputBlur(axis, false)}
                        onKeyDown={(e) => handleInputKeyDown(axis, false, e)}
                        className="w-20 h-7 px-2 text-xs font-mono text-right"
                      />
                      <span className="text-xs text-muted-foreground w-8">{unit}</span>
                    </div>
                  </div>
                  <Slider
                    min={limits.min}
                    max={limits.max}
                    step={step}
                    value={[currentValue]}
                    onValueChange={(values) => handleCartesianChange(axis, values[0])}
                    className="w-full"
                  />
                </div>
              );
            })}
          </div>
        </div>

        {/* Footer: Save/Cancel */}
        <div className="flex justify-end gap-2 mt-6 pt-4 border-t">
          <Button variant="outline" onClick={handleCancel}>
            Cancel
          </Button>
          <Button onClick={handleSave}>
            Save Changes
          </Button>
        </div>
      </DialogContent>
    </Dialog>
  );
}
