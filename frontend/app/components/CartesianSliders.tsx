'use client';

import { useState } from 'react';
import { useInputStore, useCommandStore, useRobotConfigStore } from '@/app/lib/stores';
import { useKinematicsStore } from '@/app/lib/stores/kinematicsStore';
import { CARTESIAN_AXES, CARTESIAN_LIMITS } from '@/app/lib/constants';
import { Slider } from '@/components/ui/slider';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Checkbox } from '@/components/ui/checkbox';
import { CartesianAxis, IkAxisMask } from '@/app/lib/types';
import { inverseKinematicsDetailed } from '@/app/lib/kinematics';
import { AlertCircle, CheckCircle, Calculator, Copy, ChevronLeft, ChevronRight } from 'lucide-react';
import { logger } from '@/app/lib/logger';

export default function CartesianSliders() {
  // Input store: What user is typing/moving in sliders
  const inputCartesianPose = useInputStore((state) => state.inputCartesianPose);
  const setInputCartesianValue = useInputStore((state) => state.setInputCartesianValue);
  const cartesianPositionStep = useInputStore((state) => state.cartesianPositionStep);

  // Command store: Commanded joint angles (for IK seed and result)
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const setCommandedJointAngles = useCommandStore((state) => state.setCommandedJointAngles);
  const commandedTcpPose = useCommandStore((state) => state.commandedTcpPose);

  // Kinematics store: Computation robot and tool
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);

  // Config store: TCP offset and IK mask
  const tcpOffset = useRobotConfigStore((state) => state.tcpOffset);
  const ikAxisMask = useRobotConfigStore((state) => state.ikAxisMask);
  const setIkAxisMask = useRobotConfigStore((state) => state.setIkAxisMask);

  const [ikStatus, setIkStatus] = useState<{
    type: 'idle' | 'computing' | 'success' | 'error';
    message?: string;
    distance?: number;
    iterations?: number;
  }>({ type: 'idle' });

  // Track input field values separately to allow editing
  const [inputValues, setInputValues] = useState<Record<string, string>>({});

  // Handle slider changes: ONLY update input pose, NO IK computation
  // IK will be computed later during timeline playback
  const handleSliderChange = (axis: CartesianAxis, value: number) => {
    setInputCartesianValue(axis, value);
    // Clear IK status when user changes target
    if (ikStatus.type !== 'idle') {
      setIkStatus({ type: 'idle' });
    }
  };

  // Handle increment/decrement buttons
  const handleStepAxis = (axis: CartesianAxis, direction: number) => {
    const currentValue = inputCartesianPose[axis];
    const limits = CARTESIAN_LIMITS[axis];
    const step = ['X', 'Y', 'Z'].includes(axis) ? cartesianPositionStep : 0.1; // Position step for X/Y/Z, 0.1° for rotations
    const newValue = Math.max(limits.min, Math.min(limits.max, currentValue + (direction * step)));

    setInputCartesianValue(axis, newValue);
    // Clear IK status when user changes target
    if (ikStatus.type !== 'idle') {
      setIkStatus({ type: 'idle' });
    }
  };

  // Compute IK on demand using numerical solver
  const handleComputeIK = () => {
    logger.debug('Starting IK computation...', 'IKSolve');
    setIkStatus({ type: 'computing' });

    if (!computationRobotRef) {
      logger.error('FAILED: Computation robot not loaded', 'IKSolve');
      setIkStatus({
        type: 'error',
        message: 'URDF robot model not loaded yet. Please wait...'
      });
      return;
    }

    logger.debug('Inputs', 'IKSolve', {
      targetPose: inputCartesianPose,
      seedJoints: commandedJointAngles,
      tool: {
        id: computationTool.id,
        name: computationTool.name,
        tcp_offset: computationTool.tcp_offset
      },
      ikAxisMask
    });

    // Small delay to show loading state
    setTimeout(() => {
      // Target is TCP position - numerical IK handles TCP offset internally
      logger.debug('Calling inverseKinematicsDetailed...', 'IKSolve');
      const ikResult = inverseKinematicsDetailed(
        inputCartesianPose,
        commandedJointAngles,
        computationRobotRef,
        computationTool,
        ikAxisMask
      );

      logger.debug('Result', 'IKSolve', ikResult);

      if (ikResult.success && ikResult.jointAngles) {
        // Update commanded joint angles - robot will move to match target
        logger.debug('SUCCESS', 'IKSolve', {
          jointAngles: ikResult.jointAngles,
          iterations: ikResult.iterations,
          finalError: ikResult.finalError
        });
        setCommandedJointAngles(ikResult.jointAngles);
        setIkStatus({
          type: 'success',
          message: `Converged in ${ikResult.iterations} iterations (error: ${ikResult.finalError?.toFixed(2)}mm)`,
          iterations: ikResult.iterations
        });
      } else {
        logger.error('FAILED', 'IKSolve', {
          error: ikResult.error,
          iterations: ikResult.iterations,
          finalError: ikResult.finalError
        });
        setIkStatus({
          type: 'error',
          message: ikResult.error?.message || 'IK failed',
          distance: ikResult.error?.distance,
          iterations: ikResult.iterations
        });
      }
    }, 50);
  };

  const getUnit = (axis: CartesianAxis) => {
    return ['X', 'Y', 'Z'].includes(axis) ? 'mm' : '°';
  };

  const getStep = (axis: CartesianAxis) => {
    return ['X', 'Y', 'Z'].includes(axis) ? 1 : 0.1;
  };

  const handleInputChange = (axis: CartesianAxis, value: string) => {
    // Allow typing (including partial numbers like "45." or "-")
    setInputValues({ ...inputValues, [axis]: value });
  };

  const handleInputBlur = (axis: CartesianAxis) => {
    const value = inputValues[axis];
    if (value !== undefined && value !== '') {
      const numValue = parseFloat(value);
      if (!isNaN(numValue)) {
        const limits = CARTESIAN_LIMITS[axis];
        // Clamp to limits
        const clampedValue = Math.max(limits.min, Math.min(limits.max, numValue));
        setInputCartesianValue(axis, clampedValue);
      }
    }
    // Clear input value to revert to showing inputCartesianPose
    setInputValues({ ...inputValues, [axis]: '' });
    // Clear IK status when user changes target
    if (ikStatus.type !== 'idle') {
      setIkStatus({ type: 'idle' });
    }
  };

  const handleInputKeyDown = (axis: CartesianAxis, e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      (e.target as HTMLInputElement).blur();
    }
  };

  // Sync cartesian sliders to match commanded robot's actual TCP position
  const handleSyncToRobotTcp = () => {
    if (!commandedTcpPose) {
      return;
    }

    // Copy all 6 values from commandedTcpPose to inputCartesianPose
    setInputCartesianValue('X', commandedTcpPose.X);
    setInputCartesianValue('Y', commandedTcpPose.Y);
    setInputCartesianValue('Z', commandedTcpPose.Z);
    setInputCartesianValue('RX', commandedTcpPose.RX);
    setInputCartesianValue('RY', commandedTcpPose.RY);
    setInputCartesianValue('RZ', commandedTcpPose.RZ);

    // Clear IK status since we're resetting to a known position
    setIkStatus({ type: 'idle' });
  };

  return (
    <div className="space-y-4">
      {/* Cartesian Sliders */}
      <div className="space-y-2">
        {CARTESIAN_AXES.map((axis) => {
          const limits = CARTESIAN_LIMITS[axis];
          const unit = getUnit(axis);
          const step = getStep(axis);
          const currentValue = inputCartesianPose[axis];
          const displayValue = inputValues[axis] !== undefined && inputValues[axis] !== ''
            ? inputValues[axis]
            : currentValue.toFixed(1);

          return (
            <div key={axis} className="space-y-1 pb-2 border-b last:border-b-0">
              <div className="flex justify-between items-center">
                <span className="text-xs font-medium">{axis}</span>
                <span className="text-xs text-muted-foreground">
                  [{limits.min.toFixed(0)}{unit} to {limits.max.toFixed(0)}{unit}]
                </span>
              </div>

              <div className="flex items-center gap-2">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleStepAxis(axis, -1)}
                  className="h-6 w-6 p-0"
                >
                  <ChevronLeft className="h-3 w-3" />
                </Button>
                <div className="flex-1">
                  <Slider
                    min={limits.min}
                    max={limits.max}
                    step={step}
                    value={[currentValue]}
                    onValueChange={(values) => handleSliderChange(axis, values[0])}
                    className="w-full"
                    tabIndex={-1}
                  />
                </div>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleStepAxis(axis, 1)}
                  className="h-6 w-6 p-0"
                >
                  <ChevronRight className="h-3 w-3" />
                </Button>
                <div className="flex items-center gap-1">
                  <Input
                    type="text"
                    value={displayValue}
                    onChange={(e) => handleInputChange(axis, e.target.value)}
                    onBlur={() => handleInputBlur(axis)}
                    onKeyDown={(e) => handleInputKeyDown(axis, e)}
                    className="w-12 h-6 px-1 text-xs font-mono text-right"
                  />
                  <span className="text-xs text-muted-foreground w-6">{unit}</span>
                </div>
              </div>
            </div>
          );
        })}
      </div>

      {/* IK Axis Mask Selector */}
      <div className="mt-4 pt-4 border-t">
        <div className="text-xs font-semibold mb-2 text-muted-foreground">IK Solve Axes:</div>
        <div className="flex flex-wrap gap-2">
          {(['X', 'Y', 'Z', 'RX', 'RY', 'RZ'] as const).map((axis) => (
            <label key={axis} className="flex items-center gap-2 cursor-pointer">
              <Checkbox
                checked={ikAxisMask[axis]}
                onCheckedChange={(checked) => {
                  setIkAxisMask({ [axis]: checked === true });
                }}
                className="w-4 h-4"
              />
              <span className="text-xs font-medium">
                {axis}
              </span>
            </label>
          ))}
        </div>
        <div className="mt-2 text-xs text-muted-foreground italic">
          Select which axes to solve during IK computation
        </div>
      </div>

      {/* Copy TCP and Compute IK Buttons */}
      <div className="mt-4 grid grid-cols-2 gap-2">
        <Button
          onClick={handleSyncToRobotTcp}
          disabled={!commandedTcpPose}
          className="w-full"
          variant="outline"
        >
          <Copy className="w-4 h-4 mr-2" />
          Copy TCP
        </Button>
        <Button
          onClick={handleComputeIK}
          disabled={ikStatus.type === 'computing'}
          className="w-full"
          variant="default"
        >
          <Calculator className="w-4 h-4 mr-2" />
          {ikStatus.type === 'computing' ? 'Computing...' : 'IK (Frontend)'}
        </Button>
      </div>

      {/* IK Status Feedback */}
      {ikStatus.type === 'success' && (
        <div className="bg-green-500/10 border border-green-500/50 rounded-lg p-3 flex items-start gap-2">
          <CheckCircle className="w-4 h-4 text-green-500 mt-0.5 flex-shrink-0" />
          <div className="text-xs text-green-400">
            <div className="font-semibold mb-1">IK Success!</div>
            <div>{ikStatus.message}</div>
          </div>
        </div>
      )}

      {ikStatus.type === 'error' && (
        <div className="bg-red-500/10 border border-red-500/50 rounded-lg p-3 flex items-start gap-2">
          <AlertCircle className="w-4 h-4 text-red-500 mt-0.5 flex-shrink-0" />
          <div className="text-xs">
            <div className="font-semibold text-red-500 mb-1">Frontend IK Failed</div>
            <div className="text-red-400">{ikStatus.message}</div>
            {ikStatus.iterations && (
              <div className="text-red-400/70 mt-1">
                Iterations: {ikStatus.iterations}
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
