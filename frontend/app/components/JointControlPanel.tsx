'use client';

import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Slider } from '@/components/ui/slider';
import { useInputStore, useCommandStore, useHardwareStore } from '../lib/stores';
import { JOINT_LIMITS, JOINT_NAMES } from '../lib/constants';
import { ChevronLeft, ChevronRight } from 'lucide-react';
import { useState } from 'react';

type ControlMode = 'joint' | 'cartesian';

export default function JointControlPanel() {
  const [controlMode, setControlMode] = useState<ControlMode>('joint');
  const [speed, setSpeed] = useState(80);
  const [accel, setAccel] = useState(60);

  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const setInputJointAngle = useInputStore((state) => state.setInputJointAngle);
  const setCommandedJointAngle = useCommandStore((state) => state.setCommandedJointAngle);

  // Get actual values from hardware store
  const hardwareJointAngles = useHardwareStore((state) => state.hardwareJointAngles);
  const actualJointAngles = hardwareJointAngles || commandedJointAngles; // Fallback to commanded if no hardware feedback

  const handleStepJoint = (joint: string, direction: number) => {
    const stepSize = 5.0; // degrees
    const currentValue = commandedJointAngles[joint as keyof typeof commandedJointAngles];
    const limits = JOINT_LIMITS[joint as keyof typeof JOINT_LIMITS];
    const newValue = Math.max(limits.min, Math.min(limits.max, currentValue + (direction * stepSize)));
    // Update both input and commanded stores
    setInputJointAngle(joint as any, newValue);
    setCommandedJointAngle(joint as any, newValue);
  };

  return (
    <Card className="p-6 h-full flex flex-col">
      <h2 className="text-lg font-semibold mb-4">Joint Control</h2>

      {/* Mode & Speed Controls */}
      <div className="space-y-4 mb-6">
        <div>
          <label className="text-sm font-medium mb-2 block">Mode</label>
          <div className="flex gap-2">
            <Button
              variant={controlMode === 'joint' ? 'default' : 'outline'}
              size="sm"
              onClick={() => setControlMode('joint')}
              className="flex-1"
            >
              Joint
            </Button>
            <Button
              variant={controlMode === 'cartesian' ? 'default' : 'outline'}
              size="sm"
              onClick={() => setControlMode('cartesian')}
              className="flex-1"
            >
              Cartesian
            </Button>
          </div>
        </div>

        <div>
          <label className="text-sm font-medium mb-2 block">
            Speed: {speed}%
          </label>
          <Slider
            value={[speed]}
            onValueChange={(value) => setSpeed(value[0])}
            min={0}
            max={100}
            step={1}
            className="w-full"
          />
        </div>

        <div>
          <label className="text-sm font-medium mb-2 block">
            Accel: {accel}%
          </label>
          <Slider
            value={[accel]}
            onValueChange={(value) => setAccel(value[0])}
            min={0}
            max={100}
            step={1}
            className="w-full"
          />
        </div>
      </div>

      {/* Joint Sliders */}
      <div className="flex-1 overflow-y-auto space-y-4">
        {JOINT_NAMES.map((joint) => {
          const limits = JOINT_LIMITS[joint];
          const setValue = commandedJointAngles[joint];
          const actualValue = actualJointAngles[joint];
          const error = Math.abs(setValue - actualValue);

          // Color coding based on tracking error
          let errorColor = 'text-green-500'; // Good tracking
          if (error > 1 && error <= 5) {
            errorColor = 'text-yellow-500'; // Lagging
          } else if (error > 5) {
            errorColor = 'text-red-500'; // Large error
          }

          return (
            <div key={joint} className="space-y-2 pb-4 border-b last:border-b-0">
              <div className="flex justify-between items-center">
                <span className="text-sm font-medium">
                  {joint} [{limits.min.toFixed(0)}° to {limits.max.toFixed(0)}°]
                </span>
              </div>

              {/* Set Value Slider */}
              <div>
                <div className="flex justify-between text-xs text-muted-foreground mb-1">
                  <span>Set:</span>
                  <span className="font-mono">{setValue.toFixed(1)}°</span>
                </div>
                <Slider
                  value={[setValue]}
                  onValueChange={(value) => {
                    setInputJointAngle(joint as any, value[0]);
                    setCommandedJointAngle(joint as any, value[0]);
                  }}
                  min={limits.min}
                  max={limits.max}
                  step={0.1}
                  className="w-full"
                />
              </div>

              {/* Actual Value Indicator */}
              <div>
                <div className="flex justify-between text-xs text-muted-foreground mb-1">
                  <span>Actual:</span>
                  <span className={`font-mono ${errorColor}`}>
                    {actualValue.toFixed(1)}°
                  </span>
                </div>
                <div className="relative h-2 bg-secondary rounded-full overflow-hidden">
                  {/* Indicator dot */}
                  <div
                    className={`absolute top-0 w-3 h-3 -mt-0.5 rounded-full ${
                      error > 5 ? 'bg-red-500' : error > 1 ? 'bg-yellow-500' : 'bg-green-500'
                    }`}
                    style={{
                      left: `${((actualValue - limits.min) / (limits.max - limits.min)) * 100}%`,
                      transform: 'translateX(-50%)',
                    }}
                  />
                </div>
              </div>

              {/* Step Buttons */}
              <div className="flex gap-2 justify-center">
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleStepJoint(joint, -1)}
                >
                  <ChevronLeft className="h-4 w-4" />
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  className="px-6"
                >
                  Step
                </Button>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleStepJoint(joint, 1)}
                >
                  <ChevronRight className="h-4 w-4" />
                </Button>
              </div>
            </div>
          );
        })}
      </div>

      {/* Action Buttons */}
      <div className="grid grid-cols-2 gap-2 mt-4 pt-4 border-t">
        <Button variant="default">Enable Motors</Button>
        <Button variant="outline">Disable Motors</Button>
        <Button variant="outline">Home Position</Button>
        <Button variant="outline">Record Pose</Button>
      </div>
    </Card>
  );
}
