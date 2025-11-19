'use client';

import { useState } from 'react';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Slider } from '@/components/ui/slider';
import { useInputStore, useCommandStore, useHardwareStore } from '@/app/lib/stores';
import { JOINT_LIMITS, JOINT_NAMES } from '../lib/constants';
import { ChevronLeft, ChevronRight } from 'lucide-react';
import type { JointName } from '@/app/lib/types';

export default function CompactJointSliders() {
  // Input store: What user is controlling
  const inputJointAngles = useInputStore((state) => state.inputJointAngles);
  const setInputJointAngle = useInputStore((state) => state.setInputJointAngle);

  // Command store: Commanded state and control modes
  const setCommandedJointAngle = useCommandStore((state) => state.setCommandedJointAngle);
  const teachModeEnabled = useCommandStore((state) => state.teachModeEnabled);
  const liveControlEnabled = useCommandStore((state) => state.liveControlEnabled);

  // Hardware store: Get actual values from hardware feedback
  const hardwareJointAngles = useHardwareStore((state) => state.hardwareJointAngles) || inputJointAngles;

  // Step angle for increment/decrement buttons (from settings)
  const stepAngle = useInputStore((state) => state.stepAngle);

  // Track input field values separately to allow editing (e.g., typing "45." or "-")
  const [inputValues, setInputValues] = useState<Record<string, string>>({});

  const handleInputChange = (joint: JointName, value: string) => {
    setInputValues({ ...inputValues, [joint]: value });
  };

  const handleInputBlur = (joint: JointName) => {
    const value = inputValues[joint];
    if (value !== undefined && value !== '') {
      const numValue = parseFloat(value);
      if (!isNaN(numValue)) {
        const limits = JOINT_LIMITS[joint];
        const clampedValue = Math.max(limits.min, Math.min(limits.max, numValue));
        setInputJointAngle(joint, clampedValue);
        setCommandedJointAngle(joint, clampedValue);
      }
    }
    setInputValues({ ...inputValues, [joint]: '' });
  };

  const handleInputKeyDown = (joint: JointName, e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      (e.target as HTMLInputElement).blur();
    }
  };

  const handleStepJoint = (joint: JointName, direction: number) => {
    const currentValue = inputJointAngles[joint];
    const limits = JOINT_LIMITS[joint];
    const newValue = Math.max(limits.min, Math.min(limits.max, currentValue + (direction * stepAngle)));

    // Update both input and commanded stores (in joint mode they should match)
    setInputJointAngle(joint, newValue);
    setCommandedJointAngle(joint, newValue);
  };

  return (
    <Card className="p-3">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-sm font-semibold">Joint Control</h2>
        {liveControlEnabled && (
          <span className="text-[10px] bg-yellow-500/20 text-yellow-500 px-2 py-1 rounded font-semibold">
            ⚡ LIVE MODE
          </span>
        )}
        {teachModeEnabled && (
          <span className="text-[10px] bg-green-500/20 text-green-500 px-2 py-1 rounded font-semibold">
            🔗 FOLLOWING
          </span>
        )}
      </div>

      {/* Joint Sliders - Compact */}
      <div className="space-y-2">
        {JOINT_NAMES.map((joint) => {
          const limits = JOINT_LIMITS[joint];
          const inputValue = inputJointAngles[joint];
          const hardwareValue = hardwareJointAngles[joint];
          const error = Math.abs(inputValue - hardwareValue);

          // Color coding based on tracking error
          let errorColor = 'text-green-500';
          if (error > 1 && error <= 5) {
            errorColor = 'text-yellow-500';
          } else if (error > 5) {
            errorColor = 'text-red-500';
          }

          const displayValue = inputValues[joint] !== undefined && inputValues[joint] !== ''
            ? inputValues[joint]
            : inputValue.toFixed(1);

          return (
            <div key={joint} className="space-y-1 pb-2 border-b last:border-b-0">
              <div className="flex justify-between items-center">
                <span className="text-xs font-medium">
                  {joint}
                </span>
                <span className="text-xs text-muted-foreground">
                  [{limits.min.toFixed(0)}° to {limits.max.toFixed(0)}°]
                </span>
              </div>

              {/* Set Value Slider */}
              <div className="flex items-center gap-2" title={teachModeEnabled ? 'Controls disabled - Input is following hardware robot' : ''}>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleStepJoint(joint, -1)}
                  className="h-6 w-6 p-0"
                  disabled={teachModeEnabled}
                >
                  <ChevronLeft className="h-3 w-3" />
                </Button>
                <div className="flex-1">
                  <Slider
                    value={[inputValue]}
                    onValueChange={(value) => {
                      // Update both input and commanded stores (in joint mode they should match)
                      setInputJointAngle(joint, value[0]);
                      setCommandedJointAngle(joint, value[0]);
                    }}
                    min={limits.min}
                    max={limits.max}
                    step={0.1}
                    className="w-full"
                    disabled={teachModeEnabled}
                    tabIndex={-1}
                  />
                </div>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => handleStepJoint(joint, 1)}
                  className="h-6 w-6 p-0"
                  disabled={teachModeEnabled}
                >
                  <ChevronRight className="h-3 w-3" />
                </Button>
                <div className="flex items-center gap-1">
                  <Input
                    type="text"
                    value={displayValue}
                    onChange={(e) => handleInputChange(joint, e.target.value)}
                    onBlur={() => handleInputBlur(joint)}
                    onKeyDown={(e) => handleInputKeyDown(joint, e)}
                    disabled={teachModeEnabled}
                    className={`w-12 h-6 px-1 text-xs font-mono text-right ${teachModeEnabled ? 'opacity-50' : ''}`}
                  />
                  <span className="text-xs text-muted-foreground">°</span>
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </Card>
  );
}
