'use client';

import { useState } from 'react';
import { useInputStore, useCommandStore } from '@/app/lib/stores';
import { JOINT_NAMES, JOINT_LIMITS } from '@/app/lib/constants';
import { Slider } from '@/components/ui/slider';
import { Input } from '@/components/ui/input';
import type { JointName } from '@/app/lib/types';

export default function JointSliders() {
  const inputJointAngles = useInputStore((state) => state.inputJointAngles);
  const setInputJointAngle = useInputStore((state) => state.setInputJointAngle);
  const setCommandedJointAngle = useCommandStore((state) => state.setCommandedJointAngle);

  // Track input field values separately to allow editing
  const [inputValues, setInputValues] = useState<Record<string, string>>({});

  const handleInputChange = (joint: string, value: string) => {
    // Allow typing (including partial numbers like "45." or "-")
    setInputValues({ ...inputValues, [joint]: value });
  };

  const handleInputBlur = (joint: JointName) => {
    const value = inputValues[joint];
    if (value !== undefined && value !== '') {
      const numValue = parseFloat(value);
      if (!isNaN(numValue)) {
        const limits = JOINT_LIMITS[joint];
        // Clamp to joint limits
        const clampedValue = Math.max(limits.min, Math.min(limits.max, numValue));

        // Update both input and commanded stores (in joint mode they should match)
        setInputJointAngle(joint, clampedValue);
        setCommandedJointAngle(joint, clampedValue);
      }
    }
    // Clear input value to revert to showing inputJointAngles
    setInputValues({ ...inputValues, [joint]: '' });
  };

  const handleSliderChange = (joint: JointName, value: number) => {
    // Update both input and commanded stores (in joint mode they should match)
    setInputJointAngle(joint, value);
    setCommandedJointAngle(joint, value);
  };

  const handleInputKeyDown = (joint: string, e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      (e.target as HTMLInputElement).blur();
    }
  };

  return (
    <div className="space-y-4">
      {JOINT_NAMES.map((joint) => {
        const limits = JOINT_LIMITS[joint];
        const inputValue = inputJointAngles[joint];
        const displayValue = inputValues[joint] !== undefined && inputValues[joint] !== ''
          ? inputValues[joint]
          : inputValue.toFixed(1);

        return (
          <div key={joint} className="space-y-2">
            <div className="flex justify-between items-center text-sm gap-2">
              <span className="font-medium">{joint}</span>
              <div className="flex items-center gap-1">
                <Input
                  type="text"
                  value={displayValue}
                  onChange={(e) => handleInputChange(joint, e.target.value)}
                  onBlur={() => handleInputBlur(joint)}
                  onKeyDown={(e) => handleInputKeyDown(joint, e)}
                  className="w-16 h-7 px-2 text-xs font-mono text-right"
                />
                <span className="text-xs text-muted-foreground">°</span>
              </div>
            </div>
            <Slider
              min={limits.min}
              max={limits.max}
              step={0.1}
              value={[inputValue]}
              onValueChange={(values) => handleSliderChange(joint, values[0])}
              className="w-full"
            />
          </div>
        );
      })}
    </div>
  );
}
