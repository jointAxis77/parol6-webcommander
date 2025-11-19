import { useState, useCallback } from 'react';

/**
 * Custom hook for handling numeric input fields with validation and clamping
 * Eliminates duplicate input handling code across JointSliders and CartesianSliders
 */
export function useNumericInput<T extends string>(
  currentValues: Record<T, number>,
  setValue: (key: T, value: number) => void,
  getLimits?: (key: T) => { min: number; max: number }
) {
  // Track input field values separately to allow editing
  const [inputValues, setInputValues] = useState<Record<string, string>>({});

  const handleInputChange = useCallback((key: T, value: string) => {
    // Allow typing (including partial numbers like "45." or "-")
    setInputValues((prev) => ({ ...prev, [key]: value }));
  }, []);

  const handleInputBlur = useCallback((key: T) => {
    const value = inputValues[key as string];
    if (value !== undefined && value !== '') {
      const numValue = parseFloat(value);
      if (!isNaN(numValue)) {
        // Apply limits if provided
        let finalValue = numValue;
        if (getLimits) {
          const limits = getLimits(key);
          finalValue = Math.max(limits.min, Math.min(limits.max, numValue));
        }
        setValue(key, finalValue);
      }
    }
    // Clear input value to revert to showing currentValues
    setInputValues((prev) => ({ ...prev, [key]: '' }));
  }, [inputValues, setValue, getLimits]);

  const handleInputKeyDown = useCallback((key: T, e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      (e.target as HTMLInputElement).blur();
    }
  }, []);

  const getDisplayValue = useCallback((key: T, precision: number = 1): string => {
    const inputValue = inputValues[key as string];
    if (inputValue !== undefined && inputValue !== '') {
      return inputValue;
    }
    return currentValues[key].toFixed(precision);
  }, [inputValues, currentValues]);

  return {
    handleInputChange,
    handleInputBlur,
    handleInputKeyDown,
    getDisplayValue,
  };
}
