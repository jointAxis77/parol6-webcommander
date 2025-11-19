/**
 * Safety Confirmation Hook
 *
 * Provides a reusable hook for confirming robot actions
 * Checks config.ui.show_safety_warnings setting
 * If enabled: shows confirmation dialog (Enter to confirm, Escape to cancel)
 * If disabled: executes immediately
 */

'use client';

import { useState, useCallback } from 'react';
import { useConfigStore } from '../lib/configStore';
import { ConfirmDialog } from '@/components/ui/dialog';

interface ConfirmationState {
  isOpen: boolean;
  message: string;
  title: string;
  resolve: ((value: boolean) => void) | null;
}

export function useSafetyConfirmation() {
  const showSafetyWarnings = useConfigStore(
    (state) => state.config?.ui?.show_safety_warnings ?? true
  );

  const [confirmationState, setConfirmationState] = useState<ConfirmationState>({
    isOpen: false,
    message: '',
    title: '',
    resolve: null,
  });

  const confirmAction = useCallback(
    (message: string, title: string = 'Confirm Robot Movement'): Promise<boolean> => {
      // If safety warnings disabled, execute immediately
      if (!showSafetyWarnings) {
        return Promise.resolve(true);
      }

      // Show confirmation dialog
      return new Promise((resolve) => {
        setConfirmationState({
          isOpen: true,
          message,
          title,
          resolve,
        });
      });
    },
    [showSafetyWarnings]
  );

  const handleConfirm = useCallback(() => {
    if (confirmationState.resolve) {
      confirmationState.resolve(true);
    }
    setConfirmationState({
      isOpen: false,
      message: '',
      title: '',
      resolve: null,
    });
  }, [confirmationState]);

  const handleCancel = useCallback(() => {
    if (confirmationState.resolve) {
      confirmationState.resolve(false);
    }
    setConfirmationState({
      isOpen: false,
      message: '',
      title: '',
      resolve: null,
    });
  }, [confirmationState]);

  const DialogComponent = useCallback(() => {
    return (
      <ConfirmDialog
        open={confirmationState.isOpen}
        onOpenChange={(open) => {
          if (!open) handleCancel();
        }}
        onConfirm={handleConfirm}
        title={confirmationState.title}
        description={confirmationState.message}
        confirmText="Confirm"
        cancelText="Cancel"
        variant="default"
      />
    );
  }, [confirmationState, handleConfirm, handleCancel]);

  return {
    confirmAction,
    SafetyDialog: DialogComponent,
  };
}
