/**
 * Performance Store
 * Manages performance recording state and data
 */

import { create } from 'zustand';
import { getApiBaseUrl } from '../apiConfig';
import { logger } from '../logger';

// Types for performance recording data
export interface PerformanceSample {
  cycle: number;
  network: number;
  processing: number;
  execution: number;
  serial: number;
  ik_manipulability: number;
  ik_solve: number;
}

export interface CycleStats {
  avg_ms: number;
  min_ms: number;
  max_ms: number;
}

export interface PhaseStats {
  network_ms: number;
  processing_ms: number;
  execution_ms: number;
  serial_ms: number;
  ik_manipulability_ms: number;
  ik_solve_ms: number;
}

export interface CommandPerformance {
  command_id: string;
  command_type: string;
  timestamp: string;
  duration_s: number;
  num_cycles: number;
  cycle_stats: CycleStats;
  phase_stats: PhaseStats;
  samples: PerformanceSample[];
}

export interface RecordingMetadata {
  name: string;
  timestamp: string;
  robot_config: {
    com_port: string;
    baud_rate: number;
  };
}

export interface PerformanceRecording {
  metadata: RecordingMetadata;
  commands: CommandPerformance[];
}

export interface RecordingListItem {
  filename: string;
  name: string;
  timestamp: string;
  num_commands: number;
  total_duration_s: number;
}

export interface PerformanceStore {
  // Recording state
  isRecording: boolean;
  recordingName: string;

  // Recordings list
  recordings: RecordingListItem[];
  isLoadingRecordings: boolean;

  // Selected recording
  selectedRecording: PerformanceRecording | null;
  selectedFilename: string | null;
  isLoadingRecording: boolean;

  // Actions
  setIsRecording: (isRecording: boolean) => void;
  setRecordingName: (name: string) => void;

  startRecording: (name?: string) => Promise<void>;
  stopRecording: () => Promise<void>;

  fetchRecordings: () => Promise<void>;
  fetchRecording: (filename: string) => Promise<void>;
  deleteRecording: (filename: string) => Promise<void>;
  selectRecording: (filename: string | null) => void;
}

export const usePerformanceStore = create<PerformanceStore>((set, get) => ({
  // Initial state
  isRecording: false,
  recordingName: '',

  recordings: [],
  isLoadingRecordings: false,

  selectedRecording: null,
  selectedFilename: null,
  isLoadingRecording: false,

  // Actions
  setIsRecording: (isRecording) => set({ isRecording }),
  setRecordingName: (recordingName) => set({ recordingName }),

  startRecording: async () => {
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/performance/recording/enable`, {
        method: 'POST',
      });

      if (!response.ok) {
        throw new Error('Failed to enable recording');
      }

      const data = await response.json();

      if (data.success) {
        set({ isRecording: true });
      } else {
        logger.error('Failed to enable recording', 'PerformanceStore', { message: data.message });
      }
    } catch (error) {
      logger.error('Error enabling recording', 'PerformanceStore', error);
    }
  },

  stopRecording: async () => {
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/performance/recording/disable`, {
        method: 'POST',
      });

      if (!response.ok) {
        throw new Error('Failed to disable recording');
      }

      const data = await response.json();

      if (data.success) {
        set({ isRecording: false });
        // Refresh recordings list
        get().fetchRecordings();
      } else {
        logger.error('Failed to disable recording', 'PerformanceStore', { message: data.message });
      }
    } catch (error) {
      logger.error('Error disabling recording', 'PerformanceStore', error);
    }
  },

  fetchRecordings: async () => {
    set({ isLoadingRecordings: true });

    try {
      const response = await fetch(`${getApiBaseUrl()}/api/performance/recordings`);

      if (!response.ok) {
        throw new Error('Failed to fetch recordings');
      }

      const data: RecordingListItem[] = await response.json();
      set({ recordings: data, isLoadingRecordings: false });
    } catch (error) {
      logger.error('Error fetching recordings', 'PerformanceStore', error);
      set({ isLoadingRecordings: false });
    }
  },

  fetchRecording: async (filename: string) => {
    set({ isLoadingRecording: true });

    try {
      const response = await fetch(`${getApiBaseUrl()}/api/performance/recordings/${filename}`);

      if (!response.ok) {
        throw new Error('Failed to fetch recording');
      }

      const data: PerformanceRecording = await response.json();
      set({
        selectedRecording: data,
        selectedFilename: filename,
        isLoadingRecording: false
      });
    } catch (error) {
      logger.error('Error fetching recording', 'PerformanceStore', error);
      set({ isLoadingRecording: false });
    }
  },

  deleteRecording: async (filename: string) => {
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/performance/recordings/${filename}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to delete recording');
      }

      // Clear selected recording if it was deleted
      const { selectedFilename } = get();
      if (selectedFilename === filename) {
        set({ selectedRecording: null, selectedFilename: null });
      }

      // Refresh recordings list
      get().fetchRecordings();
    } catch (error) {
      logger.error('Error deleting recording', 'PerformanceStore', error);
    }
  },

  selectRecording: (filename: string | null) => {
    if (filename) {
      get().fetchRecording(filename);
    } else {
      set({ selectedRecording: null, selectedFilename: null });
    }
  },
}));
