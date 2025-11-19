/**
 * Store Index
 * Re-exports all stores for easy importing
 *
 * Usage:
 *   import { useInputStore, useCommandStore, useHardwareStore } from '@/app/lib/stores';
 */

export { useInputStore } from './inputStore';
export type { InputStore } from './inputStore';

export { useCommandStore } from './commandStore';
export type { CommandStore } from './commandStore';

export { useHardwareStore } from './hardwareStore';
export type { HardwareStore } from './hardwareStore';

export { useTimelineStore } from './timelineStore';
export type { TimelineStore } from './timelineStore';

export { useRobotConfigStore } from './robotConfigStore';
export type { RobotConfigStore } from './robotConfigStore';

export { usePerformanceStore } from './performanceStore';
export type { PerformanceStore, PerformanceRecording, RecordingListItem, CommandPerformance } from './performanceStore';
