'use client';

import { useTimelineStore } from '@/app/lib/stores/timelineStore';

/**
 * Progress bar overlay for IK trajectory pre-calculation
 * Displays real-time progress during waypoint IK solving
 * Shows: current/total waypoints, percentage, recovery count
 */
export default function IKProgressBar() {
  const ikProgress = useTimelineStore((state) => state.ikProgress);

  if (!ikProgress.isCalculating) {
    return null; // Hidden when not calculating
  }

  const progress = ikProgress.total > 0
    ? (ikProgress.current / ikProgress.total) * 100
    : 0;

  return (
    <div className="absolute top-4 left-1/2 transform -translate-x-1/2 z-50
                    bg-black/80 backdrop-blur-sm rounded-lg px-6 py-3
                    border border-cyan-500/30 shadow-lg">
      <div className="flex flex-col gap-2 min-w-[300px]">
        {/* Title */}
        <div className="text-sm font-semibold text-cyan-400">
          Calculating Trajectory
        </div>

        {/* Progress bar */}
        <div className="w-full bg-gray-700 rounded-full h-2 overflow-hidden">
          <div
            className="bg-gradient-to-r from-cyan-500 to-blue-500 h-full transition-all duration-300 ease-out"
            style={{ width: `${progress}%` }}
          />
        </div>

        {/* Stats */}
        <div className="text-xs text-gray-300 flex justify-between">
          <span>
            {ikProgress.current} / {ikProgress.total} waypoints
          </span>
          <span className="text-cyan-400">
            {progress.toFixed(0)}%
          </span>
        </div>

        {/* Recovery count */}
        {ikProgress.recoveries !== undefined && ikProgress.recoveries > 0 && (
          <div className="text-xs text-yellow-400">
            {ikProgress.recoveries} recovered with J1 sweep
          </div>
        )}
      </div>
    </div>
  );
}
