/**
 * TCP Pose Display Component
 *
 * Displays TCP pose values with color-coded axes
 * Works with robot coordinates (Z-up) from stores
 */

import type { CartesianPose } from '@/app/lib/types';

export interface TCPPoseDisplayProps {
  /** Pose to display (in robot coordinates: Z-up) */
  pose: CartesianPose | null;
  /** Label for this pose (e.g., "Target", "Actual") */
  label: string;
  /** Color scheme for X, Y, Z axes */
  colors: {
    x: string;
    y: string;
    z: string;
  };
  /** CSS class name for additional styling */
  className?: string;
}

/**
 * Display TCP pose with color-coded axes
 *
 * Shows robot coordinates (Z-up):
 * - X: Forward
 * - Y: Left
 * - Z: Up
 */
export function TCPPoseDisplay({ pose, label, colors, className = '' }: TCPPoseDisplayProps) {
  if (!pose) {
    return (
      <div className={`grid grid-cols-7 gap-1 ${className}`}>
        <div className="text-gray-400">{label}:  </div>
        <div className="text-center text-gray-500 col-span-6">N/A</div>
      </div>
    );
  }

  return (
    <div className={`grid grid-cols-7 gap-1 ${className}`}>
      <div className="text-gray-400">{label}:  </div>
      <div className="text-center" style={{ color: colors.x }}>
        {pose.X.toFixed(1)}
      </div>
      <div className="text-center" style={{ color: colors.y }}>
        {pose.Y.toFixed(1)}
      </div>
      <div className="text-center" style={{ color: colors.z }}>
        {pose.Z.toFixed(1)}
      </div>
      <div className="text-center" style={{ color: colors.x }}>
        {pose.RX.toFixed(1)}
      </div>
      <div className="text-center" style={{ color: colors.y }}>
        {pose.RY.toFixed(1)}
      </div>
      <div className="text-center" style={{ color: colors.z }}>
        {pose.RZ.toFixed(1)}
      </div>
    </div>
  );
}

/**
 * Display header row for TCP pose table
 */
export function TCPPoseHeader({ className = '' }: { className?: string }) {
  return (
    <div className={`grid grid-cols-7 gap-1 text-xs font-semibold mb-1 ${className}`}>
      <div></div>
      <div className="text-center">X</div>
      <div className="text-center">Y</div>
      <div className="text-center">Z</div>
      <div className="text-center">RX</div>
      <div className="text-center">RY</div>
      <div className="text-center">RZ</div>
    </div>
  );
}
