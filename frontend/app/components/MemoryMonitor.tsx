/**
 * Memory Monitor Component
 *
 * Helps debug STATUS_ACCESS_VIOLATION crashes by tracking:
 * - Browser memory usage
 * - Three.js renderer info
 * - WebGL context status
 * - Component update rates
 */

import { useEffect, useState, useRef } from 'react';
import { useThree } from '@react-three/fiber';
import { logger } from '../lib/logger';

interface MemoryStats {
  jsHeapSize: number;
  jsHeapSizeLimit: number;
  totalJSHeapSize: number;
  geometries: number;
  textures: number;
  programs: number;
  calls: number;
  triangles: number;
  points: number;
  lines: number;
}

export function MemoryMonitor() {
  const { gl } = useThree();
  const [stats, setStats] = useState<MemoryStats | null>(null);
  const [updateCount, setUpdateCount] = useState(0);
  const updateCountRef = useRef(0);
  const lastLogTime = useRef(Date.now());

  useEffect(() => {
    const interval = setInterval(() => {
      // @ts-ignore - performance.memory is Chrome-specific
      const memory = (performance as any).memory;

      if (!memory) {
        logger.warn('performance.memory not available. Use Chrome with --enable-precise-memory-info flag', 'MemoryMonitor');
        return;
      }

      const info = gl.info;
      const newStats: MemoryStats = {
        jsHeapSize: memory.usedJSHeapSize,
        jsHeapSizeLimit: memory.jsHeapSizeLimit,
        totalJSHeapSize: memory.totalJSHeapSize,
        geometries: info.memory.geometries,
        textures: info.memory.textures,
        programs: info.programs?.length || 0,
        calls: info.render.calls,
        triangles: info.render.triangles,
        points: info.render.points,
        lines: info.render.lines,
      };

      setStats(newStats);

      // Log warning if memory usage is high
      const usagePercent = (memory.usedJSHeapSize / memory.jsHeapSizeLimit) * 100;
      if (usagePercent > 80) {
        logger.warn(`HIGH MEMORY USAGE: ${usagePercent.toFixed(1)}% (${formatBytes(memory.usedJSHeapSize)} / ${formatBytes(memory.jsHeapSizeLimit)})`, 'MemoryMonitor');
      }

      // Log update rate every 10 seconds
      const now = Date.now();
      if (now - lastLogTime.current > 10000) {
        const updateRate = updateCountRef.current / 10;
        logger.debug(`Component update rate: ${updateRate.toFixed(1)} updates/sec`, 'MemoryMonitor');
        updateCountRef.current = 0;
        lastLogTime.current = now;
      }
    }, 1000); // Update every second

    return () => clearInterval(interval);
  }, [gl]);

  // Track component updates
  useEffect(() => {
    updateCountRef.current++;
    setUpdateCount((c) => c + 1);
  });

  if (!stats) {
    return (
      <div className="absolute bottom-4 right-4 bg-black/70 text-white p-2 rounded text-xs font-mono">
        Memory monitoring not available
      </div>
    );
  }

  const usagePercent = (stats.jsHeapSize / stats.jsHeapSizeLimit) * 100;
  const isHighMemory = usagePercent > 80;

  return (
    <div className={`absolute bottom-4 right-4 bg-black/70 text-white p-2 rounded text-xs font-mono ${isHighMemory ? 'border-2 border-red-500' : ''}`}>
      <div className="font-bold mb-1">Memory Monitor</div>
      <div className={isHighMemory ? 'text-red-400' : ''}>
        Heap: {formatBytes(stats.jsHeapSize)} / {formatBytes(stats.jsHeapSizeLimit)} ({usagePercent.toFixed(1)}%)
      </div>
      <div>Total: {formatBytes(stats.totalJSHeapSize)}</div>
      <div className="mt-1 pt-1 border-t border-gray-600">
        <div>Geometries: {stats.geometries}</div>
        <div>Textures: {stats.textures}</div>
        <div>Programs: {stats.programs}</div>
      </div>
      <div className="mt-1 pt-1 border-t border-gray-600">
        <div>Draw calls: {stats.calls}</div>
        <div>Triangles: {stats.triangles}</div>
      </div>
      <div className="mt-1 pt-1 border-t border-gray-600 text-[10px] text-gray-400">
        Updates: {updateCount}
      </div>
    </div>
  );
}

/**
 * WebGL Context Loss Monitor
 * Detects and logs WebGL context loss events
 */
export function WebGLContextMonitor() {
  const { gl } = useThree();

  useEffect(() => {
    const canvas = gl.domElement;

    const handleContextLost = (event: Event) => {
      event.preventDefault();
      logger.error('WebGL Context Lost!', 'MemoryMonitor');
      alert('WebGL context lost! The 3D view may not work correctly. Try refreshing the page.');
    };

    const handleContextRestored = () => {
      logger.debug('WebGL Context Restored', 'MemoryMonitor');
    };

    canvas.addEventListener('webglcontextlost', handleContextLost);
    canvas.addEventListener('webglcontextrestored', handleContextRestored);

    return () => {
      canvas.removeEventListener('webglcontextlost', handleContextLost);
      canvas.removeEventListener('webglcontextrestored', handleContextRestored);
    };
  }, [gl]);

  return null;
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return bytes + ' B';
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
  return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
}
