'use client';

import { useRef, useEffect } from 'react';
import * as THREE from 'three';

interface PathOrientationGizmoProps {
  position: THREE.Vector3;
  orientation: THREE.Euler;
}

/**
 * Displays RGB orientation gizmo at a path waypoint
 * Red = X axis, Green = Y axis, Blue = Z axis
 */
export default function PathOrientationGizmo({ position, orientation }: PathOrientationGizmoProps) {
  const groupRef = useRef<THREE.Group>(null);
  const xArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const yArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const zArrowRef = useRef<THREE.ArrowHelper | null>(null);

  useEffect(() => {
    if (!groupRef.current) return;

    // Arrow size (40mm = 0.04m, same as TCP visualizers)
    const arrowLength = 0.04;
    const arrowHeadLength = arrowLength * 0.2;
    const arrowHeadWidth = arrowLength * 0.15;

    // Create X arrow (Red)
    xArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0xff0000, // Red
      arrowHeadLength,
      arrowHeadWidth
    );

    // Create Y arrow (Green)
    yArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0x00ff00, // Green
      arrowHeadLength,
      arrowHeadWidth
    );

    // Create Z arrow (Blue)
    zArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, 1),
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0x0000ff, // Blue
      arrowHeadLength,
      arrowHeadWidth
    );

    // Add arrows to group
    groupRef.current.add(xArrowRef.current);
    groupRef.current.add(yArrowRef.current);
    groupRef.current.add(zArrowRef.current);

    // Cleanup
    return () => {
      if (xArrowRef.current) {
        xArrowRef.current.dispose();
      }
      if (yArrowRef.current) {
        yArrowRef.current.dispose();
      }
      if (zArrowRef.current) {
        zArrowRef.current.dispose();
      }
    };
  }, []);

  // Update position and orientation
  useEffect(() => {
    if (!groupRef.current) return;

    // Set position
    groupRef.current.position.copy(position);

    // Set orientation
    groupRef.current.rotation.copy(orientation);
  }, [position, orientation]);

  return <group ref={groupRef} />;
}
