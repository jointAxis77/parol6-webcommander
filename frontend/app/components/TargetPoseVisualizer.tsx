import { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import { useInputStore, useTimelineStore } from '@/app/lib/stores';
import { robotToThreeJs } from '@/app/lib/coordinateTransform';

/**
 * Visualizes the INPUT cartesian pose that the user is controlling via sliders
 * This shows where the user wants the TCP to go (red/green/blue gizmo)
 * (IK will be computed later during playback to make the robot follow this target)
 * Only shown in cartesian mode - hidden in joint mode
 *
 * NOTE: Input pose is in robot coordinates (Z-up), converted to Three.js (Y-up) for rendering
 */
export default function TargetPoseVisualizer() {
  const groupRef = useRef<THREE.Group>(null);
  const xArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const yArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const zArrowRef = useRef<THREE.ArrowHelper | null>(null);

  const inputCartesianPose = useInputStore((state) => state.inputCartesianPose);
  const motionMode = useTimelineStore((state) => state.timeline.mode);

  // Only show this gizmo in cartesian mode - it represents a cartesian target
  if (motionMode !== 'cartesian') {
    return null;
  }

  // Create arrows on mount
  useEffect(() => {
    if (!groupRef.current) return;

    // Arrow length in meters (50mm = 0.05m)
    const arrowLength = 0.05;
    const arrowHeadLength = arrowLength * 0.2;
    const arrowHeadWidth = arrowLength * 0.15;

    // X axis - Red (user X = viewport X)
    xArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0xff0000,
      arrowHeadLength,
      arrowHeadWidth
    );

    // Y axis - Green (user Y = -viewport Z)
    yArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, -1),
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0x00ff00,
      arrowHeadLength,
      arrowHeadWidth
    );

    // Z axis - Blue (user Z = viewport Y)
    zArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0x0000ff,
      arrowHeadLength,
      arrowHeadWidth
    );

    groupRef.current.add(xArrowRef.current);
    groupRef.current.add(yArrowRef.current);
    groupRef.current.add(zArrowRef.current);

    return () => {
      // Properly dispose of ArrowHelpers to prevent memory leaks
      if (xArrowRef.current) {
        groupRef.current?.remove(xArrowRef.current);
        xArrowRef.current.dispose();
      }
      if (yArrowRef.current) {
        groupRef.current?.remove(yArrowRef.current);
        yArrowRef.current.dispose();
      }
      if (zArrowRef.current) {
        groupRef.current?.remove(zArrowRef.current);
        zArrowRef.current.dispose();
      }
    };
  }, []);

  // Update input pose position and orientation every frame
  useFrame(() => {
    if (!groupRef.current) return;

    // Convert input pose from robot coordinates (Z-up) to Three.js (Y-up) for rendering
    const threeJsPose = robotToThreeJs(inputCartesianPose);

    // Set position in Three.js space (convert mm to meters)
    groupRef.current.position.set(
      threeJsPose.X / 1000,   // X (same in both)
      threeJsPose.Y / 1000,   // Y (was robot Z)
      threeJsPose.Z / 1000    // Z (was robot -Y)
    );

    // Update rotation from input orientation (convert degrees to radians)
    // Transform robot coordinate rotations to Three.js coordinate rotations
    // Robot Z-up → Three.js Y-up requires: RX stays, RY→-RZ, RZ→RY
    groupRef.current.rotation.order = 'ZXY';
    groupRef.current.rotation.set(
      (inputCartesianPose.RX * Math.PI) / 180,   // RX stays same
      (inputCartesianPose.RZ * Math.PI) / 180,   // Robot RZ → Three.js RY
      (-inputCartesianPose.RY * Math.PI) / 180   // Robot RY → Three.js -RZ (negated!)
    );
  });

  return <group ref={groupRef} />;
}
