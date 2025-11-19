import { useRef, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

/**
 * Visualizes the TCP (Tool Center Point) for the gripper editor
 * Shows where the functional tool tip is based on TCP offset configuration
 * Uses orange/cyan/magenta color scheme (same as TargetTCPVisualizer)
 */
export default function GripperTCPVisualizer({
  robotRef,
  tcpOffset
}: {
  robotRef: any;
  tcpOffset: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
}) {
  const groupRef = useRef<THREE.Group>(null);
  const xArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const yArrowRef = useRef<THREE.ArrowHelper | null>(null);
  const zArrowRef = useRef<THREE.ArrowHelper | null>(null);

  // Reusable objects to prevent memory leaks (don't create new objects every frame!)
  const l6WorldPosition = useRef(new THREE.Vector3());
  const l6WorldQuaternion = useRef(new THREE.Quaternion());
  const tcpRotationQuat = useRef(new THREE.Quaternion());
  const tcpRotationEuler = useRef(new THREE.Euler());
  const postRotationQuat = useRef(new THREE.Quaternion());
  const localOffset = useRef(new THREE.Vector3());
  const worldOffset = useRef(new THREE.Vector3());
  const tcpWorldPosition = useRef(new THREE.Vector3());

  // Create arrows on mount with distinct styling
  useEffect(() => {
    if (!groupRef.current) return;

    // Arrow size (40mm)
    const arrowLength = 0.04;
    const arrowHeadLength = arrowLength * 0.2;
    const arrowHeadWidth = arrowLength * 0.15;

    // Orange/Cyan/Magenta color scheme for TCP
    // Standard orientation - TCP rotation quaternion handles all transformations

    // X axis - Orange (standard +X direction)
    xArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),  // Standard X direction
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0xff8800, // Orange
      arrowHeadLength,
      arrowHeadWidth
    );

    // Y axis - Cyan (standard +Y direction)
    yArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),  // Standard Y direction
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0x00dddd, // Cyan
      arrowHeadLength,
      arrowHeadWidth
    );

    // Z axis - Magenta (standard +Z direction)
    zArrowRef.current = new THREE.ArrowHelper(
      new THREE.Vector3(0, 0, 1),  // Standard Z direction
      new THREE.Vector3(0, 0, 0),
      arrowLength,
      0xdd00dd, // Magenta/Fuchsia
      arrowHeadLength,
      arrowHeadWidth
    );

    groupRef.current.add(xArrowRef.current);
    groupRef.current.add(yArrowRef.current);
    groupRef.current.add(zArrowRef.current);

    return () => {
      // Properly dispose ArrowHelpers to free GPU memory
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

  // Update TCP position every frame
  useFrame(() => {
    if (!groupRef.current || !robotRef) return;

    const j6Joint = robotRef.joints?.['L6'];
    if (j6Joint) {
      j6Joint.updateMatrixWorld(true);

      // Reuse objects to prevent memory leaks
      j6Joint.getWorldPosition(l6WorldPosition.current);
      j6Joint.getWorldQuaternion(l6WorldQuaternion.current);

      // Convert TCP offset from mm to m and apply
      localOffset.current.set(
        tcpOffset.x / 1000,
        tcpOffset.y / 1000,
        tcpOffset.z / 1000
      );

      worldOffset.current.copy(localOffset.current).applyQuaternion(l6WorldQuaternion.current);
      tcpWorldPosition.current.copy(l6WorldPosition.current).add(worldOffset.current);

      groupRef.current.position.copy(tcpWorldPosition.current);

      // Apply TCP orientation offset to gizmo
      // Start with L6 orientation
      groupRef.current.quaternion.copy(l6WorldQuaternion.current);

      // Apply user-configurable TCP rotation
      if (tcpOffset.rx !== 0 || tcpOffset.ry !== 0 || tcpOffset.rz !== 0) {
        tcpRotationEuler.current.set(
          tcpOffset.rx * Math.PI / 180,
          tcpOffset.ry * Math.PI / 180,
          tcpOffset.rz * Math.PI / 180,
          'XYZ'
        );
        tcpRotationQuat.current.setFromEuler(tcpRotationEuler.current);
        groupRef.current.quaternion.multiply(tcpRotationQuat.current);
      }

      // Apply fixed post-rotation: -90° around Z axis
      // This aligns the standard arrows with the display coordinate system
      postRotationQuat.current.setFromAxisAngle(new THREE.Vector3(0, 0, 1), -Math.PI / 2);
      groupRef.current.quaternion.multiply(postRotationQuat.current);
    }
  });

  return <group ref={groupRef} />;
}
