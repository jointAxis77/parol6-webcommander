import { Html } from '@react-three/drei';
import { useCommandStore, useInputStore } from '../lib/stores';
import { JOINT_LIMITS, JOINT_NAMES } from '../lib/constants';
import * as THREE from 'three';

interface JointLabelsProps {
  urdfRobot: any;
  visible?: boolean;
}

export default function JointLabels({ urdfRobot, visible = true }: JointLabelsProps) {
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const selectedJoint = useInputStore((state) => state.selectedJoint);

  if (!visible || !urdfRobot) return null;

  // Get status color based on proximity to limits
  const getJointStatus = (jointName: string, value: number): 'normal' | 'warning' | 'limit' => {
    const limits = JOINT_LIMITS[jointName as keyof typeof JOINT_LIMITS];
    const range = limits.max - limits.min;
    const warningThreshold = range * 0.2; // 20% from limits
    const limitThreshold = range * 0.1; // 10% from limits

    if (value <= limits.min + limitThreshold || value >= limits.max - limitThreshold) {
      return 'limit'; // Red
    } else if (value <= limits.min + warningThreshold || value >= limits.max - warningThreshold) {
      return 'warning'; // Yellow
    } else {
      return 'normal'; // Green
    }
  };

  const getStatusColor = (status: string): string => {
    switch (status) {
      case 'normal':
        return '#4ade80'; // green-400
      case 'warning':
        return '#fbbf24'; // yellow-400
      case 'limit':
        return '#f87171'; // red-400
      default:
        return '#9ca3af'; // gray-400
    }
  };

  // Find joint positions in the URDF hierarchy
  const getJointPosition = (jointName: string): THREE.Vector3 | null => {
    let jointObject: any = null;

    // URDF uses L1-L6, we use J1-J6, so map: J1 -> L1
    const urdfJointName = 'L' + jointName.substring(1);

    urdfRobot.traverse((child: any) => {
      if (child.isURDFJoint && child.name === urdfJointName) {
        jointObject = child;
      }
    });

    if (jointObject) {
      // Get world position and apply inverse transformation
      // The robot is inside a group with rotation={[-Math.PI / 2, 0, 0]}
      const worldPos = new THREE.Vector3();
      jointObject.getWorldPosition(worldPos);

      // Apply inverse transformation: Z → -Y, Y → Z
      const rotatedPos = new THREE.Vector3();
      rotatedPos.x = worldPos.x;
      rotatedPos.y = -worldPos.z;  // Z becomes -Y
      rotatedPos.z = worldPos.y;   // Y becomes Z

      return rotatedPos;
    }

    return null;
  };

  return (
    <>
      {JOINT_NAMES.map((jointName) => {
        const position = getJointPosition(jointName);
        if (!position) return null;

        const value = commandedJointAngles[jointName];
        const status = getJointStatus(jointName, value);
        const color = getStatusColor(status);
        const isSelected = selectedJoint === jointName;

        return (
          <Html
            key={jointName}
            position={[position.x, position.y + 0.05, position.z]}
            center
            distanceFactor={2.5}
            style={{
              pointerEvents: 'none',
              userSelect: 'none',
            }}
          >
            <div
              style={{
                background: isSelected ? 'rgba(59, 130, 246, 0.9)' : 'rgba(0, 0, 0, 0.8)',
                padding: '2px 4px',
                borderRadius: '3px',
                border: isSelected ? '1px solid #3b82f6' : 'none',
                minWidth: '30px',
                textAlign: 'center',
                fontSize: '6px',
              }}
            >
              <div
                style={{
                  color: 'white',
                  fontWeight: 'bold',
                  marginBottom: '1px',
                }}
              >
                {jointName}
              </div>
              <div
                style={{
                  color: color,
                  fontWeight: '600',
                  fontFamily: 'monospace',
                }}
              >
                {value.toFixed(1)}°
              </div>
            </div>
          </Html>
        );
      })}
    </>
  );
}
