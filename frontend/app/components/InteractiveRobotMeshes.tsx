import { useEffect } from 'react';
import { useFrame, ThreeEvent } from '@react-three/fiber';
import { useInputStore } from '../lib/stores';
import { useJointContextMenu } from './JointContextMenu';
import type { JointName } from '../lib/types';

interface InteractiveRobotMeshesProps {
  robot: any;
}

export default function InteractiveRobotMeshes({ robot }: InteractiveRobotMeshesProps) {
  const setSelectedJoint = useInputStore((state) => state.setSelectedJoint);
  const { show: showContextMenu } = useJointContextMenu();

  useEffect(() => {
    if (!robot) return;

    // Make all meshes with jointName userData interactive
    robot.traverse((child: any) => {
      if (child.isMesh && child.userData.jointName) {
        // Enable R3F event handling
        child.addEventListener = () => {};
        child.hasEventListener = () => true;
        child.raycast = child.raycast || (() => {});
      }
    });
  }, [robot]);

  const handleMeshClick = (event: ThreeEvent<MouseEvent>) => {
    const jointName = event.object.userData?.jointName as JointName;
    if (jointName) {
      event.stopPropagation();
      setSelectedJoint(jointName);
    }
  };

  const handleMeshContextMenu = (event: ThreeEvent<MouseEvent>) => {
    const jointName = event.object.userData?.jointName as JointName;
    if (jointName) {
      event.stopPropagation();
      event.nativeEvent.preventDefault();
      setSelectedJoint(jointName);
      showContextMenu({
        event: event.nativeEvent,
        props: { jointName },
      });
    }
  };

  return (
    <primitive
      object={robot}
      onClick={handleMeshClick}
      onContextMenu={handleMeshContextMenu}
    />
  );
}
