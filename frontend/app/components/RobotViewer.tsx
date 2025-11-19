'use client';

import { useEffect, useRef, Suspense, useState } from 'react';
import { Canvas, ThreeEvent } from '@react-three/fiber';
import { OrbitControls, Grid, GizmoHelper, GizmoViewport } from '@react-three/drei';
import * as THREE from 'three';
import { useInputStore, useCommandStore, useHardwareStore, useTimelineStore, useRobotConfigStore } from '@/app/lib/stores';
import { useKinematicsStore } from '@/app/lib/stores/kinematicsStore';
import TargetPoseVisualizer from './TargetPoseVisualizer';
import TargetTCPVisualizer from './TargetTCPVisualizer';
import ActualTCPVisualizer from './ActualTCPVisualizer';
import PathVisualizer from './PathVisualizer';
import IKProgressBar from './IKProgressBar';
import JointLabels from './JointLabels';
import InteractiveRobotMeshes from './InteractiveRobotMeshes';
import { JointContextMenu } from './JointContextMenu';
import { TCPPoseDisplay, TCPPoseHeader } from './TCPPoseDisplay';
import { MemoryMonitor, WebGLContextMonitor } from './MemoryMonitor';
import { JOINT_LIMITS, JOINT_ANGLE_OFFSETS, CARTESIAN_LIMITS } from '../lib/constants';
import type { JointName, CartesianAxis } from '../lib/types';
import { inverseKinematicsDetailed } from '../lib/kinematics';
import { getHomePosition, getAllPositions } from '../lib/positions';
import { getApiBaseUrl } from '../lib/apiConfig';
import { logger } from '../lib/logger';
import { moveJoints, executeTrajectory } from '../lib/api';
import { generateCartesianWaypoints, calculateWaypointCount } from '../lib/cartesianPlanner';
import { useSafetyConfirmation } from '../hooks/useSafetyConfirmation';
import { calculateTcpPoseFromUrdf } from '../lib/tcpCalculations';
import { threeJsToRobot } from '../lib/coordinateTransform';
import { applyJointAnglesToUrdf } from '../lib/urdfHelpers';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Dialog, DialogContent, DialogDescription, DialogHeader, DialogTitle, DialogFooter } from '@/components/ui/dialog';
import { DropdownMenu, DropdownMenuTrigger, DropdownMenuContent, DropdownMenuCheckboxItem } from '@/components/ui/dropdown-menu';
import { Plus, Settings } from 'lucide-react';
import { cn } from '@/lib/utils';
import { useConfigStore } from '../lib/configStore';

// @ts-ignore - urdf-loader doesn't have proper types
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three-stdlib';
import { useFrame } from '@react-three/fiber';

interface Tool {
  id: string;
  name: string;
  description: string;
  mesh_file: string | null;
  mesh_units?: 'mm' | 'm';
  mesh_offset: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
  tcp_offset: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
  gripper_config?: {
    enabled: boolean;
    io_pin: number;
    open_is_high: boolean;
    mesh_file_open: string | null;
    mesh_file_closed: string | null;
  };
}

// Tool Mesh Component - Positioned at J6 joint
// Supports both single mesh tools and gripper tools with open/closed states
function ToolMesh({
  geometry,
  gripperMeshOpen,
  gripperMeshClosed,
  displayState,
  robotRef,
  tool,
  color
}: {
  geometry: THREE.BufferGeometry | null;
  gripperMeshOpen?: THREE.BufferGeometry | null;
  gripperMeshClosed?: THREE.BufferGeometry | null;
  displayState?: 'open' | 'closed';
  robotRef: any;
  tool: Tool | null;
  color: string;
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  const gripperOpenRef = useRef<THREE.Mesh>(null);
  const gripperClosedRef = useRef<THREE.Mesh>(null);

  // Determine which mesh to show
  const hasGripperGeometries = gripperMeshOpen || gripperMeshClosed;
  const showMain = !hasGripperGeometries && geometry;
  const showOpen = hasGripperGeometries && displayState === 'open';
  const showClosed = hasGripperGeometries && displayState === 'closed';

  // Update tool mesh position/rotation to match L6 link every frame
  useFrame(() => {
    const refs = [meshRef, gripperOpenRef, gripperClosedRef];

    for (const ref of refs) {
      if (!ref.current || !robotRef) continue;

      const l6Link = robotRef.links?.['L6'];
      if (!l6Link) continue;

      l6Link.updateMatrixWorld(true);

      // Get L6 link's world transform
      const l6WorldPosition = new THREE.Vector3();
      const l6WorldQuaternion = new THREE.Quaternion();
      l6Link.getWorldPosition(l6WorldPosition);
      l6Link.getWorldQuaternion(l6WorldQuaternion);

      // Apply position offset in L6's local frame (rotates with J6)
      if (tool?.mesh_offset) {
        const offset = tool.mesh_offset;

        // Apply position offset rotated by L6 quaternion so it moves with J6
        if (offset.x !== 0 || offset.y !== 0 || offset.z !== 0) {
          const localOffset = new THREE.Vector3(offset.x, offset.y, offset.z);
          const worldOffset = localOffset.applyQuaternion(l6WorldQuaternion);
          ref.current.position.copy(l6WorldPosition).add(worldOffset);
        } else {
          ref.current.position.copy(l6WorldPosition);
        }
      } else {
        ref.current.position.copy(l6WorldPosition);
      }

      // Set base rotation from L6
      ref.current.quaternion.copy(l6WorldQuaternion);

      // Apply L6 visual origin rotation from URDF: rpy="0 0 -1.5708" (-90° Z)
      const visualOriginRotation = new THREE.Quaternion();
      visualOriginRotation.setFromEuler(new THREE.Euler(0, 0, -Math.PI / 2, 'XYZ'));
      ref.current.quaternion.multiply(visualOriginRotation);

      // Apply tool's mesh rotation offset
      if (tool?.mesh_offset) {
        const offset = tool.mesh_offset;

        if (offset.rx !== 0 || offset.ry !== 0 || offset.rz !== 0) {
          const offsetRotation = new THREE.Euler(
            (offset.rx * Math.PI) / 180,
            (offset.ry * Math.PI) / 180,
            (offset.rz * Math.PI) / 180,
            'XYZ'
          );
          const offsetQuat = new THREE.Quaternion();
          offsetQuat.setFromEuler(offsetRotation);
          ref.current.quaternion.multiply(offsetQuat);
        }
      }
    }
  });

  return (
    <>
      {/* Single mesh (for non-gripper tools) */}
      {showMain && geometry && (
        <mesh ref={meshRef} geometry={geometry}>
          <meshStandardMaterial
            color={color}
            metalness={0.3}
            roughness={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {/* Gripper open state */}
      {showOpen && gripperMeshOpen && (
        <mesh ref={gripperOpenRef} geometry={gripperMeshOpen}>
          <meshStandardMaterial
            color={color}
            metalness={0.3}
            roughness={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {/* Gripper closed state */}
      {showClosed && gripperMeshClosed && (
        <mesh ref={gripperClosedRef} geometry={gripperMeshClosed}>
          <meshStandardMaterial
            color={color}
            metalness={0.3}
            roughness={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}
    </>
  );
}

interface URDFRobotProps {
  showLabels: boolean;
  hardwareRobotColor: string;
  hardwareRobotTransparency: number;
  commanderRobotColor: string;
  commanderRobotTransparency: number;
  hardwareJointAngles: any;
  activeToolId?: string;
}

function URDFRobot({ showLabels, hardwareRobotColor, hardwareRobotTransparency, commanderRobotColor, commanderRobotTransparency, hardwareJointAngles, activeToolId }: URDFRobotProps) {
  const robotRef = useRef<any>(null);
  const hardwareRobotRef = useRef<any>(null);
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const selectedJoint = useInputStore((state) => state.selectedJoint);
  const showHardwareRobot = useInputStore((state) => state.showHardwareRobot);
  const jointHomedStatus = useCommandStore((state) => state.jointHomedStatus);
  const showTargetRobot = useInputStore((state) => state.showTargetRobot);
  const commandedGripperState = useCommandStore((state) => state.commandedGripperState);

  // Tool mesh state
  const [toolMeshGeometry, setToolMeshGeometry] = useState<THREE.BufferGeometry | null>(null);
  const [gripperMeshOpen, setGripperMeshOpen] = useState<THREE.BufferGeometry | null>(null);
  const [gripperMeshClosed, setGripperMeshClosed] = useState<THREE.BufferGeometry | null>(null);
  const [currentTool, setCurrentTool] = useState<Tool | null>(null);

  // Helper function to properly dispose URDF robot and all its resources
  const disposeURDFRobot = (robot: any) => {
    if (!robot) return;

    robot.traverse((child: any) => {
      if (child.isMesh) {
        // Dispose geometry
        if (child.geometry) {
          child.geometry.dispose();
        }

        // Dispose materials (handle both single materials and arrays)
        if (child.material) {
          const materials = Array.isArray(child.material) ? child.material : [child.material];
          materials.forEach((mat: any) => {
            if (mat) {
              // Dispose textures if any
              if (mat.map) mat.map.dispose();
              if (mat.normalMap) mat.normalMap.dispose();
              if (mat.roughnessMap) mat.roughnessMap.dispose();
              if (mat.metalnessMap) mat.metalnessMap.dispose();
              // Dispose material itself
              mat.dispose();
            }
          });
        }
      }
    });
  };

  // Load URDF and create 3 robot instances
  // 1. Commander visual (colored robot showing commands)
  // 2. Hardware visual (ghost robot showing feedback)
  // 3. Computation headless (for FK/IK calculations)
  useEffect(() => {
    const loader = new URDFLoader();

    loader.load(
      '/urdf/PAROL6.urdf',
      (loadedRobot: any) => {
        logger.debug('URDF loaded successfully', 'RobotSetup');

        // 1. Commander visual robot (use loaded robot directly)
        const commanderRobot = loadedRobot;
        robotRef.current = commanderRobot;
        useCommandStore.setState({ commanderRobotRef: commanderRobot });

        // 2. Hardware visual robot (clone)
        const hardwareRobot = loadedRobot.clone();
        hardwareRobotRef.current = hardwareRobot;
        useHardwareStore.setState({ hardwareRobotRef: hardwareRobot });

        // 3. Computation robot (clone, headless - not added to scene)
        const computationRobot = loadedRobot.clone();
        // IMPORTANT: Apply the same rotation as the visual robots for coordinate consistency
        // Visual robots are in <group rotation={[-Math.PI / 2, 0, 0]}>
        // So we must apply the same rotation to computation robot for FK/IK to match
        computationRobot.rotation.set(-Math.PI / 2, 0, 0);
        computationRobot.updateMatrixWorld(true); // Update transforms after rotation
        useKinematicsStore.setState({ computationRobotRef: computationRobot });

        // Count initial meshes in both robots
        let commanderMeshCount = 0;
        let hardwareMeshCount = 0;
        commanderRobot.traverse((child: any) => { if (child.isMesh) commanderMeshCount++; });
        hardwareRobot.traverse((child: any) => { if (child.isMesh) hardwareMeshCount++; });
        logger.debug(`Initial mesh count - Commander: ${commanderMeshCount}, Hardware: ${hardwareMeshCount}`, 'RobotSetup');

        // Wait a bit for meshes to load, then setup both robots
        setTimeout(() => {
          // === Setup Commander Robot ===
          // Remove L6 mesh only (keep the link for joint transform)
          let commanderL6Removed = 0;
          commanderRobot.traverse((child: any) => {
            if (child.isMesh) {
              let parent = child.parent;
              while (parent) {
                if (parent.isURDFLink && parent.name === 'L6') {
                  child.parent.remove(child);
                  commanderL6Removed++;
                  break;
                }
                parent = parent.parent;
              }
            }
          });
          logger.debug(`Commander: Removed ${commanderL6Removed} L6 meshes`, 'RobotSetup');

          // Setup meshes with event handlers and coloring
          commanderRobot.traverse((child: any) => {
            if (child.isMesh) {
              // Find which joint this mesh belongs to
              let jointName = null;
              let parent = child.parent;
              while (parent) {
                if (parent.isURDFJoint) {
                  const match = parent.name.match(/L(\d)/);
                  if (match) {
                    jointName = `J${match[1]}` as JointName;
                  }
                  break;
                }
                parent = parent.parent;
              }

              if (jointName) {
                child.userData.jointName = jointName;

                // Clone material for independent coloring
                if (child.material) {
                  const materials = Array.isArray(child.material) ? child.material : [child.material];
                  const newMaterials: any[] = [];
                  materials.forEach((mat: any) => {
                    if (mat) {
                      const clonedMat = mat.clone();
                      clonedMat.userData.isCloned = true;

                      // Apply commander robot colors from store
                      const commanderColor = useRobotConfigStore.getState().commanderRobotColor;
                      const commanderTransp = useRobotConfigStore.getState().commanderRobotTransparency;
                      clonedMat.color.set(commanderColor);
                      clonedMat.transparent = commanderTransp < 1.0;
                      clonedMat.opacity = commanderTransp;
                      clonedMat.depthWrite = commanderTransp >= 1.0;
                      clonedMat.needsUpdate = true;

                      newMaterials.push(clonedMat);
                    }
                  });
                  child.material = Array.isArray(child.material) ? newMaterials : newMaterials[0];
                }

                // Enable raycasting for this mesh so it can be clicked
                child.userData.clickable = true;
              }
            }
          });

          // Set initial joint positions for commander robot
          const initialAngles = useCommandStore.getState().commandedJointAngles;
          applyJointAnglesToUrdf(commanderRobot, initialAngles);
          commanderRobot.updateMatrixWorld(true);

          // === Setup Hardware Robot ===
          // Remove L6 mesh only
          let hardwareL6Removed = 0;
          hardwareRobot.traverse((child: any) => {
            if (child.isMesh) {
              let parent = child.parent;
              while (parent) {
                if (parent.isURDFLink && parent.name === 'L6') {
                  child.parent.remove(child);
                  hardwareL6Removed++;
                  break;
                }
                parent = parent.parent;
              }
            }
          });
          logger.debug(`Hardware: Removed ${hardwareL6Removed} L6 meshes`, 'RobotSetup');

          // Make all hardware meshes transparent with ghost styling
          let hardwareMeshesProcessed = 0;
          let hardwareMeshesWithJoint = 0;
          let hardwareMeshesWithoutJoint = 0;
          hardwareRobot.traverse((child: any) => {
            if (child.isMesh) {
              hardwareMeshesProcessed++;
              // Mark this mesh as part of hardware robot
              child.userData.isActual = true;

              // Find which joint this mesh belongs to
              let jointName = null;
              let parent = child.parent;
              while (parent) {
                if (parent.isURDFJoint) {
                  const match = parent.name.match(/L(\d)/);
                  if (match) {
                    jointName = `J${match[1]}` as JointName;
                  }
                  break;
                }
                parent = parent.parent;
              }
              if (jointName) {
                child.userData.jointName = jointName;
                hardwareMeshesWithJoint++;
              } else {
                hardwareMeshesWithoutJoint++;
              }

              // Handle both single materials and material arrays
              const materials = Array.isArray(child.material) ? child.material : [child.material];
              const newMaterials: any[] = [];
              materials.forEach((mat: any) => {
                if (mat) {
                  const clonedMat = mat.clone();
                  clonedMat.transparent = true;

                  // Base (no jointName) should be fully transparent
                  // Joints use configured hardware robot color and transparency
                  if (!jointName) {
                    clonedMat.opacity = 0;
                  } else {
                    const opacity = useRobotConfigStore.getState().hardwareRobotTransparency;
                    const color = useRobotConfigStore.getState().hardwareRobotColor;
                    clonedMat.opacity = opacity;
                    clonedMat.color.set(color);
                  }

                  clonedMat.depthWrite = false;
                  clonedMat.side = THREE.DoubleSide;
                  clonedMat.userData.isActualMaterial = true;
                  clonedMat.needsUpdate = true;

                  newMaterials.push(clonedMat);
                }
              });

              child.material = Array.isArray(child.material) ? newMaterials : newMaterials[0];
              child.renderOrder = -1; // Render ghost first

              // Log details for first few meshes
              if (hardwareMeshesProcessed <= 3) {
                logger.debug(`Hardware mesh #${hardwareMeshesProcessed}: joint=${jointName}, hasGeom=${!!child.geometry}, opacity=${jointName ? useRobotConfigStore.getState().hardwareRobotTransparency : 0}`, 'RobotSetup');
              }
            }
          });
          logger.debug(`Hardware: Processed ${hardwareMeshesProcessed} meshes (${hardwareMeshesWithJoint} with jointName, ${hardwareMeshesWithoutJoint} without)`, 'RobotSetup');

          // Set initial joint positions for hardware robot
          const hardwareInitialAngles = getHomePosition();
          applyJointAnglesToUrdf(hardwareRobot, hardwareInitialAngles);
          hardwareRobot.updateMatrixWorld(true);
        }, 500); // Wait 500ms for meshes to load
      }
    );

    // Cleanup function to dispose robot resources when component unmounts
    return () => {
      disposeURDFRobot(robotRef.current);
      disposeURDFRobot(hardwareRobotRef.current);
      robotRef.current = null;
      hardwareRobotRef.current = null;
      useCommandStore.setState({ commanderRobotRef: null });
      useHardwareStore.setState({ hardwareRobotRef: null });
    };
  }, []);

  // Fetch and load active tool mesh
  useEffect(() => {
    const fetchAndLoadTool = async () => {
      try {
        // Fetch active tool from backend
        const response = await fetch(`${getApiBaseUrl()}/api/config/tools`);
        if (!response.ok) return;

        const data = await response.json();
        const toolId = activeToolId || data.active_tool_id;

        // Find the active tool
        const tool = data.tools?.find((t: Tool) => t.id === toolId);
        if (!tool) return;

        setCurrentTool(tool);

        // CRITICAL: Sync tool to all three stores for consistent FK/IK
        useCommandStore.setState({ commanderTool: tool });
        useHardwareStore.setState({ hardwareTool: tool });
        useKinematicsStore.setState({ computationTool: tool });
        logger.debug(`Synced tool to all stores: ${tool.name}`, 'RobotViewer');

        // Load tool mesh if it has one (for non-gripper tools)
        if (tool.mesh_file && !tool.gripper_config?.enabled) {
          const meshResponse = await fetch(`/urdf/meshes/${tool.mesh_file}`);
          if (!meshResponse.ok) return;

          const arrayBuffer = await meshResponse.arrayBuffer();
          const loader = new STLLoader();
          const geometry = loader.parse(arrayBuffer);

          // Apply unit scaling only if mesh is in mm
          const units = tool.mesh_units || 'mm';
          if (units === 'mm') {
            geometry.scale(0.001, 0.001, 0.001);
          }

          setToolMeshGeometry(geometry);
          setGripperMeshOpen(null);
          setGripperMeshClosed(null);
        } else if (tool.gripper_config?.enabled) {
          // Load gripper meshes (open and closed states)
          const units = tool.mesh_units || 'mm';
          const scale = units === 'mm' ? 0.001 : 1;

          // Load open state mesh
          if (tool.gripper_config.mesh_file_open) {
            try {
              const meshResponse = await fetch(`/urdf/meshes/${tool.gripper_config.mesh_file_open}`);
              if (meshResponse.ok) {
                const arrayBuffer = await meshResponse.arrayBuffer();
                const loader = new STLLoader();
                const geometry = loader.parse(arrayBuffer);
                geometry.scale(scale, scale, scale);
                setGripperMeshOpen(geometry);
              } else {
                setGripperMeshOpen(null);
              }
            } catch (error) {
              logger.error('Failed to load gripper open mesh', 'RobotViewer', error);
              setGripperMeshOpen(null);
            }
          } else {
            setGripperMeshOpen(null);
          }

          // Load closed state mesh
          if (tool.gripper_config.mesh_file_closed) {
            try {
              const meshResponse = await fetch(`/urdf/meshes/${tool.gripper_config.mesh_file_closed}`);
              if (meshResponse.ok) {
                const arrayBuffer = await meshResponse.arrayBuffer();
                const loader = new STLLoader();
                const geometry = loader.parse(arrayBuffer);
                geometry.scale(scale, scale, scale);
                setGripperMeshClosed(geometry);
              } else {
                setGripperMeshClosed(null);
              }
            } catch (error) {
              logger.error('Failed to load gripper closed mesh', 'RobotViewer', error);
              setGripperMeshClosed(null);
            }
          } else {
            setGripperMeshClosed(null);
          }

          // Clear single mesh state for gripper tools
          setToolMeshGeometry(null);
        } else {
          // No mesh at all
          setToolMeshGeometry(null);
          setGripperMeshOpen(null);
          setGripperMeshClosed(null);
        }
      } catch (error) {
        logger.error('Failed to load tool mesh', 'RobotViewer', error);
      }
    };

    fetchAndLoadTool();
  }, [activeToolId]);

  // Update joint angles for target robot + apply coloring
  useEffect(() => {
    if (!robotRef.current) return;

    // Apply commanded joint angles to commander robot using centralized helper
    applyJointAnglesToUrdf(robotRef.current, commandedJointAngles);

    // Apply coloring to joint meshes (separate pass for clarity)
    let coloredMeshCount = 0;
    let skippedActualCount = 0;

    robotRef.current.traverse((child: any) => {
      if (child.isMesh) {
        // Check if this is a ghost mesh that should be skipped
        if (child.userData.isActual) {
          skippedActualCount++;
          return; // Skip ghost meshes!
        }

        // Only highlight selected joint (no color coding based on position)
        if (child.userData.jointName && child.material) {
          coloredMeshCount++;
          const jointKey = child.userData.jointName as JointName;

          // Add emissive for selection only
          if (selectedJoint === jointKey) {
            child.material.emissive.setHex(0xf97316); // orange-500 - contrasts well with blue transparent actual robot
            child.material.emissiveIntensity = 0.5;
          } else {
            child.material.emissive.setHex(0x000000);
            child.material.emissiveIntensity = 0;
          }

          child.material.needsUpdate = true;
        }
      }
    });
  }, [commandedJointAngles, selectedJoint]);

  // Update hardware robot from hardware feedback (WebSocket data)
  useEffect(() => {
    if (!hardwareRobotRef.current) return;

    // Use hardware joint angles from robot feedback, or fallback to home position
    const angles = hardwareJointAngles || getHomePosition();

    // Apply hardware joint angles using centralized helper
    applyJointAnglesToUrdf(hardwareRobotRef.current, angles);
  }, [hardwareJointAngles]);

  // Update hardware robot appearance when config changes
  useEffect(() => {
    if (!hardwareRobotRef.current) return;

    hardwareRobotRef.current.traverse((child: any) => {
      if (child.isMesh && child.userData.isActual && child.userData.jointName) {
        // Handle both single materials and material arrays
        const materials = Array.isArray(child.material) ? child.material : [child.material];

        materials.forEach((mat: any) => {
          if (mat && mat.userData.isActualMaterial) {
            // Update color and transparency from config
            mat.color.set(hardwareRobotColor);
            mat.transparent = true;
            mat.opacity = hardwareRobotTransparency;
            mat.depthWrite = false;
            mat.needsUpdate = true;
          }
        });
      }
    });
  }, [hardwareRobotColor, hardwareRobotTransparency]);

  // Update commander robot appearance when config changes
  useEffect(() => {
    if (!robotRef.current) return;

    robotRef.current.traverse((child: any) => {
      if (child.isMesh && !child.userData.isActual && child.userData.jointName) {
        // Handle both single materials and material arrays
        const materials = Array.isArray(child.material) ? child.material : [child.material];

        materials.forEach((mat: any) => {
          if (mat && mat.userData.isCloned) {
            // Update color and transparency from config
            mat.color.set(commanderRobotColor);
            mat.transparent = commanderRobotTransparency < 1.0;
            mat.opacity = commanderRobotTransparency;
            mat.depthWrite = commanderRobotTransparency >= 1.0;
            mat.needsUpdate = true;
          }
        });
      }
    });
  }, [commanderRobotColor, commanderRobotTransparency]);

  // Debug hardware robot rendering conditions
  useEffect(() => {
    logger.debug(`Hardware robot render check: showHardwareRobot=${showHardwareRobot}, hardwareRobotRef=${!!hardwareRobotRef.current}, shouldRender=${showHardwareRobot && !!hardwareRobotRef.current}`, 'RobotRender');
    if (hardwareRobotRef.current) {
      let meshCount = 0;
      hardwareRobotRef.current.traverse((child: any) => { if (child.isMesh) meshCount++; });
      logger.debug(`Hardware robot has ${meshCount} meshes in scene`, 'RobotRender');
    }
  }, [showHardwareRobot, hardwareRobotRef.current]);

  return (
    <>
      <group rotation={[-Math.PI / 2, 0, 0]}>
        {/* Hardware robot (transparent, shows hardware feedback) */}
        {showHardwareRobot && hardwareRobotRef.current && <primitive object={hardwareRobotRef.current} />}

        {/* Target robot with interactive meshes (shows commanded position) */}
        {showTargetRobot && robotRef.current && <InteractiveRobotMeshes robot={robotRef.current} />}
      </group>

      {/* Tool mesh - attached to target robot (outside rotated group) */}
      {showTargetRobot && robotRef.current && (
        <ToolMesh
          geometry={toolMeshGeometry}
          gripperMeshOpen={gripperMeshOpen}
          gripperMeshClosed={gripperMeshClosed}
          displayState={commandedGripperState}
          robotRef={robotRef.current}
          tool={currentTool}
          color={commanderRobotColor}
        />
      )}

      {/* Tool mesh - attached to hardware robot (outside rotated group) */}
      {showHardwareRobot && hardwareRobotRef.current && (
        <ToolMesh
          geometry={toolMeshGeometry}
          gripperMeshOpen={gripperMeshOpen}
          gripperMeshClosed={gripperMeshClosed}
          displayState={commandedGripperState}
          robotRef={hardwareRobotRef.current}
          tool={currentTool}
          color={hardwareRobotColor}
        />
      )}

      <Grid args={[10, 10]} cellColor="#6b6b6b" sectionColor="#3f3f3f" />
    </>
  );
}

export default function RobotViewer({ activeToolId }: { activeToolId?: string } = {}) {
  // Input store: User input state
  const inputJointAngles = useInputStore((state) => state.inputJointAngles);
  const setInputJointAngle = useInputStore((state) => state.setInputJointAngle);
  const inputCartesianPose = useInputStore((state) => state.inputCartesianPose);
  const setInputCartesianValue = useInputStore((state) => state.setInputCartesianValue);
  const selectedJoint = useInputStore((state) => state.selectedJoint);
  const setSelectedJoint = useInputStore((state) => state.setSelectedJoint);
  const showHardwareRobot = useInputStore((state) => state.showHardwareRobot);
  const setShowHardwareRobot = useInputStore((state) => state.setShowHardwareRobot);
  const showTargetRobot = useInputStore((state) => state.showTargetRobot);
  const setShowTargetRobot = useInputStore((state) => state.setShowTargetRobot);
  const showFirmwareCoordinates = useInputStore((state) => state.showFirmwareCoordinates);
  const setShowFirmwareCoordinates = useInputStore((state) => state.setShowFirmwareCoordinates);
  const showPath = useInputStore((state) => state.showPath);
  const setShowPath = useInputStore((state) => state.setShowPath);

  // Command store: Commanded robot state
  const commandedJointAngles = useCommandStore((state) => state.commandedJointAngles);
  const setCommandedJointAngle = useCommandStore((state) => state.setCommandedJointAngle);
  const setCommandedJointAngles = useCommandStore((state) => state.setCommandedJointAngles);
  const commandedTcpPose = useCommandStore((state) => state.commandedTcpPose);
  const commanderRobotRef = useCommandStore((state) => state.commanderRobotRef);
  const teachModeEnabled = useCommandStore((state) => state.teachModeEnabled);
  const setTeachModeEnabled = useCommandStore((state) => state.setTeachModeEnabled);
  const liveControlEnabled = useCommandStore((state) => state.liveControlEnabled);
  const setLiveControlEnabled = useCommandStore((state) => state.setLiveControlEnabled);
  const speed = useCommandStore((state) => state.speed);

  // Hardware store: Hardware feedback
  const hardwareJointAngles = useHardwareStore((state) => state.hardwareJointAngles);
  const hardwareTcpPose = useHardwareStore((state) => state.hardwareTcpPose);
  const hardwareCartesianPose = useHardwareStore((state) => state.hardwareCartesianPose);
  const connectionStatus = useHardwareStore((state) => state.connectionStatus);
  const robotStatus = useHardwareStore((state) => state.robotStatus);

  // Timeline store: Timeline state
  const motionMode = useTimelineStore((state) => state.timeline.mode);

  // Config store: Robot configuration
  const tcpOffset = useRobotConfigStore((state) => state.tcpOffset);
  const ikAxisMask = useRobotConfigStore((state) => state.ikAxisMask);
  const hardwareRobotColor = useRobotConfigStore((state) => state.hardwareRobotColor);
  const hardwareRobotTransparency = useRobotConfigStore((state) => state.hardwareRobotTransparency);
  const commanderRobotColor = useRobotConfigStore((state) => state.commanderRobotColor);
  const commanderRobotTransparency = useRobotConfigStore((state) => state.commanderRobotTransparency);

  // Kinematics store: Computation robot for IK/FK
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);

  // Safety confirmation hook
  const { confirmAction, SafetyDialog } = useSafetyConfirmation();

  // Step angles for keyboard controls
  const stepAngle = useInputStore((state) => state.stepAngle);
  const cartesianPositionStep = useInputStore((state) => state.cartesianPositionStep);

  const [showLabels, setShowLabels] = useState(true);

  // Record preset dialog state
  const [recordDialogOpen, setRecordDialogOpen] = useState(false);
  const [presetName, setPresetName] = useState('');

  // Config store for saving presets
  const { config, saveConfig } = useConfigStore();

  // Get all saved positions from config
  const savedPositions = getAllPositions();

  // Track last valid cartesian pose for IK failure recovery
  const lastValidCartesianPose = useRef(inputCartesianPose);

  // Handle going to a preset position
  const handleGoToPosition = async (joints: { J1: number; J2: number; J3: number; J4: number; J5: number; J6: number }, presetName: string) => {
    // Safety confirmation check
    const confirmed = await confirmAction(
      `Move robot to "${presetName}" preset position?`,
      'Confirm Preset Movement'
    );
    if (!confirmed) return;

    // Set both input and commanded joint positions
    Object.entries(joints).forEach(([joint, angle]) => {
      setInputJointAngle(joint as any, angle);
      setCommandedJointAngle(joint as any, angle);
    });

    // If in cartesian mode, also update cartesian input sliders and RGB gizmo
    // to match the FK of the joint angles
    if (motionMode === 'cartesian' && commanderRobotRef) {
      // Apply joint angles to commander robot using centralized helper
      applyJointAnglesToUrdf(commanderRobotRef, joints);

      // Calculate FK to get cartesian position
      const threeJsPose = calculateTcpPoseFromUrdf(commanderRobotRef, tcpOffset);

      if (threeJsPose) {
        // Convert Three.js coords to robot coords
        const robotPose = threeJsToRobot(threeJsPose);

        // Update cartesian input sliders to match
        setInputCartesianValue('X', robotPose.X);
        setInputCartesianValue('Y', robotPose.Y);
        setInputCartesianValue('Z', robotPose.Z);
        setInputCartesianValue('RX', robotPose.RX);
        setInputCartesianValue('RY', robotPose.RY);
        setInputCartesianValue('RZ', robotPose.RZ);
      }
    }
  };

  // Handle record button click - open dialog
  const handleRecordClick = () => {
    setPresetName('');
    setRecordDialogOpen(true);
  };

  // Save preset to config.yaml
  const handleSavePreset = async () => {
    if (!presetName.trim()) {
      alert('Please enter a name for the preset');
      return;
    }

    // Get current commanded joint angles
    const newPreset = {
      name: presetName.trim(),
      joints: [
        commandedJointAngles.J1,
        commandedJointAngles.J2,
        commandedJointAngles.J3,
        commandedJointAngles.J4,
        commandedJointAngles.J5,
        commandedJointAngles.J6,
      ],
    };

    // Add to existing presets
    const currentPresets = config?.ui?.saved_positions || [];
    const updatedPresets = [...currentPresets, newPreset];

    try {
      // Save config with updated presets - saveConfig handles the PATCH request and store update
      await saveConfig({
        ui: {
          ...config?.ui,
          saved_positions: updatedPresets,
        } as any,
      });

      // Close dialog and reset name
      setRecordDialogOpen(false);
      setPresetName('');
    } catch (error) {
      logger.error('Error saving preset', 'RobotViewer', error);
      alert('Failed to save preset. Please try again.');
    }
  };

  // Keyboard controls for joint adjustment
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Ignore keyboard events when typing in input fields
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement ||
        (event.target as HTMLElement).isContentEditable
      ) {
        return;
      }

      // Handle Escape key to deselect
      if (event.key === 'Escape') {
        setSelectedJoint(null);
        return;
      }

      // Handle number keys 1-6 to select joints J1-J6 (but not Alt+number for presets)
      if (event.key >= '1' && event.key <= '6' && !event.altKey) {
        const jointNumber = parseInt(event.key);
        const jointName = `J${jointNumber}` as JointName;
        setSelectedJoint(jointName);
        return;
      }

      // Handle W/S keys for joint adjustment (only when joint is selected)
      if (selectedJoint && (event.key === 'w' || event.key === 'W' || event.key === 's' || event.key === 'S')) {
        event.preventDefault(); // Prevent page scrolling

        const currentAngle = inputJointAngles[selectedJoint];
        const limits = JOINT_LIMITS[selectedJoint];

        // Calculate step based on modifier keys
        let adjustmentStep = stepAngle;
        if (event.shiftKey) {
          // Shift: fine control (step / 10)
          adjustmentStep = stepAngle / 10;
        } else if (event.ctrlKey || event.metaKey) {
          // Ctrl/Cmd: coarse control (step × 5)
          adjustmentStep = stepAngle * 5;
        }

        // Apply direction (W increases, S decreases)
        const direction = (event.key === 'w' || event.key === 'W') ? 1 : -1;
        const newAngle = currentAngle + (direction * adjustmentStep);

        // Clamp to joint limits
        const clampedAngle = Math.max(limits.min, Math.min(limits.max, newAngle));

        // Update both input and commanded stores
        setInputJointAngle(selectedJoint, clampedAngle);
        setCommandedJointAngle(selectedJoint, clampedAngle);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [selectedJoint, inputJointAngles, stepAngle, setInputJointAngle, setCommandedJointAngle, setSelectedJoint]);

  // Keyboard controls for cartesian TCP adjustment (WASD-QE keys)
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      // Only active in cartesian mode
      if (motionMode !== 'cartesian') {
        return;
      }

      // Ignore keyboard events when typing in input fields
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement ||
        (event.target as HTMLElement).isContentEditable
      ) {
        return;
      }

      // Key mapping: WASD-QE for position, Alt+WASD-QE for rotation
      const keyMap: { [key: string]: { axis: CartesianAxis, direction: number, isRotation: boolean } } = {
        'a': { axis: 'Y', direction: -1, isRotation: false },
        'd': { axis: 'Y', direction: 1, isRotation: false },
        'w': { axis: 'X', direction: 1, isRotation: false },
        's': { axis: 'X', direction: -1, isRotation: false },
        'q': { axis: 'Z', direction: -1, isRotation: false },
        'e': { axis: 'Z', direction: 1, isRotation: false },
        'A': { axis: 'Y', direction: -1, isRotation: false },
        'D': { axis: 'Y', direction: 1, isRotation: false },
        'W': { axis: 'X', direction: 1, isRotation: false },
        'S': { axis: 'X', direction: -1, isRotation: false },
        'Q': { axis: 'Z', direction: -1, isRotation: false },
        'E': { axis: 'Z', direction: 1, isRotation: false },
      };

      const keyPressed = event.key;
      const keyConfig = keyMap[keyPressed.toLowerCase()];

      if (!keyConfig) {
        return;
      }

      event.preventDefault(); // Prevent page scrolling

      // Determine axis (switch to rotation if Alt is pressed)
      let axis: CartesianAxis = keyConfig.axis;
      if (event.altKey) {
        // Convert X/Y/Z to RX/RY/RZ
        axis = `R${keyConfig.axis}` as CartesianAxis;
      }

      // Calculate step size based on modifier keys and axis type
      const isRotationAxis = axis.startsWith('R');
      let step: number;

      if (isRotationAxis) {
        // Rotation steps in degrees (from step_angle setting)
        if (event.shiftKey) {
          step = stepAngle / 10; // Fine
        } else if (event.ctrlKey || event.metaKey) {
          step = stepAngle * 5; // Coarse
        } else {
          step = stepAngle; // Normal
        }
      } else {
        // Position steps in mm (from cartesian_position_step_mm setting)
        if (event.shiftKey) {
          step = cartesianPositionStep / 10; // Fine
        } else if (event.ctrlKey || event.metaKey) {
          step = cartesianPositionStep * 5; // Coarse
        } else {
          step = cartesianPositionStep; // Normal
        }
      }

      // Get current value and limits (read fresh from store to avoid stale closure)
      const currentValue = useInputStore.getState().inputCartesianPose[axis];
      const limits = CARTESIAN_LIMITS[axis];

      // Calculate new value
      const newValue = currentValue + (keyConfig.direction * step);

      // Clamp to limits
      const clampedValue = Math.max(limits.min, Math.min(limits.max, newValue));

      // Build new cartesian pose with updated value (use fresh store state)
      const currentPose = useInputStore.getState().inputCartesianPose;
      const newCartesianPose = { ...currentPose, [axis]: clampedValue };

      // Try to solve IK for the new pose
      if (!commanderRobotRef) {
        // If robot not loaded yet, just update the value
        setInputCartesianValue(axis, clampedValue);
        return;
      }

      const ikResult = inverseKinematicsDetailed(
        newCartesianPose,
        useCommandStore.getState().commandedJointAngles, // Use fresh state for IK seed
        computationRobotRef,
        computationTool,
        ikAxisMask
      );

      if (ikResult.success && ikResult.jointAngles) {
        // IK succeeded - update both cartesian input and commanded joint angles
        setInputCartesianValue(axis, clampedValue);
        setCommandedJointAngles(ikResult.jointAngles);

        // Store as last valid pose
        lastValidCartesianPose.current = newCartesianPose;
      } else {
        // IK failed - revert to last valid pose
        logger.warn('Failed to reach target position, reverting to last valid pose', 'IK');
        // Update input cartesian pose to last valid
        Object.entries(lastValidCartesianPose.current).forEach(([key, value]) => {
          setInputCartesianValue(key as CartesianAxis, value);
        });

        // Optional: Play error sound or show visual feedback
        // You could add a toast notification here
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [motionMode, inputCartesianPose, commandedJointAngles, setInputCartesianValue, setCommandedJointAngles, commanderRobotRef, tcpOffset, ikAxisMask]);

  // Spacebar shortcut: Move to Target (Joint) with safety confirmation
  useEffect(() => {
    const handleKeyDown = async (event: KeyboardEvent) => {
      // Ignore keyboard events when typing in input fields
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement ||
        (event.target as HTMLElement).isContentEditable
      ) {
        return;
      }

      // Handle spacebar key (but not Shift+Space, which is handled separately)
      if ((event.key === ' ' || event.code === 'Space') && !event.shiftKey) {
        event.preventDefault(); // Prevent page scrolling

        logger.debug('Space pressed, commanded angles', 'JointMotion', { commandedJointAngles });

        // Safety confirmation check
        const confirmed = await confirmAction(
          'Move robot to target position using joint space motion?',
          'Confirm Joint Movement'
        );
        if (!confirmed) {
          logger.debug('User cancelled', 'JointMotion');
          return;
        }

        // Execute joint movement
        try {
          logger.debug('Sending moveJoints command...', 'JointMotion');
          const result = await moveJoints(commandedJointAngles, speed);
          logger.debug('Result', 'JointMotion', result);
          if (!result.success) {
            alert(`Failed to move robot (joint): ${result.error || 'Unknown error'}`);
          } else {
            logger.debug('Command sent successfully', 'JointMotion');
          }
        } catch (error) {
          logger.error('Error sending move command', 'JointMotion', error);
          alert('Failed to communicate with robot');
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [commandedJointAngles, speed, confirmAction]);

  // Alt+number shortcuts: Go to preset positions
  useEffect(() => {
    const handleKeyDown = async (event: KeyboardEvent) => {
      // Ignore keyboard events when typing in input fields
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement ||
        (event.target as HTMLElement).isContentEditable
      ) {
        return;
      }

      // Handle Alt+number keys for preset positions
      if (event.altKey && event.key >= '1' && event.key <= '9') {
        event.preventDefault(); // Prevent browser shortcuts

        const presetIndex = parseInt(event.key) - 1; // Convert to 0-based index

        // Check if preset exists
        if (presetIndex < savedPositions.length) {
          const position = savedPositions[presetIndex];
          await handleGoToPosition(position.joints, position.name);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [savedPositions, handleGoToPosition]);

  // Shift+Spacebar shortcut: Execute Cartesian Motion using frontend IK
  useEffect(() => {
    const handleKeyDown = async (event: KeyboardEvent) => {
      // Ignore keyboard events when typing in input fields
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement ||
        (event.target as HTMLElement).isContentEditable
      ) {
        return;
      }

      // Handle Shift+Space for cartesian movement
      if ((event.key === ' ' || event.code === 'Space') && event.shiftKey) {
        event.preventDefault();

        // Get required data from stores
        const hardwareJointAngles = useHardwareStore.getState().hardwareJointAngles;
        const hardwareTcpPose = useHardwareStore.getState().hardwareTcpPose;
        const commandedJointAngles = useCommandStore.getState().commandedJointAngles;
        const commandedTcpPose = useCommandStore.getState().commandedTcpPose;
        const commanderRobotRef = useCommandStore.getState().commanderRobotRef;
        const tcpOffset = useRobotConfigStore.getState().tcpOffset;
        const ikAxisMask = useRobotConfigStore.getState().ikAxisMask;
        const speed = useCommandStore.getState().speed;

        // Validate prerequisites
        if (!hardwareJointAngles || !hardwareTcpPose || !commandedTcpPose) {
          alert('Robot pose not available - wait for URDF to load');
          return;
        }

        if (!commanderRobotRef) {
          alert('URDF robot model not loaded yet');
          return;
        }

        // Safety confirmation
        const confirmed = await confirmAction(
          'Move robot to target position using cartesian space motion (straight line)?',
          'Confirm Cartesian Movement'
        );
        if (!confirmed) return;

        // Execute cartesian movement using frontend IK
        try {
          logger.debug('Start pose', 'CartesianMotion', { pose: hardwareTcpPose });
          logger.debug('Target pose', 'CartesianMotion', { pose: commandedTcpPose });

          // Calculate duration based on speed percentage
          const baseDuration = 2.0; // Base duration in seconds
          const duration = baseDuration * (100 / speed);
          const numWaypoints = calculateWaypointCount(duration);

          logger.debug(`Generating ${numWaypoints} waypoints for ${duration.toFixed(2)}s (${speed}% speed)`, 'CartesianMotion');

          // Generate Cartesian waypoints (straight-line interpolation)
          const cartesianWaypoints = generateCartesianWaypoints(hardwareTcpPose, commandedTcpPose, { duration });

          logger.debug(`Generated ${cartesianWaypoints.length} waypoints`, 'CartesianMotion');

          // Solve IK for each waypoint using frontend solver
          logger.debug('Solving IK for waypoints...', 'CartesianMotion');
          const startTime = performance.now();
          const jointTrajectory: number[][] = [];
          let currentSeed = { ...hardwareJointAngles };

          for (let i = 0; i < cartesianWaypoints.length; i++) {
            const waypoint = cartesianWaypoints[i];

            // Solve IK using frontend numerical solver
            const ikResult = inverseKinematicsDetailed(
              waypoint,
              currentSeed,
              computationRobotRef,
              computationTool,
              ikAxisMask
            );

            if (!ikResult.success || !ikResult.jointAngles) {
              alert(`IK failed at waypoint ${i + 1}/${cartesianWaypoints.length}: ${ikResult.error?.message || 'Unknown error'}`);
              logger.error('IK failed at waypoint', 'CartesianMotion', { waypoint: i, result: ikResult });
              return;
            }

            // Add to trajectory as array [J1, J2, J3, J4, J5, J6]
            const joints = ikResult.jointAngles;
            jointTrajectory.push([joints.J1, joints.J2, joints.J3, joints.J4, joints.J5, joints.J6]);

            // Use this solution as seed for next waypoint (faster convergence)
            currentSeed = joints;
          }

          const elapsed = performance.now() - startTime;
          logger.debug(`IK solved for ${jointTrajectory.length} waypoints in ${elapsed.toFixed(0)}ms`, 'CartesianMotion');

          // Execute trajectory at 100Hz
          logger.debug('Executing trajectory at 100Hz...', 'CartesianMotion');
          const execResult = await executeTrajectory({
            trajectory: jointTrajectory,
            duration: duration,
            wait_for_ack: false
          });

          if (!execResult.success) {
            alert(`Failed to execute trajectory: ${execResult.message}`);
            logger.error('Execute failed', 'CartesianMotion', execResult);
          } else {
            logger.debug(`Trajectory executing (${jointTrajectory.length} waypoints)`, 'CartesianMotion');
          }

        } catch (error) {
          logger.error('Cartesian motion error', 'CartesianMotion', error);
          alert(`Cartesian motion failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [confirmAction]);

  // Handle click on canvas background to deselect joint
  const handleCanvasClick = (event: any) => {
    // Only deselect if clicking on canvas background (not on robot meshes)
    // InteractiveRobotMeshes will call stopPropagation() for mesh clicks
    setSelectedJoint(null);
  };

  return (
    <div className="w-full h-full bg-gray-950 rounded-lg overflow-hidden border relative">
      {/* IK Progress Bar Overlay */}
      <IKProgressBar />

      {/* Context Menu */}
      <JointContextMenu />

      {/* Coordinate Overlay */}
      <div className="absolute top-4 left-4 bg-black/70 text-white p-3 rounded-lg text-xs font-mono z-10 backdrop-blur-sm max-w-[420px]">
        {/* TCP Pose Section - Robot Coordinates (Z-up) */}
        <div className="mb-3">
          <div className="font-semibold mb-2 text-sm">TCP Pose (Robot Coords)</div>
          {commandedTcpPose || hardwareTcpPose || hardwareCartesianPose ? (
            <div>
              <TCPPoseHeader className="text-[10px] text-gray-400" />
              {commandedTcpPose && (
                <TCPPoseDisplay
                  pose={commandedTcpPose}
                  label="Commanded"
                  colors={{
                    x: '#ff8800',  // Orange
                    y: '#00dddd',  // Cyan
                    z: '#dd00dd'   // Magenta/Fuchsia
                  }}
                  className="mb-0.5"
                />
              )}
              <TCPPoseDisplay
                pose={hardwareJointAngles !== null ? hardwareTcpPose : null}
                label="Hardware"
                colors={{
                  x: '#ffff00',  // Yellow
                  y: '#88ff00',  // Lime
                  z: '#aa00ff'   // Purple
                }}
                className="mb-0.5"
              />
              {showFirmwareCoordinates && (
                <TCPPoseDisplay
                  pose={hardwareCartesianPose}
                  label="HW Firmware"
                  colors={{
                    x: '#00ff00',  // Green
                    y: '#00ffaa',  // Cyan-Green
                    z: '#00aaff'   // Sky Blue
                  }}
                />
              )}
            </div>
          ) : (
            <div className="text-gray-500 italic">Loading robot model...</div>
          )}
        </div>

        {/* Joint Angles Section */}
        <div className="pt-3 border-t border-gray-700">
          <div className="font-semibold mb-2 text-sm">Joint Angles</div>
          <div>
            {/* Column Headers */}
            <div className="grid grid-cols-7 gap-1 mb-1 text-[10px] text-gray-400 text-center">
              <div></div>
              <div>J1</div>
              <div>J2</div>
              <div>J3</div>
              <div>J4</div>
              <div>J5</div>
              <div>J6</div>
            </div>
            {/* Commanded Row */}
            <div className="grid grid-cols-7 gap-1 mb-0.5">
              <div className="text-gray-400">Commanded:  </div>
              <div className="text-center">{commandedJointAngles.J1.toFixed(1)}</div>
              <div className="text-center">{commandedJointAngles.J2.toFixed(1)}</div>
              <div className="text-center">{commandedJointAngles.J3.toFixed(1)}</div>
              <div className="text-center">{commandedJointAngles.J4.toFixed(1)}</div>
              <div className="text-center">{commandedJointAngles.J5.toFixed(1)}</div>
              <div className="text-center">{commandedJointAngles.J6.toFixed(1)}</div>
            </div>
            {/* Hardware Row */}
            <div className="grid grid-cols-7 gap-1">
              <div className="text-gray-400">Hardware:  </div>
              <div className="text-center">{hardwareJointAngles?.J1.toFixed(1) ?? 'N/A'}</div>
              <div className="text-center">{hardwareJointAngles?.J2.toFixed(1) ?? 'N/A'}</div>
              <div className="text-center">{hardwareJointAngles?.J3.toFixed(1) ?? 'N/A'}</div>
              <div className="text-center">{hardwareJointAngles?.J4.toFixed(1) ?? 'N/A'}</div>
              <div className="text-center">{hardwareJointAngles?.J5.toFixed(1) ?? 'N/A'}</div>
              <div className="text-center">{hardwareJointAngles?.J6.toFixed(1) ?? 'N/A'}</div>
            </div>
          </div>
        </div>
      </div>

      {/* Robot Status - Top Right */}
      <div className="absolute top-4 right-4 bg-black/70 text-white p-2 rounded-lg text-[10px] z-10 backdrop-blur-sm">
        <div className="space-y-0.5">
          <div className="flex items-center gap-1.5">
            <span className={connectionStatus === 'connected' ? 'text-green-500' : 'text-gray-500'}>●</span>
            <span className="text-gray-400">Commander:</span>
            <span className="font-medium">
              {connectionStatus === 'connected' ?
                `CONNECTED - ${robotStatus?.commander_hz?.toFixed(0) ?? '--'}Hz` :
                connectionStatus.toUpperCase()}
            </span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className={robotStatus?.is_stopped === false ? 'text-green-500' : 'text-gray-500'}>
              {robotStatus?.is_stopped == null ? '○' : '●'}
            </span>
            <span className="text-gray-400">Robot:</span>
            <span className="font-medium">
              {robotStatus?.is_stopped == null ? 'N/A' :
               robotStatus?.is_stopped === false ? 'RUNNING' : 'STOPPED'}
            </span>
          </div>
          <div className="flex items-center gap-1.5">
            <span className={robotStatus?.estop_active === false ? 'text-blue-500' :
                           robotStatus?.estop_active === true ? 'text-red-500' : 'text-gray-500'}>
              {robotStatus?.estop_active == null ? '-' :
               robotStatus?.estop_active === false ? '✓' : '✗'}
            </span>
            <span className="text-gray-400">E-STOP:</span>
            <span className="font-medium">
              {robotStatus?.estop_active == null ? 'N/A' :
               robotStatus?.estop_active === false ? 'OK' : 'ACTIVE'}
            </span>
          </div>
        </div>
      </div>

      {/* Display Controls - Gear Icon Dropdown */}
      <div className="absolute top-[110px] right-4 z-10">
        <DropdownMenu>
          <DropdownMenuTrigger asChild>
            <button className="bg-black/70 text-white p-2 rounded-lg backdrop-blur-sm hover:bg-black/80 transition-colors">
              <Settings className="w-4 h-4" />
            </button>
          </DropdownMenuTrigger>
          <DropdownMenuContent align="end" className="w-56">
            <DropdownMenuCheckboxItem
              checked={showHardwareRobot}
              onCheckedChange={setShowHardwareRobot}
            >
              Show Hardware Robot
            </DropdownMenuCheckboxItem>
            <DropdownMenuCheckboxItem
              checked={showTargetRobot}
              onCheckedChange={setShowTargetRobot}
            >
              Show Commanded Robot
            </DropdownMenuCheckboxItem>
            <DropdownMenuCheckboxItem
              checked={showFirmwareCoordinates}
              onCheckedChange={setShowFirmwareCoordinates}
            >
              Show Firmware Coordinates
            </DropdownMenuCheckboxItem>
            <DropdownMenuCheckboxItem
              checked={showPath}
              onCheckedChange={setShowPath}
            >
              Show Path Preview
            </DropdownMenuCheckboxItem>
          </DropdownMenuContent>
        </DropdownMenu>
      </div>

      {/* Target Preset Buttons - Bottom Left */}
      <div className="absolute bottom-4 left-4 bg-black/70 text-white p-3 rounded-lg text-xs z-10 backdrop-blur-sm">
        <div className="flex items-center justify-between mb-2">
          <div className="font-semibold">Target Presets</div>
          <button
            onClick={handleRecordClick}
            className="ml-2 p-1 rounded hover:bg-white/10 transition-colors"
            title="Add current position as preset"
          >
            <Plus className="w-4 h-4" />
          </button>
        </div>
        {savedPositions.length > 0 && (
          <div className="flex flex-col gap-1.5">
            {savedPositions.map((position, index) => (
              <Button
                key={position.name}
                variant="outline"
                size="sm"
                className="h-8 text-xs justify-start hover:bg-white/10"
                onClick={() => handleGoToPosition(position.joints, position.name)}
              >
                <span className="font-mono mr-2 text-gray-400">{index + 1}.</span>
                {position.name}
                <span className="ml-auto text-[9px] text-gray-500">Alt+{index + 1}</span>
              </Button>
            ))}
          </div>
        )}
      </div>

      <Canvas camera={{ position: [0.5, 0.4, 0.8], fov: 50 }} onPointerMissed={handleCanvasClick}>
        <ambientLight intensity={0.5} />
        <directionalLight position={[10, 10, 5]} intensity={1} />
        <Suspense fallback={null}>
          <URDFRobot
            showLabels={showLabels}
            hardwareRobotColor={hardwareRobotColor}
            hardwareRobotTransparency={hardwareRobotTransparency}
            commanderRobotColor={commanderRobotColor}
            commanderRobotTransparency={commanderRobotTransparency}
            hardwareJointAngles={hardwareJointAngles}
            activeToolId={activeToolId}
          />

          {/* Target TCP visualizer (orange/cyan/magenta) - shows commanded position from target robot */}
          {/* NOTE: NO rotation applied - gets world position directly from URDF */}
          {showTargetRobot && <TargetTCPVisualizer />}

          {/* Actual TCP visualizer (yellow/lime/purple) - shows hardware feedback from actual robot */}
          {/* NOTE: NO rotation applied - gets world position directly from URDF */}
          {showHardwareRobot && <ActualTCPVisualizer />}

          {/* Cartesian input gizmo (red/green/blue) - only in cartesian mode */}
          {/* Shows user's cartesian slider input - where they WANT to command the robot */}
          {/* NOTE: NO parent rotation - TargetPoseVisualizer handles coordinate transform internally */}
          {motionMode === 'cartesian' && <TargetPoseVisualizer />}

          {/* Path visualization between timeline keyframes */}
          <PathVisualizer visible={showPath} />
        </Suspense>
        <OrbitControls target={[0, 0.2, 0]} />

        {/* WebGL Context Loss Monitor - detects and logs WebGL crashes */}
        <WebGLContextMonitor />

        {/* Interactive rotating coordinate system gizmo */}
        <GizmoHelper alignment="bottom-right" margin={[80, 80]}>
          <group rotation={[-Math.PI / 2, 0, 0]}>
            <GizmoViewport
              axisColors={['#ff0000', '#00ff00', '#0000ff']}
              labelColor="white"
            />
          </group>
        </GizmoHelper>
      </Canvas>

      {/* Safety confirmation dialog */}
      <SafetyDialog />

      {/* Record Preset Dialog */}
      <Dialog open={recordDialogOpen} onOpenChange={setRecordDialogOpen}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Record Preset Position</DialogTitle>
            <DialogDescription>
              Enter a name for this preset. Current commanded joint angles will be saved.
            </DialogDescription>
          </DialogHeader>
          <div className="py-4">
            <Input
              placeholder="e.g., Pick Position"
              value={presetName}
              onChange={(e) => setPresetName(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter') {
                  handleSavePreset();
                }
              }}
              autoFocus
            />
          </div>
          <DialogFooter>
            <Button variant="outline" onClick={() => setRecordDialogOpen(false)}>
              Cancel
            </Button>
            <Button onClick={handleSavePreset}>
              Save Preset
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </div>
  );
}
