'use client';

import { useRef, useState, useEffect, Suspense, useMemo } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, GizmoHelper, GizmoViewport } from '@react-three/drei';
import Header from '../components/Header';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Switch } from '@/components/ui/switch';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Upload, Trash2, FileIcon, Plus } from 'lucide-react';
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three-stdlib';
import * as THREE from 'three';
import { useRobotConfigStore } from '../lib/stores/robotConfigStore';
import { applyJointAnglesToUrdf } from '../lib/urdfHelpers';
import GripperTCPVisualizer from '../components/GripperTCPVisualizer';
import ToolCard from '../components/ToolCard';
import ToolMountDialog from '../components/ToolMountDialog';
import ToolDeleteDialog from '../components/ToolDeleteDialog';
import { getApiBaseUrl } from '../lib/apiConfig';
import { logger } from '../lib/logger';

// Tool Interface
interface Tool {
  id: string;
  name: string;
  description: string;
  mesh_file: string | null;
  mesh_units?: 'mm' | 'm'; // Units of the mesh file (default: mm)
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
}

// Static URDF Robot Component (no controls, fixed pose, no L6)
function StaticURDFRobot({ onRobotLoad }: { onRobotLoad: (robot: any) => void }) {
  const [localRobot, setLocalRobot] = useState<any>(null);
  const commanderRobotColor = useRobotConfigStore((state) => state.commanderRobotColor);

  useEffect(() => {
    const loader = new URDFLoader();

    loader.load(
      '/urdf/PAROL6.urdf',
      (loadedRobot: any) => {
        setLocalRobot(loadedRobot);
        onRobotLoad(loadedRobot);

        setTimeout(() => {
          // Remove L6 link (gripper)
          loadedRobot.traverse((child: any) => {
            if (child.isURDFLink && child.name === 'L6') {
              if (child.parent) {
                child.parent.remove(child);
              }
            }
          });

          // Set fixed pose: J1=0, J2=-90, J3=180, J4=0, J5=0, J6=90
          const fixedAngles = { J1: 0, J2: -90, J3: 180, J4: 0, J5: 0, J6: 90 };

          // Apply joint angles using centralized helper
          applyJointAnglesToUrdf(loadedRobot, fixedAngles);
          loadedRobot.updateMatrixWorld(true);

          // Style robot with gray color
          loadedRobot.traverse((child: any) => {
            if (child.isMesh && child.material) {
              const materials = Array.isArray(child.material) ? child.material : [child.material];
              const newMaterials: any[] = [];

              materials.forEach((mat: any) => {
                if (mat) {
                  const clonedMat = mat.clone();
                  clonedMat.color.set(commanderRobotColor); // Use color from settings
                  clonedMat.metalness = 0.2;      // Low metalness for diffuse look
                  clonedMat.roughness = 0.7;      // Higher roughness for matte finish
                  clonedMat.transparent = false;
                  clonedMat.opacity = 1.0;
                  clonedMat.needsUpdate = true;
                  newMaterials.push(clonedMat);
                }
              });

              child.material = Array.isArray(child.material) ? newMaterials : newMaterials[0];
            }
          });
        }, 500);
      }
    );

    return () => {
      if (localRobot) {
        localRobot.traverse((child: any) => {
          if (child.isMesh) {
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
              const materials = Array.isArray(child.material) ? child.material : [child.material];
              materials.forEach((mat: any) => mat.dispose && mat.dispose());
            }
          }
        });
      }
    };
  }, []);

  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      {localRobot && <primitive object={localRobot} />}
    </group>
  );
}

// STL Mesh Component - Positioned at J6 joint (L6 attachment point)
function STLMesh({
  stlGeometry,
  robotRef,
  offset,
  stlGeometryOpen,
  stlGeometryClosed,
  displayState
}: {
  stlGeometry: THREE.BufferGeometry | null;
  robotRef: any;
  offset: { x: number; y: number; z: number; rx: number; ry: number; rz: number };
  stlGeometryOpen?: THREE.BufferGeometry | null;
  stlGeometryClosed?: THREE.BufferGeometry | null;
  displayState?: 'open' | 'closed';
}) {
  const meshRef = useRef<THREE.Mesh>(null);
  const meshOpenRef = useRef<THREE.Mesh>(null);
  const meshClosedRef = useRef<THREE.Mesh>(null);


  // Update STL position/rotation to match J6 joint every frame
  useFrame(() => {
    if (!robotRef) return;

    const j6Joint = robotRef.joints?.['L6'];
    if (!j6Joint) return;

    j6Joint.updateMatrixWorld(true);

    // Get L6 world transform
    const l6WorldPosition = new THREE.Vector3();
    const l6WorldQuaternion = new THREE.Quaternion();
    j6Joint.getWorldPosition(l6WorldPosition);
    j6Joint.getWorldQuaternion(l6WorldQuaternion);

    // Helper function to update a mesh's transform
    const updateMeshTransform = (meshRef: React.RefObject<THREE.Mesh>) => {
      if (!meshRef.current) return;

      // Apply position offset in L6's local frame (rotates with J6)
      if (offset.x !== 0 || offset.y !== 0 || offset.z !== 0) {
        const localOffset = new THREE.Vector3(offset.x, offset.y, offset.z);
        const worldOffset = localOffset.applyQuaternion(l6WorldQuaternion);
        meshRef.current.position.copy(l6WorldPosition).add(worldOffset);
      } else {
        meshRef.current.position.copy(l6WorldPosition);
      }

      // Set base rotation from L6
      meshRef.current.quaternion.copy(l6WorldQuaternion);

      // Apply L6 visual origin rotation from URDF: rpy="0 0 -1.5708" (-90° Z)
      const visualOriginRotation = new THREE.Quaternion();
      visualOriginRotation.setFromEuler(new THREE.Euler(0, 0, -Math.PI / 2, 'XYZ'));
      meshRef.current.quaternion.multiply(visualOriginRotation);

      // Apply user rotation offset (degrees -> radians -> quaternion)
      if (offset.rx !== 0 || offset.ry !== 0 || offset.rz !== 0) {
        const userRotationEuler = new THREE.Euler(
          offset.rx * Math.PI / 180,
          offset.ry * Math.PI / 180,
          offset.rz * Math.PI / 180,
          'XYZ'
        );
        const userRotationQuat = new THREE.Quaternion();
        userRotationQuat.setFromEuler(userRotationEuler);
        meshRef.current.quaternion.multiply(userRotationQuat);
      }
    };

    // Update all active meshes
    updateMeshTransform(meshRef);
    updateMeshTransform(meshOpenRef);
    updateMeshTransform(meshClosedRef);
  });

  // Determine which geometries to show
  const hasGripperGeometries = stlGeometryOpen || stlGeometryClosed;
  const showMain = !hasGripperGeometries && stlGeometry;
  const showOpen = hasGripperGeometries && displayState === 'open';
  const showClosed = hasGripperGeometries && displayState === 'closed';

  return (
    <>
      {/* Main single STL (backward compatible) */}
      {showMain && stlGeometry && (
        <mesh ref={meshRef} geometry={stlGeometry}>
          <meshStandardMaterial
            color="#2196f3"
            emissive="#2196f3"
            emissiveIntensity={0.3}
            metalness={0.3}
            roughness={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {/* Open state STL */}
      {stlGeometryOpen && (
        <mesh ref={meshOpenRef} geometry={stlGeometryOpen} visible={showOpen}>
          <meshStandardMaterial
            color="#2196f3"
            emissive="#2196f3"
            emissiveIntensity={0.3}
            metalness={0.3}
            roughness={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {/* Closed state STL */}
      {stlGeometryClosed && (
        <mesh ref={meshClosedRef} geometry={stlGeometryClosed} visible={showClosed}>
          <meshStandardMaterial
            color="#2196f3"
            emissive="#2196f3"
            emissiveIntensity={0.3}
            metalness={0.3}
            roughness={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}
    </>
  );
}

export default function ConfigurationPage() {
  // Tool management state
  const [tools, setTools] = useState<Tool[]>([]);
  const [selectedToolId, setSelectedToolId] = useState<string | null>(null);
  const [activeToolId, setActiveToolId] = useState<string>('default_j6_tip');
  const [isCreatingNew, setIsCreatingNew] = useState(false);

  // Editor state (current form values)
  const [editedToolName, setEditedToolName] = useState('');
  const [editedToolDescription, setEditedToolDescription] = useState('');
  const [uploadedFile, setUploadedFile] = useState<File | null>(null);
  const [fileInfo, setFileInfo] = useState<{ name: string; size: string } | null>(null);
  const [stlGeometry, setStlGeometry] = useState<THREE.BufferGeometry | null>(null);
  const [stlUnits, setStlUnits] = useState<'mm' | 'm'>('mm'); // Default to mm
  const [meshOffset, setMeshOffset] = useState({ x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 }); // Manual offset adjustment (position + rotation)
  const [tcpOffset, setTcpOffset] = useState({ x: 0, y: -45.2, z: -62.8, rx: 90, ry: 180, rz: 0 }); // TCP offset (position in mm, rotation in degrees)

  // Gripper I/O configuration state
  const [gripperEnabled, setGripperEnabled] = useState(false);
  const [gripperIoPin, setGripperIoPin] = useState<1 | 2>(1);
  const [gripperOpenIsHigh, setGripperOpenIsHigh] = useState(true);
  const [stlFileOpen, setStlFileOpen] = useState<File | null>(null);
  const [stlFileClosed, setStlFileClosed] = useState<File | null>(null);
  const [stlGeometryOpen, setStlGeometryOpen] = useState<THREE.BufferGeometry | null>(null);
  const [stlGeometryClosed, setStlGeometryClosed] = useState<THREE.BufferGeometry | null>(null);
  const [displayState, setDisplayState] = useState<'open' | 'closed'>('open');

  // UI state
  const [robot, setRobot] = useState<any>(null); // URDF robot reference
  const [isDragging, setIsDragging] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [isLoadingTools, setIsLoadingTools] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isSaving, setIsSaving] = useState(false);
  const [saveSuccess, setSaveSuccess] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);

  // Dialog state
  const [mountDialogOpen, setMountDialogOpen] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [toolToMount, setToolToMount] = useState<Tool | null>(null);
  const [toolToDelete, setToolToDelete] = useState<Tool | null>(null);

  // API Integration Functions
  // Load tools from backend
  const loadTools = async () => {
    setIsLoadingTools(true);
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/config/tools`);
      if (!response.ok) throw new Error('Failed to load tools');
      const data = await response.json();
      setTools(data.tools || []);
      setActiveToolId(data.active_tool_id || 'default_j6_tip');
    } catch (err) {
      logger.error('Error loading tools', 'ConfigurationPage', { error: err });
      setError(err instanceof Error ? err.message : 'Failed to load tools');
    } finally {
      setIsLoadingTools(false);
    }
  };

  // Load tool into editor
  const loadToolIntoEditor = async (toolId: string) => {
    const tool = tools.find(t => t.id === toolId);
    if (tool) {
      setSelectedToolId(toolId);
      setEditedToolName(tool.name);
      setEditedToolDescription(tool.description);
      setMeshOffset(tool.mesh_offset);
      setTcpOffset(tool.tcp_offset);
      setIsCreatingNew(false);

      // Load gripper config if present
      if (tool.gripper_config) {
        setGripperEnabled(tool.gripper_config.enabled);
        setGripperIoPin(tool.gripper_config.io_pin);
        setGripperOpenIsHigh(tool.gripper_config.open_is_high);

        // Load open state STL if present
        if (tool.gripper_config.mesh_file_open) {
          try {
            const url = `/urdf/meshes/${tool.gripper_config.mesh_file_open}`;
            const response = await fetch(url);
            if (response.ok) {
              const arrayBuffer = await response.arrayBuffer();
              const loader = new STLLoader();
              const geometry = loader.parse(arrayBuffer);
              const units = tool.mesh_units || 'mm';
              if (units === 'mm') {
                geometry.scale(0.001, 0.001, 0.001);
              }
              setStlGeometryOpen(geometry);
              setStlFileOpen(new File([arrayBuffer], tool.gripper_config.mesh_file_open));
            }
          } catch (err) {
            logger.error('Error loading open state mesh', 'loadToolIntoEditor', { error: err });
          }
        }

        // Load closed state STL if present
        if (tool.gripper_config.mesh_file_closed) {
          try {
            const url = `/urdf/meshes/${tool.gripper_config.mesh_file_closed}`;
            const response = await fetch(url);
            if (response.ok) {
              const arrayBuffer = await response.arrayBuffer();
              const loader = new STLLoader();
              const geometry = loader.parse(arrayBuffer);
              const units = tool.mesh_units || 'mm';
              if (units === 'mm') {
                geometry.scale(0.001, 0.001, 0.001);
              }
              setStlGeometryClosed(geometry);
              setStlFileClosed(new File([arrayBuffer], tool.gripper_config.mesh_file_closed));
            }
          } catch (err) {
            logger.error('Error loading closed state mesh', 'loadToolIntoEditor', { error: err });
          }
        }
      } else {
        // Clear gripper config
        setGripperEnabled(false);
        setStlGeometryOpen(null);
        setStlGeometryClosed(null);
        setStlFileOpen(null);
        setStlFileClosed(null);
      }

      // Load STL file if tool has one
      if (tool.mesh_file) {
        try {
          setIsLoading(true);
          const url = `/urdf/meshes/${tool.mesh_file}`;
          const response = await fetch(url);
          if (response.ok) {
            const arrayBuffer = await response.arrayBuffer();
            const loader = new STLLoader();
            const geometry = loader.parse(arrayBuffer);

            // Apply unit scaling only if mesh is in mm (convert to meters for Three.js)
            const units = tool.mesh_units || 'mm'; // Default to mm for backward compatibility
            if (units === 'mm') {
              geometry.scale(0.001, 0.001, 0.001);
            }

            setStlGeometry(geometry);
            setFileInfo({
              name: tool.mesh_file,
              size: `${(arrayBuffer.byteLength / 1024).toFixed(2)} KB`
            });
            setStlUnits(units);
          } else {
            logger.error(`Fetch failed with status: ${response.status}`, 'loadToolIntoEditor', { toolId, meshFile: tool.mesh_file });
            setError(`Failed to load mesh: ${tool.mesh_file} (HTTP ${response.status})`);
          }
        } catch (err) {
          logger.error('Error loading tool mesh', 'loadToolIntoEditor', { error: err, toolId, meshFile: tool.mesh_file });
          setError(`Failed to load mesh: ${tool.mesh_file}`);
        } finally {
          setIsLoading(false);
        }
      } else {
        // No mesh file - clear geometry
        setUploadedFile(null);
        setFileInfo(null);
        setStlGeometry(null);
      }
    }
  };

  // Create new tool
  const handleCreateNew = () => {
    setSelectedToolId(null);
    setIsCreatingNew(true);
    setEditedToolName('');
    setEditedToolDescription('');
    setMeshOffset({ x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 });
    setTcpOffset({ x: 0, y: 0, z: -62.8, rx: 0, ry: 0, rz: 0 });
    setUploadedFile(null);
    setFileInfo(null);
    setStlGeometry(null);

    // Clear gripper state
    setGripperEnabled(false);
    setGripperIoPin(1);
    setGripperOpenIsHigh(true);
    setStlFileOpen(null);
    setStlFileClosed(null);
    setStlGeometryOpen(null);
    setStlGeometryClosed(null);
    setDisplayState('open');
  };

  // Save tool (create or update)
  const handleSaveTool = async () => {
    if (!editedToolName.trim()) {
      setSaveError('Tool name is required');
      return;
    }

    setIsSaving(true);
    setSaveSuccess(false);
    setSaveError(null);

    try {
      // Convert main STL to base64 if present
      let base64Data = null;
      if (uploadedFile) {
        const arrayBuffer = await uploadedFile.arrayBuffer();
        base64Data = btoa(
          new Uint8Array(arrayBuffer).reduce((data, byte) => data + String.fromCharCode(byte), '')
        );
      }

      // Convert open state STL to base64 if present
      let base64DataOpen = null;
      if (stlFileOpen) {
        const arrayBuffer = await stlFileOpen.arrayBuffer();
        base64DataOpen = btoa(
          new Uint8Array(arrayBuffer).reduce((data, byte) => data + String.fromCharCode(byte), '')
        );
      }

      // Convert closed state STL to base64 if present
      let base64DataClosed = null;
      if (stlFileClosed) {
        const arrayBuffer = await stlFileClosed.arrayBuffer();
        base64DataClosed = btoa(
          new Uint8Array(arrayBuffer).reduce((data, byte) => data + String.fromCharCode(byte), '')
        );
      }

      const payload: any = {
        name: editedToolName,
        description: editedToolDescription,
        mesh_file: uploadedFile ? uploadedFile.name : null,
        mesh_units: stlUnits, // 'mm' or 'm'
        mesh_offset_position: { x: meshOffset.x, y: meshOffset.y, z: meshOffset.z },
        mesh_offset_rotation: { rx: meshOffset.rx, ry: meshOffset.ry, rz: meshOffset.rz },
        tcp_offset_position: { x: tcpOffset.x, y: tcpOffset.y, z: tcpOffset.z },
        tcp_offset_rotation: { rx: tcpOffset.rx, ry: tcpOffset.ry, rz: tcpOffset.rz },
        stl_data: base64Data,
      };

      // Add gripper config if enabled
      if (gripperEnabled) {
        payload.gripper_config = {
          enabled: true,
          io_pin: gripperIoPin,
          open_is_high: gripperOpenIsHigh,
          mesh_file_open: stlFileOpen ? stlFileOpen.name : null,
          mesh_file_closed: stlFileClosed ? stlFileClosed.name : null,
        };
        payload.stl_data_open = base64DataOpen;
        payload.stl_data_closed = base64DataClosed;
      } else {
        payload.gripper_config = null;
      }

      let response;
      if (isCreatingNew) {
        // Create new tool
        response = await fetch(`${getApiBaseUrl()}/api/config/tools`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        });
      } else {
        // Update existing tool
        response = await fetch(`${getApiBaseUrl()}/api/config/tools/${selectedToolId}`, {
          method: 'PATCH',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(payload),
        });
      }

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to save tool');
      }

      setSaveSuccess(true);
      await loadTools(); // Refresh tools list
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (err) {
      logger.error('Error saving tool', 'saveTool', { error: err });
      setSaveError(err instanceof Error ? err.message : 'Failed to save tool');
    } finally {
      setIsSaving(false);
    }
  };

  // Delete tool
  const handleDeleteTool = (tool: Tool) => {
    setToolToDelete(tool);
    setDeleteDialogOpen(true);
  };

  const confirmDeleteTool = async () => {
    if (!toolToDelete) return;

    try {
      const response = await fetch(`${getApiBaseUrl()}/api/config/tools/${toolToDelete.id}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to delete tool');
      }

      await loadTools(); // Refresh tools list

      // Clear editor if deleted tool was selected
      if (selectedToolId === toolToDelete.id) {
        setSelectedToolId(null);
        setIsCreatingNew(false);
      }
    } catch (err) {
      logger.error('Error deleting tool', 'confirmDeleteTool', { error: err });
      setError(err instanceof Error ? err.message : 'Failed to delete tool');
    } finally {
      setDeleteDialogOpen(false);
      setToolToDelete(null);
    }
  };

  // Mount tool
  const handleMountTool = (tool: Tool) => {
    setToolToMount(tool);
    setMountDialogOpen(true);
  };

  const confirmMountTool = async () => {
    if (!toolToMount) return;

    try {
      const response = await fetch(`${getApiBaseUrl()}/api/config/tools/${toolToMount.id}/mount`, {
        method: 'POST',
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to mount tool');
      }

      await loadTools(); // Refresh tools list
    } catch (err) {
      logger.error('Error mounting tool', 'confirmMountTool', { error: err });
      setError(err instanceof Error ? err.message : 'Failed to mount tool');
    } finally {
      setMountDialogOpen(false);
      setToolToMount(null);
    }
  };

  // Compute unsaved changes
  const hasUnsavedChanges = useMemo(() => {
    if (isCreatingNew) {
      return editedToolName.trim().length > 0 || editedToolDescription.trim().length > 0 || uploadedFile !== null;
    }

    if (!selectedToolId) return false;

    const savedTool = tools.find(t => t.id === selectedToolId);
    if (!savedTool) return false;

    return (
      editedToolName !== savedTool.name ||
      editedToolDescription !== savedTool.description ||
      JSON.stringify(meshOffset) !== JSON.stringify(savedTool.mesh_offset) ||
      JSON.stringify(tcpOffset) !== JSON.stringify(savedTool.tcp_offset)
    );
  }, [isCreatingNew, selectedToolId, tools, editedToolName, editedToolDescription, meshOffset, tcpOffset, uploadedFile]);

  // Load tools on mount
  useEffect(() => {
    loadTools();
  }, []);

  // Warn before leaving with unsaved changes
  useEffect(() => {
    const handleBeforeUnload = (e: BeforeUnloadEvent) => {
      if (hasUnsavedChanges) {
        e.preventDefault();
        e.returnValue = 'You have unsaved changes. Are you sure you want to leave?';
      }
    };

    window.addEventListener('beforeunload', handleBeforeUnload);
    return () => window.removeEventListener('beforeunload', handleBeforeUnload);
  }, [hasUnsavedChanges]);

  // Load STL file
  const loadSTL = async (file: File) => {
    setIsLoading(true);
    setError(null);

    try {
      // Validate file type
      if (!file.name.toLowerCase().endsWith('.stl')) {
        throw new Error('Please upload a valid STL file');
      }

      // Read file as ArrayBuffer
      const arrayBuffer = await file.arrayBuffer();

      // Load STL
      const loader = new STLLoader();
      const geometry = loader.parse(arrayBuffer);

      // Scale based on selected units (no normalization, just unit conversion)
      // Note: We preserve the STL's original origin point instead of centering
      const scale = stlUnits === 'mm' ? 0.001 : 1.0; // mm->m or keep as-is
      geometry.scale(scale, scale, scale);

      setStlGeometry(geometry);

      // Update file info
      const sizeKB = (file.size / 1024).toFixed(2);
      setFileInfo({
        name: file.name,
        size: sizeKB + ' KB',
      });

      setUploadedFile(file);
      setIsLoading(false);
    } catch (err) {
      logger.error('Error loading STL', 'handleFileDrop', { error: err });
      setError(err instanceof Error ? err.message : 'Failed to load STL file');
      setIsLoading(false);
    }
  };

  // Handle file input change
  const handleFileChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      loadSTL(file);
    }
  };

  // Handle drag and drop
  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    setIsDragging(true);
  };

  const handleDragLeave = (e: React.DragEvent) => {
    e.preventDefault();
    setIsDragging(false);
  };

  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    setIsDragging(false);

    const file = e.dataTransfer.files?.[0];
    if (file) {
      loadSTL(file);
    }
  };

  // Clear uploaded mesh
  const handleClear = () => {
    if (stlGeometry) {
      stlGeometry.dispose();
    }
    setStlGeometry(null);
    setUploadedFile(null);
    setFileInfo(null);
    setError(null);
  };

  // Load open state STL for gripper
  const loadSTLOpen = async (file: File) => {
    setIsLoading(true);
    setError(null);

    try {
      if (!file.name.toLowerCase().endsWith('.stl')) {
        throw new Error('Please upload a valid STL file');
      }

      const arrayBuffer = await file.arrayBuffer();
      const loader = new STLLoader();
      const geometry = loader.parse(arrayBuffer);

      const scale = stlUnits === 'mm' ? 0.001 : 1.0;
      geometry.scale(scale, scale, scale);

      setStlGeometryOpen(geometry);
      setStlFileOpen(file);
      setIsLoading(false);
    } catch (err) {
      logger.error('Error loading open STL', 'loadSTLOpen', { error: err });
      setError(err instanceof Error ? err.message : 'Failed to load STL file');
      setIsLoading(false);
    }
  };

  // Load closed state STL for gripper
  const loadSTLClosed = async (file: File) => {
    setIsLoading(true);
    setError(null);

    try {
      if (!file.name.toLowerCase().endsWith('.stl')) {
        throw new Error('Please upload a valid STL file');
      }

      const arrayBuffer = await file.arrayBuffer();
      const loader = new STLLoader();
      const geometry = loader.parse(arrayBuffer);

      const scale = stlUnits === 'mm' ? 0.001 : 1.0;
      geometry.scale(scale, scale, scale);

      setStlGeometryClosed(geometry);
      setStlFileClosed(file);
      setIsLoading(false);
    } catch (err) {
      logger.error('Error loading closed STL', 'loadSTLClosed', { error: err });
      setError(err instanceof Error ? err.message : 'Failed to load STL file');
      setIsLoading(false);
    }
  };

  // Handle file input change for open state
  const handleFileChangeOpen = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      loadSTLOpen(file);
    }
  };

  // Handle file input change for closed state
  const handleFileChangeClosed = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      loadSTLClosed(file);
    }
  };

  // Clear open state STL
  const handleClearOpen = () => {
    if (stlGeometryOpen) {
      stlGeometryOpen.dispose();
    }
    setStlGeometryOpen(null);
    setStlFileOpen(null);
  };

  // Clear closed state STL
  const handleClearClosed = () => {
    if (stlGeometryClosed) {
      stlGeometryClosed.dispose();
    }
    setStlGeometryClosed(null);
    setStlFileClosed(null);
  };

  // Save configuration to URDF
  const handleSaveToURDF = async () => {
    if (!uploadedFile) {
      setSaveError('No STL file uploaded');
      return;
    }

    setIsSaving(true);
    setSaveSuccess(false);
    setSaveError(null);

    try {
      // Read file as base64
      const arrayBuffer = await uploadedFile.arrayBuffer();
      const base64Data = btoa(
        new Uint8Array(arrayBuffer).reduce((data, byte) => data + String.fromCharCode(byte), '')
      );

      // Call API endpoint
      const response = await fetch(`${getApiBaseUrl()}/api/urdf/update-gripper`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          mesh_filename: uploadedFile.name,
          mesh_offset_position: {
            x: meshOffset.x,
            y: meshOffset.y,
            z: meshOffset.z,
          },
          mesh_offset_rotation: {
            rx: meshOffset.rx,
            ry: meshOffset.ry,
            rz: meshOffset.rz,
          },
          tcp_offset_position: {
            x: tcpOffset.x,
            y: tcpOffset.y,
            z: tcpOffset.z,
          },
          tcp_offset_rotation: {
            rx: tcpOffset.rx,
            ry: tcpOffset.ry,
            rz: tcpOffset.rz,
          },
          stl_data: base64Data,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to save configuration');
      }

      const result = await response.json();

      setSaveSuccess(true);
      setIsSaving(false);

      // Clear success message after 3 seconds
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (err) {
      logger.error('Error saving to URDF', 'saveToURDF', { error: err });
      setSaveError(err instanceof Error ? err.message : 'Failed to save configuration');
      setIsSaving(false);
    }
  };

  // Re-scale STL when units change
  useEffect(() => {
    if (stlGeometry && uploadedFile) {
      loadSTL(uploadedFile);
    }
  }, [stlUnits]);

  // Get current mounted tool
  const currentMountedTool = tools.find(t => t.id === activeToolId) || null;

  return (
    <main className="h-screen flex flex-col bg-background">
      <Header />

      <div className="flex-1 p-4 grid grid-cols-[400px_1fr] gap-4 overflow-hidden">
        {/* Left Panel - Tool List + Editor */}
        <div className="space-y-3 overflow-auto">
          {/* Tool List */}
          <Card className="p-3">
            <div className="flex items-center justify-between mb-2">
              <h2 className="text-base font-semibold">Tools</h2>
              <Button size="sm" onClick={handleCreateNew} className="h-7">
                <Plus className="w-3.5 h-3.5 mr-1" />
                New
              </Button>
            </div>

            {isLoadingTools && (
              <div className="text-xs text-muted-foreground text-center py-3">
                Loading...
              </div>
            )}

            {!isLoadingTools && tools.length === 0 && (
              <div className="text-xs text-muted-foreground text-center py-3">
                No tools found
              </div>
            )}

            <div className="space-y-1.5">
              {tools.map((tool) => (
                <ToolCard
                  key={tool.id}
                  tool={tool}
                  isMounted={tool.id === activeToolId}
                  isSelected={tool.id === selectedToolId}
                  onSelect={() => loadToolIntoEditor(tool.id)}
                  onDelete={() => handleDeleteTool(tool)}
                />
              ))}
            </div>
          </Card>

          {/* Tool Editor */}
          {(selectedToolId || isCreatingNew) && (
            <Card className="p-3">
              <h2 className="text-base font-semibold mb-2">
                {isCreatingNew ? 'New Tool' : 'Edit Tool'}
              </h2>

              {/* Tool Metadata */}
              <div className="space-y-2 mb-3">
                <div>
                  <label className="text-xs font-medium mb-1 block">Name</label>
                  <input
                    type="text"
                    value={editedToolName}
                    onChange={(e) => setEditedToolName(e.target.value)}
                    placeholder="Tool name"
                    className="w-full h-8 px-2 text-sm bg-background border rounded"
                  />
                </div>

                <div>
                  <label className="text-xs font-medium mb-1 block">Description (optional)</label>
                  <input
                    type="text"
                    value={editedToolDescription}
                    onChange={(e) => setEditedToolDescription(e.target.value)}
                    placeholder="Brief description"
                    className="w-full h-8 px-2 text-sm bg-background border rounded"
                  />
                </div>
              </div>

            {/* Upload Area - Always visible when creating/editing */}
            {/* Conditional STL Upload: Single or Dual based on gripper */}
            {!gripperEnabled && (selectedToolId || isCreatingNew) && (
              <div
                className={`
                  border-2 border-dashed rounded-lg p-4 text-center cursor-pointer
                  transition-colors
                  ${isDragging ? 'border-primary bg-primary/10' : 'border-border hover:border-primary/50'}
                `}
                onDragOver={handleDragOver}
                onDragLeave={handleDragLeave}
                onDrop={handleDrop}
                onClick={() => document.getElementById('file-input')?.click()}
              >
                <Upload className="w-8 h-8 mx-auto mb-2 text-muted-foreground" />
                <p className="text-xs font-medium mb-0.5">
                  {isDragging ? 'Drop STL file here' : 'Upload STL Mesh'}
                </p>
                <p className="text-[10px] text-muted-foreground">
                  Click or drag & drop
                </p>
                <input
                  id="file-input"
                  type="file"
                  accept=".stl"
                  className="hidden"
                  onChange={handleFileChange}
                />
              </div>
            )}

            {/* Dual STL Upload for Gripper */}
            {gripperEnabled && (selectedToolId || isCreatingNew) && (
              <div className="space-y-2">
                <label className="text-xs font-medium block">Gripper State STLs</label>
                <div className="grid grid-cols-2 gap-2">
                  {/* Open State Upload */}
                  <div>
                    <label className="text-[10px] text-muted-foreground block mb-1">Open State</label>
                    {!stlFileOpen ? (
                      <div
                        className="border-2 border-dashed rounded p-2 text-center cursor-pointer transition-colors border-border hover:border-primary/50"
                        onClick={() => document.getElementById('file-input-open')?.click()}
                      >
                        <Upload className="w-5 h-5 mx-auto mb-1 text-muted-foreground" />
                        <p className="text-[9px] text-muted-foreground">Upload STL</p>
                        <input
                          id="file-input-open"
                          type="file"
                          accept=".stl"
                          className="hidden"
                          onChange={handleFileChangeOpen}
                        />
                      </div>
                    ) : (
                      <div className="p-2 bg-card border rounded flex items-center justify-between gap-1">
                        <FileIcon className="w-3 h-3 text-primary shrink-0" />
                        <p className="text-[9px] font-medium truncate flex-1">{stlFileOpen.name}</p>
                        <Button
                          variant="ghost"
                          size="icon"
                          onClick={handleClearOpen}
                          className="h-5 w-5 shrink-0"
                        >
                          <Trash2 className="w-2.5 h-2.5" />
                        </Button>
                      </div>
                    )}
                  </div>

                  {/* Closed State Upload */}
                  <div>
                    <label className="text-[10px] text-muted-foreground block mb-1">Closed State</label>
                    {!stlFileClosed ? (
                      <div
                        className="border-2 border-dashed rounded p-2 text-center cursor-pointer transition-colors border-border hover:border-primary/50"
                        onClick={() => document.getElementById('file-input-closed')?.click()}
                      >
                        <Upload className="w-5 h-5 mx-auto mb-1 text-muted-foreground" />
                        <p className="text-[9px] text-muted-foreground">Upload STL</p>
                        <input
                          id="file-input-closed"
                          type="file"
                          accept=".stl"
                          className="hidden"
                          onChange={handleFileChangeClosed}
                        />
                      </div>
                    ) : (
                      <div className="p-2 bg-card border rounded flex items-center justify-between gap-1">
                        <FileIcon className="w-3 h-3 text-primary shrink-0" />
                        <p className="text-[9px] font-medium truncate flex-1">{stlFileClosed.name}</p>
                        <Button
                          variant="ghost"
                          size="icon"
                          onClick={handleClearClosed}
                          className="h-5 w-5 shrink-0"
                        >
                          <Trash2 className="w-2.5 h-2.5" />
                        </Button>
                      </div>
                    )}
                  </div>
                </div>
              </div>
            )}

            {/* Unit Selector - shown when any STL is uploaded or when creating/editing */}
            {(uploadedFile || stlFileOpen || stlFileClosed || selectedToolId || isCreatingNew) && (
              <div className="mt-2 p-2 bg-card border rounded">
                <label className="text-xs font-medium mb-1.5 block">Mesh Units</label>
                <div className="flex gap-1">
                  <Button
                    variant={stlUnits === 'mm' ? 'default' : 'outline'}
                    size="sm"
                    onClick={() => setStlUnits('mm')}
                    className="flex-1 h-7 text-xs"
                  >
                    mm
                  </Button>
                  <Button
                    variant={stlUnits === 'm' ? 'default' : 'outline'}
                    size="sm"
                    onClick={() => setStlUnits('m')}
                    className="flex-1 h-7 text-xs"
                  >
                    m
                  </Button>
                </div>
              </div>
            )}

            {/* Loading State */}
            {isLoading && (
              <div className="mt-2 p-2 bg-blue-500/10 border border-blue-500/20 rounded text-xs text-blue-400">
                Loading...
              </div>
            )}

            {/* Error State */}
            {error && (
              <div className="mt-2 p-2 bg-red-500/10 border border-red-500/20 rounded text-xs text-red-400">
                {error}
              </div>
            )}

            {/* File Info */}
            {fileInfo && !error && (
              <div className="mt-2 p-2 bg-card border rounded flex items-center justify-between gap-2">
                <div className="flex items-center gap-2 min-w-0">
                  <FileIcon className="w-4 h-4 text-primary shrink-0" />
                  <div className="min-w-0">
                    <p className="text-xs font-medium truncate">{fileInfo.name}</p>
                    <p className="text-[10px] text-muted-foreground">{fileInfo.size}</p>
                  </div>
                </div>
                <Button
                  variant="ghost"
                  size="icon"
                  onClick={handleClear}
                  className="h-7 w-7 shrink-0"
                >
                  <Trash2 className="w-3.5 h-3.5" />
                </Button>
              </div>
            )}

            {/* Mesh Offset Adjustment */}
            {(selectedToolId || isCreatingNew) && (
              <div className="mt-2 p-2 bg-card border rounded">
                <div className="flex items-center justify-between mb-1.5">
                  <label className="text-xs font-medium">Mesh Offset</label>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setMeshOffset({ x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 })}
                    className="h-6 px-2 text-[10px]"
                  >
                    Reset
                  </Button>
                </div>

                {/* Position & Rotation in one section */}
                <div className="space-y-1.5">
                  <div>
                    <label className="text-[10px] text-muted-foreground mb-1 block">Position (m)</label>
                    <div className="grid grid-cols-3 gap-1.5">
                      <input
                        type="number"
                        step="0.001"
                        value={meshOffset.x}
                        onChange={(e) => setMeshOffset({ ...meshOffset, x: parseFloat(e.target.value) || 0 })}
                        placeholder="X"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="0.001"
                        value={meshOffset.y}
                        onChange={(e) => setMeshOffset({ ...meshOffset, y: parseFloat(e.target.value) || 0 })}
                        placeholder="Y"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="0.001"
                        value={meshOffset.z}
                        onChange={(e) => setMeshOffset({ ...meshOffset, z: parseFloat(e.target.value) || 0 })}
                        placeholder="Z"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                    </div>
                  </div>

                  <div>
                    <label className="text-[10px] text-muted-foreground mb-1 block">Rotation (°)</label>
                    <div className="grid grid-cols-3 gap-1.5">
                      <input
                        type="number"
                        step="1"
                        value={meshOffset.rx}
                        onChange={(e) => setMeshOffset({ ...meshOffset, rx: parseFloat(e.target.value) || 0 })}
                        placeholder="RX"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="1"
                        value={meshOffset.ry}
                        onChange={(e) => setMeshOffset({ ...meshOffset, ry: parseFloat(e.target.value) || 0 })}
                        placeholder="RY"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="1"
                        value={meshOffset.rz}
                        onChange={(e) => setMeshOffset({ ...meshOffset, rz: parseFloat(e.target.value) || 0 })}
                        placeholder="RZ"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                    </div>
                  </div>
                </div>
              </div>
            )}

            {/* TCP Offset Configuration */}
            {(selectedToolId || isCreatingNew) && (
              <div className="mt-2 p-2 bg-card border rounded">
                <div className="flex items-center justify-between mb-1.5">
                  <label className="text-xs font-medium">TCP Offset</label>
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={() => setTcpOffset({ x: 0, y: -45.2, z: -62.8, rx: 90, ry: 180, rz: 0 })}
                    className="h-6 px-2 text-[10px]"
                  >
                    Reset
                  </Button>
                </div>

                <div className="space-y-1.5">
                  <div>
                    <label className="text-[10px] text-muted-foreground mb-1 block">Position (mm)</label>
                    <div className="grid grid-cols-3 gap-1.5">
                      <input
                        type="number"
                        step="0.1"
                        value={tcpOffset.x}
                        onChange={(e) => setTcpOffset({ ...tcpOffset, x: parseFloat(e.target.value) || 0 })}
                        placeholder="X"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="0.1"
                        value={tcpOffset.y}
                        onChange={(e) => setTcpOffset({ ...tcpOffset, y: parseFloat(e.target.value) || 0 })}
                        placeholder="Y"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="0.1"
                        value={tcpOffset.z}
                        onChange={(e) => setTcpOffset({ ...tcpOffset, z: parseFloat(e.target.value) || 0 })}
                        placeholder="Z"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                    </div>
                  </div>

                  <div>
                    <label className="text-[10px] text-muted-foreground mb-1 block">Orientation (°)</label>
                    <div className="grid grid-cols-3 gap-1.5">
                      <input
                        type="number"
                        step="1"
                        value={tcpOffset.rx}
                        onChange={(e) => setTcpOffset({ ...tcpOffset, rx: parseFloat(e.target.value) || 0 })}
                        placeholder="RX"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="1"
                        value={tcpOffset.ry}
                        onChange={(e) => setTcpOffset({ ...tcpOffset, ry: parseFloat(e.target.value) || 0 })}
                        placeholder="RY"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                      <input
                        type="number"
                        step="1"
                        value={tcpOffset.rz}
                        onChange={(e) => setTcpOffset({ ...tcpOffset, rz: parseFloat(e.target.value) || 0 })}
                        placeholder="RZ"
                        className="h-7 px-1.5 text-xs bg-background border rounded text-center"
                      />
                    </div>
                  </div>
                </div>
              </div>
            )}

            {/* Gripper I/O Configuration */}
            {(selectedToolId || isCreatingNew) && (
              <div className="mt-2 p-2 bg-card border rounded">
                <div className="flex items-center justify-between mb-1.5">
                  <label className="text-xs font-medium">Gripper I/O</label>
                  <Switch
                    checked={gripperEnabled}
                    onCheckedChange={setGripperEnabled}
                    className="scale-75"
                  />
                </div>

                {gripperEnabled && (
                  <div className="space-y-1.5 pt-1.5">
                    <div>
                      <label className="text-[10px] text-muted-foreground mb-1 block">I/O Pin</label>
                      <Select value={gripperIoPin.toString()} onValueChange={(val) => setGripperIoPin(parseInt(val) as 1 | 2)}>
                        <SelectTrigger className="h-7 text-xs">
                          <SelectValue placeholder="Select pin" />
                        </SelectTrigger>
                        <SelectContent>
                          <SelectItem value="1">Output 1</SelectItem>
                          <SelectItem value="2">Output 2</SelectItem>
                        </SelectContent>
                      </Select>
                    </div>

                    <div className="flex items-center justify-between py-1">
                      <span className="text-[10px] text-muted-foreground">
                        {gripperOpenIsHigh ? 'Open = I/O HIGH' : 'Open = I/O LOW'}
                      </span>
                      <Switch
                        checked={gripperOpenIsHigh}
                        onCheckedChange={setGripperOpenIsHigh}
                        className="scale-75"
                      />
                    </div>

                    <div className="text-[9px] text-muted-foreground/60 text-center">
                      Close = I/O {gripperOpenIsHigh ? 'LOW' : 'HIGH'}
                    </div>
                  </div>
                )}
              </div>
            )}

              {/* Tool Action Buttons */}
              <div className="mt-3 space-y-1.5">
                {/* Success/Error Messages */}
                {saveSuccess && (
                  <div className="p-2 bg-green-500/10 border border-green-500/20 rounded text-xs text-green-400">
                    Saved!
                  </div>
                )}
                {saveError && (
                  <div className="p-2 bg-red-500/10 border border-red-500/20 rounded text-xs text-red-400">
                    {saveError}
                  </div>
                )}

                {/* Save Button */}
                <Button
                  onClick={handleSaveTool}
                  disabled={isSaving || !editedToolName.trim()}
                  className="w-full h-8"
                  size="sm"
                >
                  {hasUnsavedChanges && <span className="mr-1">●</span>}
                  {isSaving ? 'Saving...' : isCreatingNew ? 'Create' : 'Save'}
                </Button>

                {/* Mount Button (only for existing tools) */}
                {selectedToolId && !isCreatingNew && (
                  <Button
                    variant="secondary"
                    onClick={() => handleMountTool(tools.find(t => t.id === selectedToolId)!)}
                    disabled={selectedToolId === activeToolId}
                    className="w-full h-8"
                    size="sm"
                  >
                    {selectedToolId === activeToolId ? 'Mounted' : 'Mount'}
                  </Button>
                )}
              </div>
            </Card>
          )}

        </div>

        {/* Right Panel - 3D Preview (React Three Fiber Canvas) */}
        <div className="w-full h-full bg-gray-950 rounded-lg overflow-hidden border relative">
          <div className="absolute top-4 left-4 z-10 bg-black/70 px-3 py-1.5 rounded-lg backdrop-blur-sm">
            <h3 className="text-sm font-semibold">Tool Preview</h3>
          </div>

          <Canvas camera={{ position: [0.5, 0.4, 0.8], fov: 50 }}>
            <ambientLight intensity={0.5} />
            <directionalLight position={[10, 10, 5]} intensity={1} />

            <Suspense fallback={null}>
              {/* Static URDF Robot (gray, no L6, fixed pose) */}
              <StaticURDFRobot onRobotLoad={setRobot} />

              {/* Uploaded STL Mesh (blue) - positioned at L6 attachment point */}
              <STLMesh
                stlGeometry={stlGeometry}
                robotRef={robot}
                offset={meshOffset}
                stlGeometryOpen={stlGeometryOpen}
                stlGeometryClosed={stlGeometryClosed}
                displayState={displayState}
              />

              {/* TCP Gizmo (orange/cyan/magenta arrows) - shows functional tool center point */}
              <GripperTCPVisualizer robotRef={robot} tcpOffset={tcpOffset} />
            </Suspense>

            <OrbitControls target={[0, 0.2, 0]} />

            {/* Grid - meter scale */}
            <Grid args={[10, 10]} cellColor="#6b6b6b" sectionColor="#3f3f3f" />

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

          {/* Display State Toggle (independent of gripper I/O) */}
          {(stlGeometryOpen || stlGeometryClosed) && (
            <div className="absolute bottom-4 left-4 z-10 bg-black/70 px-3 py-2 rounded-lg backdrop-blur-sm">
              <div className="flex items-center gap-2">
                <span className="text-[10px] text-muted-foreground">Display State:</span>
                <div className="flex items-center gap-1.5">
                  <Button
                    variant={displayState === 'open' ? 'default' : 'ghost'}
                    size="sm"
                    onClick={() => setDisplayState('open')}
                    className="h-6 px-2 text-[10px]"
                  >
                    Open
                  </Button>
                  <Button
                    variant={displayState === 'closed' ? 'default' : 'ghost'}
                    size="sm"
                    onClick={() => setDisplayState('closed')}
                    className="h-6 px-2 text-[10px]"
                  >
                    Closed
                  </Button>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Dialogs */}
      <ToolMountDialog
        open={mountDialogOpen}
        onOpenChange={setMountDialogOpen}
        currentTool={currentMountedTool}
        newTool={toolToMount}
        onConfirm={confirmMountTool}
      />

      <ToolDeleteDialog
        open={deleteDialogOpen}
        onOpenChange={setDeleteDialogOpen}
        tool={toolToDelete}
        onConfirm={confirmDeleteTool}
      />
    </main>
  );
}
