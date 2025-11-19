'use client';

import { useMemo, useRef, useEffect, useState } from 'react';
import { Instances, Instance } from '@react-three/drei';
import * as THREE from 'three';
import PathOrientationGizmo from './PathOrientationGizmo';
import { useTimelineStore } from '../lib/stores/timelineStore';
import { useKinematicsStore } from '../lib/stores/kinematicsStore';
import { getJointAnglesAtTime } from '../lib/interpolation';
import { calculateTcpPoseFromUrdf } from '../lib/tcpCalculations';
import { applyJointAnglesToUrdf } from '../lib/urdfHelpers';
import { robotToThreeJs } from '../lib/coordinateTransform';
import type { Keyframe, JointAngles } from '../types';
import { logger } from '../lib/logger';

// Distance-based sampling configuration
// Path shape is independent of timing - samples based on geometric distance
const SAMPLES_PER_10MM = 1; // 1 sample per 10mm of linear distance
const MIN_SAMPLES = 5;       // Minimum samples for very short movements
const MAX_SAMPLES = 100;     // Maximum samples to prevent performance issues

/**
 * Calculate number of samples based on Cartesian distance.
 * This makes path shape independent of timing - only depends on geometry.
 */
function calculateSamplesFromDistance(distanceMm: number): number {
  const samples = Math.ceil(distanceMm / 10); // 1 sample per 10mm
  return Math.max(MIN_SAMPLES, Math.min(MAX_SAMPLES, samples));
}

interface PathVisualizerProps {
  visible?: boolean;
}

interface PathPoint {
  position: THREE.Vector3;
  orientation: THREE.Euler; // RX, RY, RZ orientation in radians
  valid: boolean; // For IK validation (cartesian segments only)
}

interface PathSegment {
  points: PathPoint[];
  motionType: 'joint' | 'cartesian';
  startKeyframe: Keyframe;
  endKeyframe: Keyframe;
}

export default function PathVisualizer({ visible = true }: PathVisualizerProps) {
  const groupRef = useRef<THREE.Group>(null);

  // Hover state for orientation gizmo
  const [hoveredSegmentIndex, setHoveredSegmentIndex] = useState<number | null>(null);
  const [hoveredPointIndex, setHoveredPointIndex] = useState<number | null>(null);

  // Subscribe to stores
  const keyframes = useTimelineStore((state) => state.timeline.keyframes);
  const trajectoryCache = useTimelineStore((state) => state.trajectoryCache);
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);

  // Generate path segments from keyframes
  const pathSegments = useMemo(() => {
    if (!computationRobotRef || keyframes.length < 2) {
      return [];
    }

    const segments: PathSegment[] = [];

    // Iterate through adjacent keyframe pairs
    for (let i = 0; i < keyframes.length - 1; i++) {
      const startKeyframe = keyframes[i];
      const endKeyframe = keyframes[i + 1];

      // Motion type is defined by the END keyframe
      const motionType = endKeyframe.motionType || 'joint';

      const points: PathPoint[] = [];

      if (motionType === 'joint') {
        // Joint motion: Calculate TCP distance to determine sampling density
        // This makes path shape independent of timing

        // Calculate start TCP position using centralized helper
        applyJointAnglesToUrdf(computationRobotRef, startKeyframe.jointAngles);
        const startTcpPose = calculateTcpPoseFromUrdf(computationRobotRef, computationTool.tcp_offset);

        // Calculate end TCP position using centralized helper
        applyJointAnglesToUrdf(computationRobotRef, endKeyframe.jointAngles);
        const endTcpPose = calculateTcpPoseFromUrdf(computationRobotRef, computationTool.tcp_offset);

        // Calculate Cartesian distance (in mm)
        let distance = 0;
        if (startTcpPose && endTcpPose) {
          const dx = endTcpPose.X - startTcpPose.X;
          const dy = endTcpPose.Y - startTcpPose.Y;
          const dz = endTcpPose.Z - startTcpPose.Z;
          distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        // Calculate number of samples based on distance (not time!)
        const numSamples = calculateSamplesFromDistance(distance);

        // Duration still needed for interpolation (but not for sample count!)
        const duration = endKeyframe.time - startKeyframe.time;

        for (let j = 0; j <= numSamples; j++) {
          const t = startKeyframe.time + (j / numSamples) * duration;
          const jointAngles = getJointAnglesAtTime(keyframes, t);

          if (jointAngles) {
            // Apply joint angles to computation robot using centralized helper
            applyJointAnglesToUrdf(computationRobotRef, jointAngles);

            // Calculate TCP pose (returns Three.js coordinates in mm)
            const tcpPose = calculateTcpPoseFromUrdf(computationRobotRef, computationTool.tcp_offset);

            if (tcpPose) {
              // tcpPose is in mm, convert to meters for Three.js scene
              points.push({
                position: new THREE.Vector3(
                  tcpPose.X / 1000,
                  tcpPose.Y / 1000,
                  tcpPose.Z / 1000
                ),
                orientation: new THREE.Euler(
                  tcpPose.RX * Math.PI / 180,
                  tcpPose.RY * Math.PI / 180,
                  tcpPose.RZ * Math.PI / 180,
                  'XYZ'
                ),
                valid: true
              });
            }
          }
        }
      } else {
        // Cartesian motion: Use cached trajectory if available
        if (!startKeyframe.cartesianPose || !endKeyframe.cartesianPose) {
          // Can't generate cartesian path without poses
          continue;
        }

        try {
          // Check cache first
          const cacheKey = `${startKeyframe.id}_${endKeyframe.id}`;
          const cachedTrajectory = useTimelineStore.getState().getCachedTrajectory(cacheKey);

          if (!cachedTrajectory) {
            logger.warn(`No cached trajectory for segment ${cacheKey} - skipping visualization`, 'PathViz');
            continue;
          }

          // Use cached trajectory - sample every 20th waypoint (0.2s at 100Hz)
          const SAMPLE_INTERVAL = 20;
          const waypoints = cachedTrajectory.waypointPoses;
          const ikValid = cachedTrajectory.ikValid;

          // Debug: Log validity distribution
          const validCount = ikValid.filter(v => v).length;
          const invalidCount = ikValid.filter(v => !v).length;
          logger.debug(`Segment ${cacheKey}: ${validCount} valid, ${invalidCount} invalid out of ${ikValid.length} waypoints`, 'PathViz');

          for (let j = 0; j < waypoints.length; j += SAMPLE_INTERVAL) {
            const waypointPose = waypoints[j];

            // Check validity of this specific waypoint (not surrounding waypoints)
            const isValid = ikValid[j];

            // Waypoint poses are in robot coordinates - convert to Three.js coordinates
            const threeJsPose = robotToThreeJs(waypointPose);

            points.push({
              position: new THREE.Vector3(
                threeJsPose.X / 1000,  // Convert mm to meters
                threeJsPose.Y / 1000,
                threeJsPose.Z / 1000
              ),
              orientation: new THREE.Euler(
                threeJsPose.RX * Math.PI / 180,
                threeJsPose.RY * Math.PI / 180,
                threeJsPose.RZ * Math.PI / 180,
                'XYZ'
              ),
              valid: isValid  // Red if any surrounding waypoint failed IK
            });
          }
        } catch (error) {
          logger.error('Failed to generate cartesian waypoints', 'PathViz', error);
          continue;
        }
      }

      if (points.length > 0) {
        segments.push({
          points,
          motionType,
          startKeyframe,
          endKeyframe
        });
      }
    }

    return segments;
  }, [keyframes, trajectoryCache, computationRobotRef, computationTool]);

  // Clear old geometries when segments change
  useEffect(() => {
    return () => {
      if (groupRef.current) {
        groupRef.current.children.forEach((child) => {
          if (child instanceof THREE.Line || child instanceof THREE.Points) {
            child.geometry.dispose();
            if (Array.isArray(child.material)) {
              child.material.forEach(m => m.dispose());
            } else {
              child.material.dispose();
            }
          }
        });
      }
    };
  }, [pathSegments]);

  if (!visible || pathSegments.length === 0) {
    return null;
  }

  // Get hovered point for orientation gizmo
  const hoveredPoint = hoveredSegmentIndex !== null && hoveredPointIndex !== null
    ? pathSegments[hoveredSegmentIndex]?.points[hoveredPointIndex]
    : null;

  return (
    <group ref={groupRef}>
      {/* Orientation gizmo on hovered waypoint */}
      {hoveredPoint && (
        <PathOrientationGizmo
          position={hoveredPoint.position}
          orientation={hoveredPoint.orientation}
        />
      )}

      {/* Path segments (lines and waypoint markers) */}
      {pathSegments.map((segment, segmentIdx) => {
        const isJoint = segment.motionType === 'joint';
        const baseColor = isJoint ? 0xff9800 : 0x00bcd4; // Orange for joint, cyan for cartesian

        // For cartesian segments, split into valid/invalid sub-segments
        if (!isJoint) {
          const validSegments: PathPoint[][] = [];
          const invalidSegments: PathPoint[][] = [];
          let currentValid: PathPoint[] = [];
          let currentInvalid: PathPoint[] = [];

          segment.points.forEach((point, i) => {
            if (point.valid) {
              if (currentInvalid.length > 0) {
                // Add connecting point to invalid segment
                currentInvalid.push(point);
                invalidSegments.push(currentInvalid);
                currentInvalid = [];
              }
              currentValid.push(point);
            } else {
              if (currentValid.length > 0) {
                // Add connecting point to valid segment
                if (i > 0) {
                  currentValid.push(point);
                }
                validSegments.push(currentValid);
                currentValid = [];
              }
              currentInvalid.push(point);
            }
          });

          if (currentValid.length > 0) validSegments.push(currentValid);
          if (currentInvalid.length > 0) invalidSegments.push(currentInvalid);

          return (
            <group key={segmentIdx}>
              {/* Valid cartesian segments (cyan) */}
              {validSegments.map((points, subIdx) => {
                if (points.length < 2) return null;

                const positions = new Float32Array(points.length * 3);
                points.forEach((p, i) => {
                  positions[i * 3] = p.position.x;
                  positions[i * 3 + 1] = p.position.y;
                  positions[i * 3 + 2] = p.position.z;
                });

                // Calculate starting point index for this sub-segment
                const startPointIdx = segment.points.indexOf(points[0]);

                return (
                  <group key={`valid-${subIdx}`}>
                    {/* Path line */}
                    <line>
                      <bufferGeometry>
                        <bufferAttribute
                          attach="attributes-position"
                          count={points.length}
                          array={positions}
                          itemSize={3}
                        />
                      </bufferGeometry>
                      <lineBasicMaterial color={baseColor} transparent opacity={0.75} linewidth={2} />
                    </line>

                    {/* Waypoint markers as hoverable spheres */}
                    <Instances limit={points.length}>
                      <sphereGeometry args={[0.0012, 8, 8]} />
                      <meshBasicMaterial color={baseColor} />
                      {points.map((point, idx) => (
                        <Instance
                          key={idx}
                          position={point.position}
                          onPointerEnter={() => {
                            setHoveredSegmentIndex(segmentIdx);
                            setHoveredPointIndex(startPointIdx + idx);
                          }}
                          onPointerLeave={() => {
                            setHoveredSegmentIndex(null);
                            setHoveredPointIndex(null);
                          }}
                        />
                      ))}
                    </Instances>
                  </group>
                );
              })}

              {/* Invalid cartesian segments (red) */}
              {invalidSegments.map((points, subIdx) => {
                if (points.length < 2) return null;

                const positions = new Float32Array(points.length * 3);
                points.forEach((p, i) => {
                  positions[i * 3] = p.position.x;
                  positions[i * 3 + 1] = p.position.y;
                  positions[i * 3 + 2] = p.position.z;
                });

                // Calculate starting point index for this sub-segment
                const startPointIdx = segment.points.indexOf(points[0]);

                return (
                  <group key={`invalid-${subIdx}`}>
                    {/* Path line */}
                    <line>
                      <bufferGeometry>
                        <bufferAttribute
                          attach="attributes-position"
                          count={points.length}
                          array={positions}
                          itemSize={3}
                        />
                      </bufferGeometry>
                      <lineBasicMaterial color={0xf44336} transparent opacity={0.75} linewidth={2} />
                    </line>

                    {/* Waypoint markers as hoverable spheres */}
                    <Instances limit={points.length}>
                      <sphereGeometry args={[0.0012, 8, 8]} />
                      <meshBasicMaterial color={0xf44336} />
                      {points.map((point, idx) => (
                        <Instance
                          key={idx}
                          position={point.position}
                          onPointerEnter={() => {
                            setHoveredSegmentIndex(segmentIdx);
                            setHoveredPointIndex(startPointIdx + idx);
                          }}
                          onPointerLeave={() => {
                            setHoveredSegmentIndex(null);
                            setHoveredPointIndex(null);
                          }}
                        />
                      ))}
                    </Instances>
                  </group>
                );
              })}
            </group>
          );
        }

        // Joint segment (simple, all valid)
        const positions = new Float32Array(segment.points.length * 3);
        segment.points.forEach((p, i) => {
          positions[i * 3] = p.position.x;
          positions[i * 3 + 1] = p.position.y;
          positions[i * 3 + 2] = p.position.z;
        });

        return (
          <group key={segmentIdx}>
            {/* Path line */}
            <line>
              <bufferGeometry>
                <bufferAttribute
                  attach="attributes-position"
                  count={segment.points.length}
                  array={positions}
                  itemSize={3}
                />
              </bufferGeometry>
              <lineBasicMaterial color={baseColor} transparent opacity={0.75} linewidth={2} />
            </line>

            {/* Waypoint markers as hoverable spheres */}
            <Instances limit={segment.points.length}>
              <sphereGeometry args={[0.0012, 8, 8]} />
              <meshBasicMaterial color={baseColor} />
              {segment.points.map((point, idx) => (
                <Instance
                  key={idx}
                  position={point.position}
                  onPointerEnter={() => {
                    setHoveredSegmentIndex(segmentIdx);
                    setHoveredPointIndex(idx);
                  }}
                  onPointerLeave={() => {
                    setHoveredSegmentIndex(null);
                    setHoveredPointIndex(null);
                  }}
                />
              ))}
            </Instances>
          </group>
        );
      })}
    </group>
  );
}
