'use client';

import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Switch } from '@/components/ui/switch';
import { Label } from '@/components/ui/label';
import { Tooltip, TooltipContent, TooltipProvider, TooltipTrigger } from '@/components/ui/tooltip';
import { useState } from 'react';
import { useCommandStore, useHardwareStore, useInputStore, usePerformanceStore, useRobotConfigStore } from '../lib/stores';
import { useKinematicsStore } from '../lib/stores/kinematicsStore';
import { AlertTriangle, StopCircle, Target, Copy, Circle } from 'lucide-react';
import { getApiBaseUrl } from '../lib/apiConfig';
import { executeTrajectory } from '../lib/api';
import { JointAngles, JointName } from '../lib/types';
import { generateCartesianWaypoints, calculateWaypointCount } from '../lib/cartesianPlanner';
import { inverseKinematicsDetailed } from '../lib/kinematics';
import { logger } from '../lib/logger';

export default function ControlOptions() {
  const [isMoving, setIsMoving] = useState(false);
  const [isHoming, setIsHoming] = useState(false);
  const [isClearing, setIsClearing] = useState(false);
  const [isStopping, setIsStopping] = useState(false);
  const [isMovingCartesianBackend, setIsMovingCartesianBackend] = useState(false);

  // Command store: Commanded state and movement parameters
  const speed = useCommandStore((state) => state.speed);
  const commandedJoints = useCommandStore((state) => state.commandedJointAngles);
  const setCommandedJointAngle = useCommandStore((state) => state.setCommandedJointAngle);

  // Hardware store: Robot status and hardware angles
  const robotStatus = useHardwareStore((state) => state.robotStatus);
  const hardwareJointAngles = useHardwareStore((state) => state.hardwareJointAngles);
  const hardwareTcpPose = useHardwareStore((state) => state.hardwareTcpPose);

  // Input store: Input angle setters and target cartesian pose
  const setInputJointAngle = useInputStore((state) => state.setInputJointAngle);

  // Command store: TCP pose
  const commandedTcpPose = useCommandStore((state) => state.commandedTcpPose);

  // Kinematics store: Computation robot and tool
  const computationRobotRef = useKinematicsStore((state) => state.computationRobotRef);
  const computationTool = useKinematicsStore((state) => state.computationTool);

  // Robot config store: TCP offset and IK axis mask
  const tcpOffset = useRobotConfigStore((state) => state.tcpOffset);
  const ikAxisMask = useRobotConfigStore((state) => state.ikAxisMask);

  // Performance store: Recording state
  const isRecording = usePerformanceStore((state) => state.isRecording);
  const startRecording = usePerformanceStore((state) => state.startRecording);
  const stopRecording = usePerformanceStore((state) => state.stopRecording);

  const handleHome = async () => {
    setIsHoming(true);
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/robot/home`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const error = await response.json();
        logger.error('Failed to home robot', 'ControlOptions', error);
        alert(`Failed to home robot: ${error.detail || 'Unknown error'}`);
      }
    } catch (error) {
      logger.error('Error homing robot', 'ControlOptions', error);
      alert('Failed to communicate with robot');
    } finally {
      setIsHoming(false);
    }
  };

  const handleCopyPoseFromHardware = () => {
    if (!hardwareJointAngles) return;

    // Copy all 6 joint angles from hardware to commanded and input
    const joints: JointName[] = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];
    joints.forEach(joint => {
      setInputJointAngle(joint, hardwareJointAngles[joint]);
      setCommandedJointAngle(joint, hardwareJointAngles[joint]);
    });
  };

  const handleClearEstop = async () => {
    setIsClearing(true);
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/robot/clear-estop`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const error = await response.json();
        logger.error('Failed to clear E-stop', 'ControlOptions', error);
        alert(`Failed to clear E-stop: ${error.detail || 'Unknown error'}`);
      }
    } catch (error) {
      logger.error('Error clearing E-stop', 'ControlOptions', error);
      alert('Failed to communicate with robot');
    } finally{
      setIsClearing(false);
    }
  };

  const handleStop = async () => {
    setIsStopping(true);
    try {
      const response = await fetch(`${getApiBaseUrl()}/api/robot/stop`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const error = await response.json();
        logger.error('Failed to stop robot', 'ControlOptions', error);
        alert(`Failed to stop robot: ${error.detail || 'Unknown error'}`);
      } else {
        // Reset moving states when stop is successful
        setIsMoving(false);
        setIsHoming(false);
      }
    } catch (error) {
      logger.error('Error stopping robot', 'ControlOptions', error);
      alert('Failed to communicate with robot');
    } finally {
      setIsStopping(false);
    }
  };


  const handleMoveCartesianBackend = async () => {
    setIsMovingCartesianBackend(true);
    try {
      // ========================================================================
      // 100Hz Cartesian Motion System (Frontend IK)
      // ========================================================================
      // 1. Get current pose (from hardware store - already computed by URDF)
      // 2. Get target pose (from command store - already computed by URDF)
      // 3. Generate waypoints for straight-line motion
      // 4. Solve IK for each waypoint (frontend numericalIK)
      // 5. Execute trajectory at 100Hz
      // ========================================================================

      if (!hardwareJointAngles || !hardwareTcpPose || !commandedTcpPose) {
        alert('Robot pose not available - wait for URDF to load');
        return;
      }

      if (!computationRobotRef) {
        alert('URDF robot model not loaded yet');
        return;
      }

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

        // Solve IK using frontend numerical solver with computation robot
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
    } finally {
      setIsMovingCartesianBackend(false);
    }
  };


  const handleRecordingToggle = async (checked: boolean) => {
    if (checked) {
      await startRecording();
    } else {
      await stopRecording();
    }
  };

  return (
    <Card className="p-3">
      <h2 className="text-sm font-semibold mb-3">Actions</h2>

      {/* Action Buttons */}
      <div className="space-y-1.5">
        {/* E-Stop Clear Button - Show prominently when E-stop is active */}
        {robotStatus?.estop_active && (
          <Button
            variant="destructive"
            size="sm"
            className="h-8 text-xs w-full font-semibold"
            onClick={handleClearEstop}
            disabled={isClearing}
          >
            <AlertTriangle className="h-3.5 w-3.5 mr-1.5" />
            {isClearing ? 'Clearing...' : 'Clear E-Stop'}
          </Button>
        )}

        {/* Stop Button - Show prominently when robot is moving */}
        {isMoving && !isHoming && (
          <Button
            variant="destructive"
            size="sm"
            className="h-8 text-xs w-full font-semibold bg-red-600 hover:bg-red-700"
            onClick={handleStop}
            disabled={isStopping}
          >
            <StopCircle className="h-3.5 w-3.5 mr-1.5" />
            {isStopping ? 'Stopping...' : 'STOP Robot'}
          </Button>
        )}

        <Button
          variant="outline"
          size="sm"
          className="h-8 text-xs w-full"
          onClick={handleHome}
          disabled={isHoming || isMoving || hardwareJointAngles === null}
        >
          {isHoming ? 'Homing...' : 'Home hardware robot'}
        </Button>

        <TooltipProvider>
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                onClick={handleCopyPoseFromHardware}
                disabled={hardwareJointAngles === null}
                size="sm"
                variant="outline"
                className="h-8 text-xs w-full"
              >
                <Copy className="h-3.5 w-3.5 mr-1.5" />
                Copy Pose
              </Button>
            </TooltipTrigger>
            <TooltipContent>
              <p>Copy hardware robot position to commanded robot</p>
            </TooltipContent>
          </Tooltip>
        </TooltipProvider>

        <Button
          variant="outline"
          size="sm"
          className="h-8 text-xs w-full"
          onClick={handleMoveCartesianBackend}
          disabled={isMovingCartesianBackend || isMoving || hardwareJointAngles === null}
        >
          <Target className="h-3.5 w-3.5 mr-1.5" />
          {isMovingCartesianBackend ? 'Moving...' : 'Execute Cartesian Motion'}
        </Button>

        {/* Performance Recording Toggle */}
        <div className="flex items-center justify-between pt-2 border-t">
          <div className="flex items-center gap-2">
            <Circle className={`h-3 w-3 ${isRecording ? 'fill-red-500 text-red-500 animate-pulse' : 'text-gray-400'}`} />
            <Label htmlFor="recording-toggle" className="text-xs font-medium cursor-pointer">
              {isRecording ? 'Recording...' : 'Record Performance'}
            </Label>
          </div>
          <Switch
            id="recording-toggle"
            checked={isRecording}
            onCheckedChange={handleRecordingToggle}
          />
        </div>
      </div>
    </Card>
  );
}
