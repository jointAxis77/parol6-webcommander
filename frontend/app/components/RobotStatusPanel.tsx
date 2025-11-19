'use client';

import { Card } from '@/components/ui/card';
import { useHardwareStore } from '../lib/stores';
import { useEffect, useState } from 'react';

export default function RobotStatusPanel() {
  // Get real hardware feedback from hardware store
  const connectionStatus = useHardwareStore((state) => state.connectionStatus);
  const hardwareCartesianPose = useHardwareStore((state) => state.hardwareCartesianPose);
  const ioStatus = useHardwareStore((state) => state.ioStatus);
  const gripperStatus = useHardwareStore((state) => state.gripperStatus);
  const robotStatus = useHardwareStore((state) => state.robotStatus);

  const [lastUpdate, setLastUpdate] = useState<string>('N/A');

  // Update timestamp when any data changes
  useEffect(() => {
    if (hardwareCartesianPose || ioStatus || gripperStatus) {
      const now = new Date();
      setLastUpdate(now.toLocaleTimeString());
    }
  }, [hardwareCartesianPose, ioStatus, gripperStatus]);

  return (
    <Card className="p-3 h-full flex flex-col overflow-hidden">
      <h2 className="text-sm font-semibold mb-2">Robot Status</h2>

      <div className="space-y-2 text-xs overflow-y-auto flex-1">
        {/* Connection Status */}
        <div>
          <h3 className="font-semibold mb-1 text-xs">Connection</h3>
          <div className="space-y-0.5 pl-3">
            <div className="flex items-center gap-2">
              <span className={connectionStatus === 'connected' ? 'text-green-500' : 'text-gray-500'}>
                ●
              </span>
              <span className="text-muted-foreground">Commander:</span>
              <span className="font-medium">
                {connectionStatus === 'connected' ?
                  `CONNECTED - ${robotStatus?.commander_hz?.toFixed(0) ?? '--'}Hz` :
                  connectionStatus.toUpperCase()}
              </span>
            </div>
            <div className="flex items-center gap-2">
              <span className={robotStatus?.is_stopped === false ? 'text-green-500' : 'text-gray-500'}>
                {robotStatus?.is_stopped == null ? '○' : '●'}
              </span>
              <span className="text-muted-foreground">Robot:</span>
              <span className="font-medium">
                {robotStatus?.is_stopped == null ? 'N/A' :
                 robotStatus?.is_stopped === false ? 'RUNNING' : 'STOPPED'}
              </span>
            </div>
            <div className="flex items-center gap-2">
              <span className={robotStatus?.estop_active === false ? 'text-blue-500' :
                             robotStatus?.estop_active === true ? 'text-red-500' : 'text-gray-500'}>
                {robotStatus?.estop_active == null ? '-' :
                 robotStatus?.estop_active === false ? '✓' : '✗'}
              </span>
              <span className="text-muted-foreground">E-STOP:</span>
              <span className="font-medium">
                {robotStatus?.estop_active == null ? 'N/A' :
                 robotStatus?.estop_active === false ? 'OK' : 'ACTIVE'}
              </span>
            </div>
          </div>
        </div>

        {/* I/O Status */}
        <div>
          <h3 className="font-semibold mb-1 text-xs">I/O Status</h3>
          <div className="space-y-0.5 pl-3">
            <div className="grid grid-cols-2 gap-4">
              <div className="flex items-center gap-2">
                <span className={ioStatus?.input_1 ? 'text-green-500' : 'text-gray-400'}>
                  {ioStatus?.input_1 ? '●' : '○'}
                </span>
                <span className="text-muted-foreground">IN1:</span>
                <span className="font-mono">({!ioStatus ? 'N/A' : ioStatus.input_1 ? '1' : '0'})</span>
              </div>
              <div className="flex items-center gap-2">
                <span className={ioStatus?.output_1 ? 'text-green-500' : 'text-gray-400'}>
                  {ioStatus?.output_1 ? '●' : '○'}
                </span>
                <span className="text-muted-foreground">OUT1:</span>
                <span className="font-mono">({!ioStatus ? 'N/A' : ioStatus.output_1 ? '1' : '0'})</span>
              </div>
              <div className="flex items-center gap-2">
                <span className={ioStatus?.input_2 ? 'text-green-500' : 'text-gray-400'}>
                  {ioStatus?.input_2 ? '●' : '○'}
                </span>
                <span className="text-muted-foreground">IN2:</span>
                <span className="font-mono">({!ioStatus ? 'N/A' : ioStatus.input_2 ? '1' : '0'})</span>
              </div>
              <div className="flex items-center gap-2">
                <span className={ioStatus?.output_2 ? 'text-green-500' : 'text-gray-400'}>
                  {ioStatus?.output_2 ? '●' : '○'}
                </span>
                <span className="text-muted-foreground">OUT2:</span>
                <span className="font-mono">({!ioStatus ? 'N/A' : ioStatus.output_2 ? '1' : '0'})</span>
              </div>
            </div>
            <div className="flex items-center gap-2 mt-2">
              <span className={!ioStatus ? 'text-gray-500' :
                             !ioStatus.estop_pressed ? 'text-blue-500' : 'text-red-500'}>
                {!ioStatus ? '-' :
                 !ioStatus.estop_pressed ? '✓' : '✗'}
              </span>
              <span className="text-muted-foreground">E-STOP:</span>
              <span className="font-medium">
                {!ioStatus ? 'N/A' :
                 !ioStatus.estop_pressed ? 'OK' : 'PRESSED'}
              </span>
            </div>
          </div>
        </div>

        {/* Gripper Status */}
        <div>
          <h3 className="font-semibold mb-1 text-xs">Gripper</h3>
          <div className="flex items-center gap-3 pl-3 text-xs font-mono">
            <div>
              <span className="text-muted-foreground">ID:</span>{' '}
              <span>{gripperStatus?.device_id ?? 'N/A'}</span>
            </div>
            <div>
              <span className="text-muted-foreground">Pos:</span>{' '}
              <span>{gripperStatus?.position ?? 'N/A'}</span>
            </div>
            <div>
              <span className="text-muted-foreground">Obj:</span>{' '}
              <span>
                {gripperStatus ?
                  (gripperStatus.object_detected === 0 ? 'none' :
                   gripperStatus.object_detected === 1 ? 'closing' : 'opening')
                  : 'N/A'}
              </span>
            </div>
          </div>
        </div>

        {/* Update Indicator */}
        <div className="pt-1 border-t">
          <div className="text-xs text-muted-foreground">
            Last Update: <span className="font-mono">{lastUpdate}</span>
          </div>
        </div>
      </div>
    </Card>
  );
}
