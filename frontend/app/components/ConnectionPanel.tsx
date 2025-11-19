'use client';

import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { useState } from 'react';

type ConnectionMode = 'robot' | 'simulator';

export default function ConnectionPanel() {
  const [serialPort, setSerialPort] = useState('COM5');
  const [mode, setMode] = useState<ConnectionMode>('robot');
  const [isConnected, setIsConnected] = useState(false);

  // Mock serial ports (in real app, would be detected via API)
  const availablePorts = ['COM3', 'COM4', 'COM5', 'COM6', '/dev/ttyACM0', '/dev/ttyUSB0'];

  const handleConnect = () => {
    setIsConnected(true);
    // In real app: establish WebSocket connection, send START command
  };

  const handleDisconnect = () => {
    setIsConnected(false);
    // In real app: close WebSocket connection
  };

  const handleClearError = () => {
    // In real app: send CLEAR_ERROR command
  };

  const handleEmergencyStop = () => {
    // In real app: send STOP command immediately
    alert('Emergency Stop Activated!');
  };

  return (
    <Card className="p-3 h-full flex flex-col justify-between">
      <div>
        <h2 className="text-sm font-semibold mb-2">Connection</h2>

        <div className="space-y-2">
          {/* Serial Port Selector */}
          <div>
            <label className="text-xs font-medium mb-1 block">Serial Port</label>
            <Select value={serialPort} onValueChange={setSerialPort} disabled={isConnected}>
              <SelectTrigger>
                <SelectValue placeholder="Select port" />
              </SelectTrigger>
              <SelectContent>
                {availablePorts.map((port) => (
                  <SelectItem key={port} value={port}>
                    {port}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
          </div>

          {/* Mode Toggle */}
          <div>
            <label className="text-xs font-medium mb-1 block">Mode</label>
            <div className="flex gap-1.5">
              <Button
                variant={mode === 'robot' ? 'default' : 'outline'}
                size="sm"
                onClick={() => setMode('robot')}
                disabled={isConnected}
                className="flex-1 h-8 text-xs"
              >
                Robot
              </Button>
              <Button
                variant={mode === 'simulator' ? 'default' : 'outline'}
                size="sm"
                onClick={() => setMode('simulator')}
                disabled={isConnected}
                className="flex-1 h-8 text-xs"
              >
                Simulator
              </Button>
            </div>
          </div>
        </div>
      </div>

      {/* Connection Buttons */}
      <div className="space-y-1.5">
        <div className="grid grid-cols-2 gap-1.5">
          <Button
            variant="default"
            onClick={handleConnect}
            disabled={isConnected}
            className="w-full h-8 text-xs"
          >
            Connect
          </Button>
          <Button
            variant="outline"
            onClick={handleDisconnect}
            disabled={!isConnected}
            className="w-full h-8 text-xs"
          >
            Disconnect
          </Button>
        </div>

        <div className="grid grid-cols-2 gap-1.5">
          <Button
            variant="outline"
            onClick={handleClearError}
            disabled={!isConnected}
            className="w-full h-8 text-xs"
          >
            Clear Error
          </Button>
          <Button
            variant="destructive"
            onClick={handleEmergencyStop}
            className="w-full h-8 text-xs font-bold"
          >
            E-Stop
          </Button>
        </div>
      </div>
    </Card>
  );
}
