'use client';

import { useState, useEffect, useRef } from 'react';
import Header from '../components/Header';
import {
  getCameraDevices,
  getCameraStatus,
  startCamera,
  stopCamera,
  getCameraStreamUrl,
  CameraDevice,
  CameraStatus
} from '../lib/api';
import { logger } from '../lib/logger';
import { Button } from '@/components/ui/button';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { Alert, AlertDescription } from '@/components/ui/alert';
import { Loader2, Camera, Video, VideoOff, RefreshCw } from 'lucide-react';

export default function CameraPage() {
  const [devices, setDevices] = useState<CameraDevice[]>([]);
  const [selectedDevice, setSelectedDevice] = useState<string>('');
  const [cameraStatus, setCameraStatus] = useState<CameraStatus | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [detectingDevices, setDetectingDevices] = useState(false);
  const imgRef = useRef<HTMLImageElement>(null);

  // Load available devices on mount
  useEffect(() => {
    loadDevices();
    loadStatus();
  }, []);

  // Poll camera status periodically
  useEffect(() => {
    const interval = setInterval(() => {
      loadStatus();
    }, 2000);

    return () => clearInterval(interval);
  }, []);

  const loadDevices = async () => {
    setDetectingDevices(true);
    setError(null);

    const result = await getCameraDevices();

    if (result.success && result.devices) {
      setDevices(result.devices);

      // Auto-select first device if none selected
      if (result.devices.length > 0 && !selectedDevice) {
        setSelectedDevice(result.devices[0].device);
      }
    } else {
      setError(result.error || 'Failed to load camera devices');
    }

    setDetectingDevices(false);
  };

  const loadStatus = async () => {
    const result = await getCameraStatus();

    if (result.success && result.status) {
      setCameraStatus(result.status);

      // Update selected device if camera is streaming
      if (result.status.streaming && result.status.device) {
        setSelectedDevice(result.status.device);
      }
    }
  };

  const handleStartCamera = async () => {
    if (!selectedDevice) {
      setError('Please select a camera device');
      return;
    }

    setLoading(true);
    setError(null);

    const result = await startCamera(selectedDevice);

    if (result.success) {
      setCameraStatus(result.status || null);
    } else {
      setError(result.error || 'Failed to start camera');
    }

    setLoading(false);
  };

  const handleStopCamera = async () => {
    setLoading(true);
    setError(null);

    const result = await stopCamera();

    if (result.success) {
      await loadStatus();
    } else {
      setError(result.error || 'Failed to stop camera');
    }

    setLoading(false);
  };

  const isStreaming = cameraStatus?.streaming || false;
  const streamUrl = getCameraStreamUrl();

  return (
    <main className="h-screen flex flex-col bg-background">
      <Header />

      {/* Main Content */}
      <div className="flex-1 flex flex-col min-h-0 p-4 gap-4">
        <div className="flex gap-4 h-full">
          {/* Camera View - Main Area */}
          <div className="flex-1 min-w-0">
            <Card className="h-full flex flex-col">
              <CardHeader>
                <div className="flex items-center justify-between">
                  <div className="flex items-center gap-2">
                    <Camera className="w-5 h-5" />
                    <CardTitle>Camera View</CardTitle>
                  </div>
                  {isStreaming && (
                    <Badge variant="default" className="gap-1">
                      <Video className="w-3 h-3" />
                      Streaming
                    </Badge>
                  )}
                </div>
                <CardDescription>
                  USB camera stream for PAROL6 robot workspace
                </CardDescription>
              </CardHeader>
              <CardContent className="flex-1 flex items-center justify-center min-h-0">
                {isStreaming ? (
                  <div className="w-full h-full flex items-center justify-center bg-black rounded-lg overflow-hidden">
                    <img
                      ref={imgRef}
                      src={streamUrl}
                      alt="Camera stream"
                      className="max-w-full max-h-full object-contain"
                      onError={() => {
                        logger.error('Camera stream error', 'CameraPage');
                      }}
                    />
                  </div>
                ) : (
                  <div className="flex flex-col items-center justify-center gap-4 text-muted-foreground">
                    <VideoOff className="w-16 h-16" />
                    <p>Camera not streaming</p>
                    <p className="text-sm">Select a device and click Start to begin</p>
                  </div>
                )}
              </CardContent>
            </Card>
          </div>

          {/* Control Panel - Right Side */}
          <div className="w-[320px] flex-shrink-0 flex flex-col gap-4">
            {/* Camera Controls */}
            <Card>
              <CardHeader>
                <CardTitle>Camera Controls</CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                {/* Device Selector */}
                <div className="space-y-2">
                  <label className="text-sm font-medium">Camera Device</label>
                  <div className="flex gap-2">
                    <Select
                      value={selectedDevice}
                      onValueChange={setSelectedDevice}
                      disabled={loading || isStreaming}
                    >
                      <SelectTrigger>
                        <SelectValue placeholder="Select camera..." />
                      </SelectTrigger>
                      <SelectContent>
                        {devices.map((device) => (
                          <SelectItem key={device.device} value={device.device}>
                            {device.name}
                          </SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                    <Button
                      size="icon"
                      variant="outline"
                      onClick={loadDevices}
                      disabled={detectingDevices || isStreaming}
                    >
                      <RefreshCw className={`w-4 h-4 ${detectingDevices ? 'animate-spin' : ''}`} />
                    </Button>
                  </div>
                  {devices.length === 0 && !detectingDevices && (
                    <p className="text-xs text-muted-foreground">
                      No cameras detected. Connect a USB camera and click refresh.
                    </p>
                  )}
                </div>

                {/* Start/Stop Controls */}
                <div className="flex gap-2">
                  {!isStreaming ? (
                    <Button
                      onClick={handleStartCamera}
                      disabled={loading || !selectedDevice}
                      className="flex-1"
                    >
                      {loading ? (
                        <>
                          <Loader2 className="w-4 h-4 mr-2 animate-spin" />
                          Starting...
                        </>
                      ) : (
                        <>
                          <Video className="w-4 h-4 mr-2" />
                          Start Camera
                        </>
                      )}
                    </Button>
                  ) : (
                    <Button
                      onClick={handleStopCamera}
                      disabled={loading}
                      variant="destructive"
                      className="flex-1"
                    >
                      {loading ? (
                        <>
                          <Loader2 className="w-4 h-4 mr-2 animate-spin" />
                          Stopping...
                        </>
                      ) : (
                        <>
                          <VideoOff className="w-4 h-4 mr-2" />
                          Stop Camera
                        </>
                      )}
                    </Button>
                  )}
                </div>

                {/* Error Display */}
                {error && (
                  <Alert variant="destructive">
                    <AlertDescription>{error}</AlertDescription>
                  </Alert>
                )}
              </CardContent>
            </Card>

            {/* Camera Status */}
            <Card>
              <CardHeader>
                <CardTitle>Status</CardTitle>
              </CardHeader>
              <CardContent className="space-y-3">
                <div className="flex justify-between text-sm">
                  <span className="text-muted-foreground">State:</span>
                  <span className="font-medium">
                    {isStreaming ? 'Streaming' : 'Stopped'}
                  </span>
                </div>

                {cameraStatus?.device && (
                  <div className="flex justify-between text-sm">
                    <span className="text-muted-foreground">Device:</span>
                    <span className="font-mono text-xs">
                      {cameraStatus.device}
                    </span>
                  </div>
                )}

                {cameraStatus?.width && cameraStatus?.height && (
                  <div className="flex justify-between text-sm">
                    <span className="text-muted-foreground">Resolution:</span>
                    <span className="font-medium">
                      {cameraStatus.width} × {cameraStatus.height}
                    </span>
                  </div>
                )}

                {cameraStatus?.fps && (
                  <div className="flex justify-between text-sm">
                    <span className="text-muted-foreground">FPS:</span>
                    <span className="font-medium">{cameraStatus.fps}</span>
                  </div>
                )}

                {!isStreaming && (
                  <p className="text-xs text-muted-foreground">
                    No active camera stream
                  </p>
                )}
              </CardContent>
            </Card>

            {/* Info Card */}
            <Card>
              <CardHeader>
                <CardTitle>About</CardTitle>
              </CardHeader>
              <CardContent className="text-sm text-muted-foreground space-y-2">
                <p>
                  This page allows you to view a live camera feed from USB cameras
                  connected to the Raspberry Pi.
                </p>
                <p>
                  The camera stream uses MJPEG format for low latency and broad
                  compatibility.
                </p>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </main>
  );
}
