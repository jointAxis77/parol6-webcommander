'use client';

import { useEffect, useState } from 'react';
import Header from '../components/Header';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select';
import { Checkbox } from '@/components/ui/checkbox';
import { Slider } from '@/components/ui/slider';
import { Save, RotateCcw, AlertCircle, Loader2, Plus, Pencil, Trash2 } from 'lucide-react';
import { useConfigStore, Config } from '../lib/configStore';
import { useCommandStore, useHardwareStore } from '../lib/stores';
import { getApiBaseUrl } from '../lib/apiConfig';
import { JOINT_LIMITS } from '../lib/constants';
import { logger } from '../lib/logger';

export default function SettingsPage() {
  const { config, isLoading, error, fetchConfig, saveConfig } = useConfigStore();
  const [localConfig, setLocalConfig] = useState<Config | null>(null);
  const [hasChanges, setHasChanges] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [availablePorts, setAvailablePorts] = useState<(string | { device: string; description: string; hwid: string })[]>([]);
  const [loadingPorts, setLoadingPorts] = useState(false);

  // Saved position editor state
  const [editingPosition, setEditingPosition] = useState<{ index: number; name: string; joints: number[] } | null>(null);
  const [isAddingPosition, setIsAddingPosition] = useState(false);

  // Runtime controls
  const accel = useCommandStore((state) => state.accel);
  const setAccel = useCommandStore((state) => state.setAccel);

  // Hardware status
  const ioStatus = useHardwareStore((state) => state.ioStatus);
  const gripperStatus = useHardwareStore((state) => state.gripperStatus);

  // Fetch config on mount
  useEffect(() => {
    fetchConfig();
  }, [fetchConfig]);

  // Fetch available COM ports
  useEffect(() => {
    const fetchPorts = async () => {
      setLoadingPorts(true);
      try {
        const response = await fetch(`${getApiBaseUrl()}/api/config/com-ports`);
        if (response.ok) {
          const data = await response.json();
          setAvailablePorts(data.ports || []);
        }
      } catch (error) {
        logger.error('Failed to fetch COM ports', 'SettingsPage', error);
      } finally{
        setLoadingPorts(false);
      }
    };
    fetchPorts();
  }, []);

  // Update local state when config changes
  useEffect(() => {
    if (config) {
      setLocalConfig(JSON.parse(JSON.stringify(config))); // Deep copy
      setHasChanges(false);
    }
  }, [config]);

  const handleSave = async () => {
    if (!localConfig) return;

    setIsSaving(true);
    try {
      await saveConfig(localConfig);
      setHasChanges(false);
    } finally {
      setIsSaving(false);
    }
  };

  const handleReset = () => {
    if (config) {
      setLocalConfig(JSON.parse(JSON.stringify(config)));
      setHasChanges(false);
    }
  };

  const updateConfig = (path: string[], value: any) => {
    if (!localConfig) return;

    const newConfig = JSON.parse(JSON.stringify(localConfig));
    let current = newConfig;

    for (let i = 0; i < path.length - 1; i++) {
      current = current[path[i]];
    }
    current[path[path.length - 1]] = value;

    setLocalConfig(newConfig);
    setHasChanges(true);
  };

  // Validate joint angles against limits
  const validateJointAngles = (joints: number[]): { valid: boolean; errors: string[] } => {
    const errors: string[] = [];
    if (joints.length !== 6) {
      errors.push('Must have exactly 6 joint values');
      return { valid: false, errors };
    }

    joints.forEach((angle, index) => {
      const jointName = `J${index + 1}`;
      const limits = JOINT_LIMITS[jointName];
      if (angle < limits.min || angle > limits.max) {
        errors.push(`${jointName}: ${angle.toFixed(1)}° is out of range [${limits.min.toFixed(1)}°, ${limits.max.toFixed(1)}°]`);
      }
    });

    return { valid: errors.length === 0, errors };
  };

  const handleAddPosition = () => {
    setIsAddingPosition(true);
    setEditingPosition({ index: -1, name: '', joints: [90, -90, 180, 0, 0, 180] });
  };

  const handleEditPosition = (index: number) => {
    if (!localConfig) return;
    const position = localConfig.ui.saved_positions[index];
    setIsAddingPosition(false);
    setEditingPosition({ index, name: position.name, joints: [...position.joints] });
  };

  const handleDeletePosition = (index: number) => {
    if (!localConfig) return;
    if (!confirm(`Delete position "${localConfig.ui.saved_positions[index].name}"?`)) return;

    const newPositions = localConfig.ui.saved_positions.filter((_, i) => i !== index);
    updateConfig(['ui', 'saved_positions'], newPositions);
  };

  const handleSavePosition = () => {
    if (!localConfig || !editingPosition) return;

    // Validate name
    if (!editingPosition.name.trim()) {
      alert('Position name cannot be empty');
      return;
    }

    // Validate joint angles
    const validation = validateJointAngles(editingPosition.joints);
    if (!validation.valid) {
      alert('Invalid joint angles:\n' + validation.errors.join('\n'));
      return;
    }

    const newPosition = { name: editingPosition.name, joints: editingPosition.joints };
    let newPositions;

    if (isAddingPosition) {
      // Add new position
      newPositions = [...localConfig.ui.saved_positions, newPosition];
    } else {
      // Update existing position
      newPositions = localConfig.ui.saved_positions.map((pos, i) =>
        i === editingPosition.index ? newPosition : pos
      );
    }

    updateConfig(['ui', 'saved_positions'], newPositions);
    setEditingPosition(null);
    setIsAddingPosition(false);
  };

  const handleCancelEditPosition = () => {
    setEditingPosition(null);
    setIsAddingPosition(false);
  };

  if (isLoading && !localConfig) {
    return (
      <main className="h-screen flex flex-col bg-background">
        <Header />
        <div className="flex-1 flex items-center justify-center">
          <Loader2 className="h-8 w-8 animate-spin text-muted-foreground" />
        </div>
      </main>
    );
  }

  if (error) {
    return (
      <main className="h-screen flex flex-col bg-background">
        <Header />
        <div className="flex-1 flex items-center justify-center p-8">
          <Card className="p-8">
            <AlertCircle className="h-12 w-12 text-red-500 mx-auto mb-4" />
            <h2 className="text-xl font-bold mb-2 text-center">Error Loading Config</h2>
            <p className="text-muted-foreground text-center">{error}</p>
            <Button onClick={fetchConfig} className="mt-4 mx-auto block">
              Retry
            </Button>
          </Card>
        </div>
      </main>
    );
  }

  if (!localConfig) return null;

  // Ensure ui config exists
  if (!localConfig.ui) {
    return (
      <main className="h-screen flex flex-col bg-background">
        <Header />
        <div className="flex-1 flex items-center justify-center">
          <div className="text-center">
            <AlertCircle className="h-8 w-8 text-destructive mx-auto mb-2" />
            <p className="text-muted-foreground">UI configuration is missing</p>
          </div>
        </div>
      </main>
    );
  }

  return (
    <main className="h-screen flex flex-col bg-background">
      <Header />

      <div className="flex-1 overflow-y-auto p-8">
        <div className="max-w-4xl mx-auto space-y-6">
          {/* Header with Save/Reset Buttons */}
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-3xl font-bold">Settings</h1>
              <p className="text-muted-foreground">Configure robot and UI preferences</p>
            </div>
            <div className="flex gap-2">
              <Button
                variant="outline"
                onClick={handleReset}
                disabled={!hasChanges || isSaving}
              >
                <RotateCcw className="h-4 w-4 mr-2" />
                Reset
              </Button>
              <Button onClick={handleSave} disabled={!hasChanges || isSaving}>
                {isSaving ? (
                  <>
                    <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                    Saving...
                  </>
                ) : (
                  <>
                    <Save className="h-4 w-4 mr-2" />
                    Save Changes
                  </>
                )}
              </Button>
            </div>
          </div>

          {/* Basic Settings */}
          <Card className="p-6">
            <h2 className="text-xl font-semibold mb-4">Basic Settings</h2>
            <div className="space-y-4">
              {/* Default Speed Percentage */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Default Speed %</Label>
                <Input
                  type="number"
                  min="1"
                  max="100"
                  value={localConfig.ui.default_speed_percentage}
                  onChange={(e) =>
                    updateConfig(['ui', 'default_speed_percentage'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Default Acceleration Percentage */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Default Accel %</Label>
                <Input
                  type="number"
                  min="1"
                  max="100"
                  value={localConfig.ui.default_acceleration_percentage}
                  onChange={(e) =>
                    updateConfig(['ui', 'default_acceleration_percentage'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Runtime Acceleration Control */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Current Accel %</Label>
                <div className="col-span-2 flex items-center gap-2">
                  <Slider
                    value={[accel]}
                    onValueChange={(value) => setAccel(value[0])}
                    min={0}
                    max={100}
                    step={1}
                    className="flex-1"
                  />
                  <span className="text-sm text-muted-foreground w-12 text-right">
                    {accel}%
                  </span>
                </div>
              </div>

              {/* Step Angle */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Step Angle (degrees)</Label>
                <Input
                  type="number"
                  min="0.1"
                  max="10"
                  step="0.1"
                  value={localConfig.ui.step_angle}
                  onChange={(e) =>
                    updateConfig(['ui', 'step_angle'], parseFloat(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Cartesian Position Step */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Cartesian Position Step (mm)</Label>
                <Input
                  type="number"
                  min="0.1"
                  max="100"
                  step="0.1"
                  value={localConfig.ui.cartesian_position_step_mm}
                  onChange={(e) =>
                    updateConfig(['ui', 'cartesian_position_step_mm'], parseFloat(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Show Safety Warnings */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Show Safety Warnings</Label>
                <div className="col-span-2 flex items-center">
                  <Checkbox
                    checked={localConfig.ui.show_safety_warnings}
                    onCheckedChange={(checked) =>
                      updateConfig(['ui', 'show_safety_warnings'], checked)
                    }
                  />
                </div>
              </div>
            </div>
          </Card>

          {/* Robot Appearance */}
          <Card className="p-6">
            <h2 className="text-xl font-semibold mb-4">Robot Appearance</h2>
            <div className="space-y-4">
              {/* Hardware Robot Color */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Hardware Robot Color</Label>
                <div className="col-span-2 flex gap-2 items-center">
                  <Input
                    type="color"
                    value={localConfig.ui.hardware_robot.color}
                    onChange={(e) =>
                      updateConfig(['ui', 'hardware_robot', 'color'], e.target.value)
                    }
                    className="w-20 h-10 cursor-pointer"
                  />
                  <Input
                    type="text"
                    value={localConfig.ui.hardware_robot.color}
                    onChange={(e) =>
                      updateConfig(['ui', 'hardware_robot', 'color'], e.target.value)
                    }
                    className="flex-1"
                    placeholder="#808080"
                  />
                </div>
              </div>

              {/* Hardware Robot Transparency */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Hardware Robot Transparency</Label>
                <div className="col-span-2 flex gap-2 items-center">
                  <Input
                    type="range"
                    min="0"
                    max="1"
                    step="0.05"
                    value={localConfig.ui.hardware_robot.transparency}
                    onChange={(e) =>
                      updateConfig(['ui', 'hardware_robot', 'transparency'], parseFloat(e.target.value))
                    }
                    className="flex-1"
                  />
                  <span className="text-sm text-muted-foreground w-12 text-right">
                    {(localConfig.ui.hardware_robot.transparency * 100).toFixed(0)}%
                  </span>
                </div>
              </div>

              {/* Commander Robot Color */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Commander Robot Color</Label>
                <div className="col-span-2 flex gap-2 items-center">
                  <Input
                    type="color"
                    value={localConfig.ui.commander_robot.color}
                    onChange={(e) =>
                      updateConfig(['ui', 'commander_robot', 'color'], e.target.value)
                    }
                    className="w-20 h-10 cursor-pointer"
                  />
                  <Input
                    type="text"
                    value={localConfig.ui.commander_robot.color}
                    onChange={(e) =>
                      updateConfig(['ui', 'commander_robot', 'color'], e.target.value)
                    }
                    className="flex-1"
                    placeholder="#4ECDC4"
                  />
                </div>
              </div>

              {/* Commander Robot Transparency */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Commander Robot Transparency</Label>
                <div className="col-span-2 flex gap-2 items-center">
                  <Input
                    type="range"
                    min="0"
                    max="1"
                    step="0.05"
                    value={localConfig.ui.commander_robot.transparency}
                    onChange={(e) =>
                      updateConfig(['ui', 'commander_robot', 'transparency'], parseFloat(e.target.value))
                    }
                    className="flex-1"
                  />
                  <span className="text-sm text-muted-foreground w-12 text-right">
                    {(localConfig.ui.commander_robot.transparency * 100).toFixed(0)}%
                  </span>
                </div>
              </div>
            </div>
          </Card>

          {/* Saved Positions */}
          <Card className="p-6">
            <div className="flex items-center justify-between mb-4">
              <h2 className="text-xl font-semibold">Saved Positions</h2>
              <Button onClick={handleAddPosition} size="sm">
                <Plus className="h-4 w-4 mr-2" />
                Add Position
              </Button>
            </div>

            {/* Position Editor Modal/Inline Form */}
            {editingPosition && (
              <div className="mb-4 p-4 border rounded-lg bg-muted/50">
                <h3 className="font-semibold mb-3">{isAddingPosition ? 'Add New Position' : 'Edit Position'}</h3>
                <div className="space-y-3">
                  {/* Name Input */}
                  <div>
                    <Label className="text-sm">Position Name</Label>
                    <Input
                      value={editingPosition.name}
                      onChange={(e) => setEditingPosition({ ...editingPosition, name: e.target.value })}
                      placeholder="e.g., Home, Park, Ready"
                      className="mt-1"
                    />
                  </div>

                  {/* Joint Angles */}
                  <div>
                    <Label className="text-sm">Joint Angles (degrees)</Label>
                    <div className="grid grid-cols-6 gap-2 mt-1">
                      {editingPosition.joints.map((angle, index) => (
                        <div key={index}>
                          <Label className="text-xs text-muted-foreground">J{index + 1}</Label>
                          <Input
                            type="number"
                            step="0.1"
                            value={angle}
                            onChange={(e) => {
                              const newJoints = [...editingPosition.joints];
                              newJoints[index] = parseFloat(e.target.value) || 0;
                              setEditingPosition({ ...editingPosition, joints: newJoints });
                            }}
                            className="text-xs"
                          />
                        </div>
                      ))}
                    </div>
                  </div>

                  {/* Action Buttons */}
                  <div className="flex gap-2 pt-2">
                    <Button onClick={handleSavePosition} size="sm">
                      Save
                    </Button>
                    <Button onClick={handleCancelEditPosition} variant="outline" size="sm">
                      Cancel
                    </Button>
                  </div>
                </div>
              </div>
            )}

            {/* Positions Table */}
            <div className="border rounded-lg overflow-hidden">
              <table className="w-full">
                <thead className="bg-muted">
                  <tr>
                    <th className="text-left p-2 font-semibold text-sm">Name</th>
                    <th className="text-center p-2 font-semibold text-sm w-16">J1</th>
                    <th className="text-center p-2 font-semibold text-sm w-16">J2</th>
                    <th className="text-center p-2 font-semibold text-sm w-16">J3</th>
                    <th className="text-center p-2 font-semibold text-sm w-16">J4</th>
                    <th className="text-center p-2 font-semibold text-sm w-16">J5</th>
                    <th className="text-center p-2 font-semibold text-sm w-16">J6</th>
                    <th className="text-center p-2 font-semibold text-sm w-24">Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {localConfig.ui.saved_positions.length === 0 ? (
                    <tr>
                      <td colSpan={8} className="text-center p-4 text-muted-foreground text-sm">
                        No saved positions. Click "Add Position" to create one.
                      </td>
                    </tr>
                  ) : (
                    localConfig.ui.saved_positions.map((position, index) => (
                      <tr key={index} className="border-t hover:bg-muted/50">
                        <td className="p-2 font-medium">{position.name}</td>
                        {position.joints.map((angle, jointIndex) => (
                          <td key={jointIndex} className="text-center p-2 text-sm font-mono">
                            {angle.toFixed(1)}
                          </td>
                        ))}
                        <td className="p-2">
                          <div className="flex gap-1 justify-center">
                            <Button
                              onClick={() => handleEditPosition(index)}
                              variant="ghost"
                              size="sm"
                              className="h-7 w-7 p-0"
                            >
                              <Pencil className="h-3 w-3" />
                            </Button>
                            <Button
                              onClick={() => handleDeletePosition(index)}
                              variant="ghost"
                              size="sm"
                              className="h-7 w-7 p-0 text-destructive hover:text-destructive"
                            >
                              <Trash2 className="h-3 w-3" />
                            </Button>
                          </div>
                        </td>
                      </tr>
                    ))
                  )}
                </tbody>
              </table>
            </div>
          </Card>

          {/* WebSocket Settings */}
          <Card className="p-6">
            <h2 className="text-xl font-semibold mb-4">WebSocket Settings</h2>
            <div className="space-y-4">
              {/* Default Rate */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Update Rate (Hz)</Label>
                <Input
                  type="number"
                  min="1"
                  max="50"
                  value={localConfig.frontend.websocket.default_rate_hz}
                  onChange={(e) =>
                    updateConfig(['frontend', 'websocket', 'default_rate_hz'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Max Reconnect Attempts */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Max Reconnect Attempts</Label>
                <Input
                  type="number"
                  min="1"
                  max="100"
                  value={localConfig.frontend.websocket.reconnect.max_attempts}
                  onChange={(e) =>
                    updateConfig(['frontend', 'websocket', 'reconnect', 'max_attempts'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Reconnect Delay */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Reconnect Delay (ms)</Label>
                <Input
                  type="number"
                  min="100"
                  max="10000"
                  step="100"
                  value={localConfig.frontend.websocket.reconnect.base_delay_ms}
                  onChange={(e) =>
                    updateConfig(['frontend', 'websocket', 'reconnect', 'base_delay_ms'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>
            </div>
          </Card>

          {/* Logging Settings */}
          <Card className="p-6">
            <h2 className="text-xl font-semibold mb-4">Logging Settings</h2>
            <div className="space-y-4">
              {/* Commander Log Level */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Commander Log Level</Label>
                <Select
                  value={localConfig.logging.commander?.level || 'INFO'}
                  onValueChange={(value) => updateConfig(['logging', 'commander', 'level'], value)}
                >
                  <SelectTrigger className="col-span-2">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="DEBUG">DEBUG</SelectItem>
                    <SelectItem value="INFO">INFO</SelectItem>
                    <SelectItem value="WARNING">WARNING</SelectItem>
                    <SelectItem value="ERROR">ERROR</SelectItem>
                    <SelectItem value="CRITICAL">CRITICAL</SelectItem>
                  </SelectContent>
                </Select>
              </div>

              {/* API Log Level */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>API Log Level</Label>
                <Select
                  value={localConfig.logging.api?.level || 'INFO'}
                  onValueChange={(value) => updateConfig(['logging', 'api', 'level'], value)}
                >
                  <SelectTrigger className="col-span-2">
                    <SelectValue />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="DEBUG">DEBUG</SelectItem>
                    <SelectItem value="INFO">INFO</SelectItem>
                    <SelectItem value="WARNING">WARNING</SelectItem>
                    <SelectItem value="ERROR">ERROR</SelectItem>
                    <SelectItem value="CRITICAL">CRITICAL</SelectItem>
                  </SelectContent>
                </Select>
              </div>

              {/* Buffer Size */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Buffer Size</Label>
                <Input
                  type="number"
                  min="100"
                  max="10000"
                  step="100"
                  value={localConfig.logging.buffer_size}
                  onChange={(e) =>
                    updateConfig(['logging', 'buffer_size'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Initial Log Count */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Initial Log Count</Label>
                <Input
                  type="number"
                  min="0"
                  max="1000"
                  value={localConfig.logging.initial_log_count}
                  onChange={(e) =>
                    updateConfig(['logging', 'initial_log_count'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>
            </div>
          </Card>

          {/* Advanced Settings */}
          <Card className="p-6 border-yellow-500/50">
            <div className="flex items-start gap-3 mb-4">
              <AlertCircle className="h-5 w-5 text-yellow-500 mt-0.5" />
              <div>
                <h2 className="text-xl font-semibold">Advanced Settings</h2>
                <p className="text-sm text-yellow-600 dark:text-yellow-500">
                  Changes to these settings require restarting the backend server
                </p>
              </div>
            </div>
            <div className="space-y-4">
              {/* COM Port */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>COM Port</Label>
                {loadingPorts ? (
                  <div className="col-span-2 flex items-center gap-2 text-muted-foreground">
                    <Loader2 className="h-4 w-4 animate-spin" />
                    <span className="text-sm">Scanning ports...</span>
                  </div>
                ) : availablePorts.length > 0 ? (
                  <Select
                    value={localConfig.robot.com_port}
                    onValueChange={(value) => updateConfig(['robot', 'com_port'], value)}
                  >
                    <SelectTrigger className="col-span-2">
                      <SelectValue />
                    </SelectTrigger>
                    <SelectContent>
                      {availablePorts.map((port) => {
                        const portValue = typeof port === 'string' ? port : port.device;
                        const portLabel = typeof port === 'string' ? port : `${port.device} - ${port.description}`;
                        return (
                          <SelectItem key={portValue} value={portValue}>
                            {portLabel}
                          </SelectItem>
                        );
                      })}
                    </SelectContent>
                  </Select>
                ) : (
                  <Input
                    value={localConfig.robot.com_port}
                    onChange={(e) => updateConfig(['robot', 'com_port'], e.target.value)}
                    placeholder="No ports detected - enter manually"
                    className="col-span-2"
                  />
                )}
              </div>

              {/* Baud Rate */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Baud Rate</Label>
                <Input
                  type="number"
                  value={localConfig.robot.baud_rate}
                  onChange={(e) =>
                    updateConfig(['robot', 'baud_rate'], parseInt(e.target.value))
                  }
                  className="col-span-2"
                />
              </div>

              {/* Auto Home on Startup */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>Auto Home on Startup</Label>
                <div className="col-span-2 flex items-center">
                  <Checkbox
                    checked={localConfig.robot.auto_home_on_startup}
                    onCheckedChange={(checked) =>
                      updateConfig(['robot', 'auto_home_on_startup'], checked)
                    }
                  />
                </div>
              </div>

              {/* API Port */}
              <div className="grid grid-cols-3 items-center gap-4">
                <Label>API Port</Label>
                <Input
                  type="number"
                  min="1024"
                  max="65535"
                  value={localConfig.api.port}
                  onChange={(e) => updateConfig(['api', 'port'], parseInt(e.target.value))}
                  className="col-span-2"
                />
              </div>
            </div>
          </Card>

          {/* Runtime Controls */}
          <Card className="p-6">
            <h2 className="text-lg font-semibold mb-4">Runtime Controls</h2>
            <div className="space-y-4">
              {/* Acceleration Slider */}
              <div>
                <Label className="text-sm font-medium mb-2 block">
                  Acceleration: {accel}%
                </Label>
                <Slider
                  value={[accel]}
                  onValueChange={(value) => setAccel(value[0])}
                  min={0}
                  max={100}
                  step={1}
                  className="w-full"
                />
              </div>
            </div>
          </Card>

          {/* I/O Status */}
          <Card className="p-6">
            <h2 className="text-lg font-semibold mb-4">I/O Status</h2>
            <div className="space-y-3 text-sm">
              {/* Digital I/O */}
              <div>
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
                <div className="flex items-center gap-2 mt-3">
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

              {/* Gripper Status */}
              <div className="pt-3 border-t">
                <h3 className="font-semibold mb-2 text-sm">Gripper</h3>
                <div className="flex items-center gap-4 text-sm font-mono">
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
            </div>
          </Card>
        </div>
      </div>
    </main>
  );
}
