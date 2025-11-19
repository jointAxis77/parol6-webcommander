'use client';

import { useEffect, useRef, useState, useMemo } from 'react';
import Header from './components/Header';
import RobotViewer from './components/RobotViewer';
import CompactJointSliders from './components/CompactJointSliders';
import CartesianSliders from './components/CartesianSliders';
import ControlOptions from './components/ControlOptions';
import Timeline from './components/Timeline';
import { useConfigStore } from './lib/configStore';
import { useActualFollowsTarget } from './hooks/useActualFollowsTarget';
import { usePlayback } from './hooks/usePlayback';
import { useScrubbing } from './hooks/useScrubbing';
import { useTimelineStore } from './lib/stores/timelineStore';
import { useCommandStore } from './lib/stores/commandStore';
import { useInputStore } from './lib/stores/inputStore';
import { useRobotConfigStore } from './lib/stores/robotConfigStore';
import { useHardwareStore } from './lib/stores/hardwareStore';
import { ToggleGroup, ToggleGroupItem } from '@/components/ui/toggle-group';
import { Slider } from '@/components/ui/slider';
import { Label } from '@/components/ui/label';
import { Switch } from '@/components/ui/switch';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Button } from '@/components/ui/button';
import { Card } from '@/components/ui/card';
import { Collapsible, CollapsibleContent, CollapsibleTrigger } from '@/components/ui/collapsible';
import { ChevronDown, Download, Upload } from 'lucide-react';
import { cn } from '@/lib/utils';
import { MotionMode } from './lib/types';
import { getApiBaseUrl } from './lib/apiConfig';
import { logger } from './lib/logger';

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

export default function Home() {
  // Initialize playback loop
  usePlayback();

  // Initialize scrubbing (robot follows playhead when not playing)
  useScrubbing();

  // Enable live control mode - automatically sends move commands when target changes
  useActualFollowsTarget();

  // Fetch config from backend on mount
  const { config, fetchConfig } = useConfigStore();

  // Tool management state
  const [tools, setTools] = useState<Tool[]>([]);
  const [activeToolId, setActiveToolId] = useState<string | null>(null);

  // Timeline collapse state
  const [timelineOpen, setTimelineOpen] = useState(true);
  const fileInputRef = useRef<HTMLInputElement>(null);

  // Timeline export/import
  const exportTimeline = useTimelineStore((state) => state.exportTimeline);
  const loadTimeline = useTimelineStore((state) => state.loadTimeline);

  // Get mode and related state
  const motionMode = useTimelineStore((state) => state.timeline.mode);
  const setMotionMode = useTimelineStore((state) => state.setMotionMode);
  const speed = useCommandStore((state) => state.speed);
  const setSpeed = useCommandStore((state) => state.setSpeed);
  const setAccel = useCommandStore((state) => state.setAccel);
  const setStepAngle = useInputStore((state) => state.setStepAngle);
  const setCartesianPositionStep = useInputStore((state) => state.setCartesianPositionStep);
  const commandedTcpPose = useCommandStore((state) => state.commandedTcpPose);

  // Live control state
  const liveControlEnabled = useCommandStore((state) => state.liveControlEnabled);
  const setLiveControlEnabled = useCommandStore((state) => state.setLiveControlEnabled);
  const hardwareJointAngles = useHardwareStore((state) => state.hardwareJointAngles);

  // Gripper state
  const commandedGripperState = useCommandStore((state) => state.commandedGripperState);
  const setCommandedGripperState = useCommandStore((state) => state.setCommandedGripperState);

  // Derive active tool from tools and activeToolId
  const activeTool = useMemo(() => {
    return tools.find((t) => t.id === activeToolId) || null;
  }, [tools, activeToolId]);

  // Robot config setters
  const setTcpOffset = useRobotConfigStore((state) => state.setTcpOffset);
  const setHardwareRobotColor = useRobotConfigStore((state) => state.setHardwareRobotColor);
  const setHardwareRobotTransparency = useRobotConfigStore((state) => state.setHardwareRobotTransparency);
  const setCommanderRobotColor = useRobotConfigStore((state) => state.setCommanderRobotColor);
  const setCommanderRobotTransparency = useRobotConfigStore((state) => state.setCommanderRobotTransparency);

  // Track if we've synced the RGB gizmo for current cartesian session
  const hasSyncedRef = useRef(false);

  useEffect(() => {
    fetchConfig();
  }, [fetchConfig]);

  // Fetch tools from backend on mount
  useEffect(() => {
    const fetchTools = async () => {
      try {
        const response = await fetch(`${getApiBaseUrl()}/api/config/tools`);
        if (response.ok) {
          const data = await response.json();
          setTools(data.tools || []);
          setActiveToolId(data.active_tool_id || null);

          // Load TCP offset from active tool
          if (data.active_tool_id) {
            const activeTool = data.tools?.find((t: Tool) => t.id === data.active_tool_id);
            if (activeTool?.tcp_offset) {
              setTcpOffset('x', activeTool.tcp_offset.x);
              setTcpOffset('y', activeTool.tcp_offset.y);
              setTcpOffset('z', activeTool.tcp_offset.z);
              setTcpOffset('rx', activeTool.tcp_offset.rx ?? 0);
              setTcpOffset('ry', activeTool.tcp_offset.ry ?? 0);
              setTcpOffset('rz', activeTool.tcp_offset.rz ?? 0);
            }
          }
        }
      } catch (error) {
        logger.error('Failed to fetch tools', 'page', error);
      }
    };
    fetchTools();
  }, []);

  // Sync default_speed_percentage from config to commandStore when config loads
  useEffect(() => {
    if (config?.ui?.default_speed_percentage !== undefined) {
      setSpeed(config.ui.default_speed_percentage);
    }
  }, [config, setSpeed]);

  // Sync default_acceleration_percentage from config to commandStore when config loads
  useEffect(() => {
    if (config?.ui?.default_acceleration_percentage !== undefined) {
      setAccel(config.ui.default_acceleration_percentage);
    }
  }, [config, setAccel]);

  // Sync step_angle from config to inputStore when config loads
  useEffect(() => {
    if (config?.ui?.step_angle !== undefined) {
      setStepAngle(config.ui.step_angle);
    }
  }, [config, setStepAngle]);

  // Sync cartesian_position_step_mm from config to inputStore when config loads
  useEffect(() => {
    if (config?.ui?.cartesian_position_step_mm !== undefined) {
      setCartesianPositionStep(config.ui.cartesian_position_step_mm);
    }
  }, [config, setCartesianPositionStep]);

  // Note: TCP offset is now synced from active tool when tool is mounted (see handleToolChange)

  // Sync robot appearance from config to robotConfigStore when config loads
  useEffect(() => {
    if (config?.ui?.hardware_robot) {
      setHardwareRobotColor(config.ui.hardware_robot.color);
      setHardwareRobotTransparency(config.ui.hardware_robot.transparency);
    }
    if (config?.ui?.commander_robot) {
      setCommanderRobotColor(config.ui.commander_robot.color);
      setCommanderRobotTransparency(config.ui.commander_robot.transparency);
    }
  }, [config, setHardwareRobotColor, setHardwareRobotTransparency, setCommanderRobotColor, setCommanderRobotTransparency]);

  // Auto-sync cartesian pose to robot TCP when switching to cartesian mode
  // Only runs ONCE per cartesian session to prevent feedback loop
  useEffect(() => {
    if (motionMode === 'cartesian' && commandedTcpPose && !hasSyncedRef.current) {
      useInputStore.setState({
        inputCartesianPose: {
          X: commandedTcpPose.X,
          Y: commandedTcpPose.Y,
          Z: commandedTcpPose.Z,
          RX: commandedTcpPose.RX,
          RY: commandedTcpPose.RY,
          RZ: commandedTcpPose.RZ
        }
      });
      hasSyncedRef.current = true;
    }

    // Reset sync flag when leaving cartesian mode
    if (motionMode !== 'cartesian') {
      hasSyncedRef.current = false;
    }
  }, [motionMode, commandedTcpPose]);

  // Handle mode change
  const handleModeChange = (newMode: string) => {
    if (newMode === 'joint' || newMode === 'cartesian') {
      setMotionMode(newMode as MotionMode);
    }
  };

  // Handle tool change - mount tool and sync TCP offset
  const handleToolChange = async (toolId: string) => {
    try {
      const response = await fetch(
        `${getApiBaseUrl()}/api/config/tools/${toolId}/mount`,
        { method: 'POST' }
      );

      if (response.ok) {
        const data = await response.json();

        // Sync TCP offset to robotConfigStore
        if (data.tcp_offset) {
          setTcpOffset('x', data.tcp_offset.x);
          setTcpOffset('y', data.tcp_offset.y);
          setTcpOffset('z', data.tcp_offset.z);
          setTcpOffset('rx', data.tcp_offset.rx ?? 0);
          setTcpOffset('ry', data.tcp_offset.ry ?? 0);
          setTcpOffset('rz', data.tcp_offset.rz ?? 0);
        }

        // Update active tool ID
        setActiveToolId(toolId);
      } else {
        logger.error('Failed to mount tool', 'page', response.statusText);
      }
    } catch (error) {
      logger.error('Error mounting tool', 'page', error);
    }
  };

  // Handle timeline export
  const handleExport = () => {
    const json = exportTimeline();
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'timeline.json';
    a.click();
    URL.revokeObjectURL(url);
  };

  // Handle timeline import
  const handleImport = () => {
    fileInputRef.current?.click();
  };

  const handleFileSelected = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (event) => {
      try {
        const timeline = JSON.parse(event.target?.result as string);
        loadTimeline(timeline);
      } catch (error) {
        alert('Failed to load timeline file. Please check the file format.');
      }
    };
    reader.readAsText(file);

    // Reset file input
    if (fileInputRef.current) {
      fileInputRef.current.value = '';
    }
  };

  return (
    <main className="h-screen flex flex-col bg-background">
      <Header />

      {/* Main Content Grid */}
      <div className="flex-1 flex gap-4 min-h-0 p-4">
        {/* Left Column: 3D View + Timeline */}
        <div className="flex-1 flex flex-col gap-4 min-w-0 overflow-hidden">
          {/* 3D Robot View - Flexible height */}
          <div className="flex-1 min-h-0 overflow-hidden">
            <RobotViewer activeToolId={activeToolId || undefined} />
          </div>

          {/* Timeline Editor - Collapsable */}
          <Collapsible open={timelineOpen} onOpenChange={setTimelineOpen} className="flex-shrink-0">
            <Card className={cn("transition-all", timelineOpen ? "h-[400px]" : "h-auto")}>
              <CollapsibleTrigger asChild>
                <div className="flex items-center justify-between p-3 border-b cursor-pointer hover:bg-accent/50 transition-colors">
                  <div className="flex items-center gap-2">
                    <ChevronDown className={cn("h-4 w-4 transition-transform", !timelineOpen && "-rotate-90")} />
                    <h2 className="text-sm font-semibold">Timeline Editor</h2>
                  </div>
                  <div className="flex items-center gap-2" onClick={(e) => e.stopPropagation()}>
                    <Button size="sm" variant="outline" onClick={handleExport} className="h-7 text-xs">
                      <Download className="h-3 w-3 mr-1" />
                      Export
                    </Button>
                    <Button size="sm" variant="outline" onClick={handleImport} className="h-7 text-xs">
                      <Upload className="h-3 w-3 mr-1" />
                      Import
                    </Button>
                    <input
                      ref={fileInputRef}
                      type="file"
                      accept=".json"
                      className="hidden"
                      onChange={handleFileSelected}
                    />
                  </div>
                </div>
              </CollapsibleTrigger>
              <CollapsibleContent className={cn("transition-all overflow-visible", timelineOpen ? "h-[356px]" : "h-0")}>
                <Timeline />
              </CollapsibleContent>
            </Card>
          </Collapsible>
        </div>

        {/* Right Column: Control Panels - Full height */}
        <div className="w-[300px] flex-shrink-0 flex flex-col gap-4">
          {/* Mode Toggle */}
          <div className="bg-card rounded-lg border p-3 flex items-center gap-3">
            <span className="text-sm font-semibold text-muted-foreground">Motion Mode:</span>
            <ToggleGroup type="single" value={motionMode} onValueChange={handleModeChange}>
              <ToggleGroupItem value="joint" className="px-4">
                Joint Space
              </ToggleGroupItem>
              <ToggleGroupItem value="cartesian" className="px-4">
                Cartesian Space
              </ToggleGroupItem>
            </ToggleGroup>
          </div>

          {/* Tool Selector */}
          <div className="bg-card rounded-lg border p-3">
            <Label className="text-sm font-medium mb-2 block">Active Tool</Label>
            <Select value={activeToolId || undefined} onValueChange={handleToolChange}>
              <SelectTrigger className="w-full">
                <SelectValue placeholder="Select tool..." />
              </SelectTrigger>
              <SelectContent>
                {tools.map((tool) => (
                  <SelectItem key={tool.id} value={tool.id}>
                    {tool.name}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>

            {/* Gripper State Toggle */}
            {activeTool?.gripper_config?.enabled && (
              <div className="flex items-center justify-between mt-3 pt-3 border-t">
                <Label className="text-sm">
                  Gripper: {commandedGripperState === 'open' ? 'Open' : 'Closed'}
                </Label>
                <Switch
                  checked={commandedGripperState === 'closed'}
                  onCheckedChange={(checked) => setCommandedGripperState(checked ? 'closed' : 'open')}
                />
              </div>
            )}
          </div>

          {/* Live Control Switch */}
          <div className="bg-card rounded-lg border p-3">
            <div className="flex items-center justify-between">
              <Label className={liveControlEnabled && hardwareJointAngles !== null ? "text-yellow-400 font-semibold text-sm" : "text-sm"}>
                Live Control (HW Follows)
                {liveControlEnabled && hardwareJointAngles !== null && (
                  <span className="ml-2 text-[9px] bg-yellow-500/20 px-1 py-0.5 rounded">LIVE</span>
                )}
              </Label>
              <Switch
                checked={liveControlEnabled && hardwareJointAngles !== null}
                onCheckedChange={setLiveControlEnabled}
                disabled={hardwareJointAngles === null}
              />
            </div>
          </div>

          {/* Control Sliders - Auto height */}
          <div className="flex-shrink-0">
            {motionMode === 'joint' ? (
              <CompactJointSliders />
            ) : (
              <div className="bg-card rounded-lg border p-4">
                <h2 className="text-sm font-semibold mb-4">Cartesian Control</h2>
                <CartesianSliders />
              </div>
            )}
          </div>

          {/* Speed Control */}
          <div className="flex-shrink-0">
            <div className="bg-card rounded-lg border p-3">
              <Label className="text-xs font-medium mb-2 block">
                Speed: {speed}%
              </Label>
              <Slider
                value={[speed]}
                onValueChange={(value) => setSpeed(value[0])}
                min={0}
                max={100}
                step={1}
                className="w-full"
              />
            </div>
          </div>

          {/* Control Options - Auto height */}
          <div className="flex-shrink-0">
            <ControlOptions />
          </div>
        </div>
      </div>
    </main>
  );
}
