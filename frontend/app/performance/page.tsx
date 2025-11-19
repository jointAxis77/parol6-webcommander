'use client';

import { useEffect, useState } from 'react';
import Header from '../components/Header';
import { Card } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { ChartContainer, ChartTooltip, ChartTooltipContent, ChartLegend, ChartLegendContent } from '@/components/ui/chart';
import { LineChart, Line, BarChart, Bar, XAxis, YAxis, CartesianGrid, ReferenceLine } from 'recharts';
import { useHardwareStore, usePerformanceStore } from '../lib/stores';
import { Download, Trash2, RefreshCw } from 'lucide-react';

export default function PerformancePage() {
  // Live Hz monitoring state
  const robotStatus = useHardwareStore((state) => state.robotStatus);
  const [hzData, setHzData] = useState<Array<{ timestamp: number; hz: number; time: string }>>([]);

  // Performance recording state
  const recordings = usePerformanceStore((state) => state.recordings);
  const selectedRecording = usePerformanceStore((state) => state.selectedRecording);
  const selectedFilename = usePerformanceStore((state) => state.selectedFilename);
  const isLoadingRecordings = usePerformanceStore((state) => state.isLoadingRecordings);
  const isLoadingRecording = usePerformanceStore((state) => state.isLoadingRecording);
  const fetchRecordings = usePerformanceStore((state) => state.fetchRecordings);
  const selectRecording = usePerformanceStore((state) => state.selectRecording);
  const deleteRecording = usePerformanceStore((state) => state.deleteRecording);

  // Load recordings on mount
  useEffect(() => {
    fetchRecordings();
  }, [fetchRecordings]);

  // Update live Hz data
  useEffect(() => {
    const hz = robotStatus?.commander_hz;
    if (hz !== null && hz !== undefined) {
      const now = Date.now();
      const timeStr = new Date(now).toLocaleTimeString('en-US', { hour12: false });

      setHzData(prev => {
        const newPoint = { timestamp: now, hz, time: timeStr };
        const updated = [...prev, newPoint];

        // Keep only last 60 seconds of data
        const cutoff = now - 60000;
        return updated.filter(d => d.timestamp >= cutoff);
      });
    }
  }, [robotStatus?.commander_hz]);

  // Calculate live statistics
  const currentHz = robotStatus?.commander_hz ?? 0;
  const avgHz = hzData.length > 0
    ? hzData.reduce((sum, d) => sum + d.hz, 0) / hzData.length
    : 0;
  const minHz = hzData.length > 0 ? Math.min(...hzData.map(d => d.hz)) : 0;
  const maxHz = hzData.length > 0 ? Math.max(...hzData.map(d => d.hz)) : 0;

  // Process recording data for charts
  const processRecordingForCharts = () => {
    if (!selectedRecording) return { barData: [], hzData: [] };

    // Aggregate all samples from all commands
    const allSamples = selectedRecording.commands.flatMap(cmd => cmd.samples);

    // Group into max 100 bars
    const maxBars = 100;
    const samplesPerBar = Math.ceil(allSamples.length / maxBars);
    const barData = [];

    for (let i = 0; i < allSamples.length; i += samplesPerBar) {
      const group = allSamples.slice(i, i + samplesPerBar);

      // Average the phase times
      const avgNetwork = group.reduce((sum, s) => sum + s.network, 0) / group.length;
      const avgProcessing = group.reduce((sum, s) => sum + s.processing, 0) / group.length;
      const avgExecution = group.reduce((sum, s) => sum + s.execution, 0) / group.length;
      const avgSerial = group.reduce((sum, s) => sum + s.serial, 0) / group.length;
      const avgIkManip = group.reduce((sum, s) => sum + (s.ik_manipulability || 0), 0) / group.length;
      const avgIkSolve = group.reduce((sum, s) => sum + (s.ik_solve || 0), 0) / group.length;

      barData.push({
        group: Math.floor(i / samplesPerBar),
        network: Number(avgNetwork.toFixed(2)),
        processing: Number(avgProcessing.toFixed(2)),
        execution: Number(avgExecution.toFixed(2)),
        serial: Number(avgSerial.toFixed(2)),
        ik_manipulability: Number(avgIkManip.toFixed(2)),
        ik_solve: Number(avgIkSolve.toFixed(2)),
      });
    }

    // Hz data - use captured Hz value from recording
    const hzData = allSamples.map((sample, idx) => ({
      index: idx,
      progress: ((idx / allSamples.length) * 100).toFixed(0) + '%',
      hz: sample.hz ?? 0,
    }));

    return { barData, hzData };
  };

  const { barData, hzData: recordingHzData } = processRecordingForCharts();

  // Handle recording selection
  const handleRecordingSelect = (filename: string) => {
    if (filename === 'none') {
      selectRecording(null);
    } else {
      selectRecording(filename);
    }
  };

  // Handle delete
  const handleDelete = async () => {
    if (!selectedFilename) return;

    if (confirm(`Delete recording "${selectedFilename}"?`)) {
      await deleteRecording(selectedFilename);
    }
  };

  // Handle export
  const handleExport = () => {
    if (!selectedRecording || !selectedFilename) return;

    const dataStr = JSON.stringify(selectedRecording, null, 2);
    const dataBlob = new Blob([dataStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    const link = document.createElement('a');
    link.href = url;
    link.download = selectedFilename;
    link.click();
    URL.revokeObjectURL(url);
  };

  return (
    <main className="h-screen flex flex-col bg-background">
      <Header />

      <div className="flex-1 p-4 space-y-4 overflow-auto">
        {/* Live Performance Monitoring */}
        <div className="space-y-4">
          <h2 className="text-xl font-semibold">Live Performance Monitoring</h2>

          {/* Statistics Cards */}
          <div className="grid grid-cols-4 gap-4">
            <Card className="p-4">
              <div className="text-sm text-muted-foreground">Current</div>
              <div className="text-2xl font-bold">{currentHz.toFixed(1)} Hz</div>
            </Card>
            <Card className="p-4">
              <div className="text-sm text-muted-foreground">Average (60s)</div>
              <div className="text-2xl font-bold">{avgHz.toFixed(1)} Hz</div>
            </Card>
            <Card className="p-4">
              <div className="text-sm text-muted-foreground">Min (60s)</div>
              <div className="text-2xl font-bold text-orange-500">{minHz.toFixed(1)} Hz</div>
            </Card>
            <Card className="p-4">
              <div className="text-sm text-muted-foreground">Max (60s)</div>
              <div className="text-2xl font-bold text-green-500">{maxHz.toFixed(1)} Hz</div>
            </Card>
          </div>

          {/* Live Line Chart */}
          <Card className="p-6">
            <h3 className="text-lg font-semibold mb-4">Commander Loop Frequency (Last 60 Seconds)</h3>
            <ChartContainer
              config={{
                hz: {
                  label: "Frequency",
                  color: "hsl(var(--chart-1))",
                },
              }}
              className="h-[300px] w-full"
            >
              <LineChart data={hzData}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis
                  dataKey="time"
                  angle={-45}
                  textAnchor="end"
                  height={80}
                  interval="preserveStartEnd"
                />
                <YAxis
                  domain={[0, 120]}
                  label={{ value: 'Hz', angle: -90, position: 'insideLeft' }}
                />
                <ChartTooltip content={<ChartTooltipContent />} />
                <ReferenceLine
                  y={100}
                  stroke="hsl(var(--muted-foreground))"
                  strokeDasharray="5 5"
                  strokeWidth={2}
                  label={{ value: 'Target (100 Hz)', position: 'right', fill: 'hsl(var(--muted-foreground))' }}
                />
                <Line
                  type="monotone"
                  dataKey="hz"
                  stroke="var(--color-hz)"
                  strokeWidth={2}
                  dot={false}
                  isAnimationActive={false}
                />
              </LineChart>
            </ChartContainer>

            {hzData.length === 0 && (
              <div className="text-center text-muted-foreground py-8">
                Waiting for data...
              </div>
            )}
          </Card>
        </div>

        {/* Recording Analysis */}
        <div className="space-y-4 pt-4 border-t">
          <div className="flex items-center justify-between">
            <h2 className="text-xl font-semibold">Recording Analysis</h2>
            <Button
              variant="outline"
              size="sm"
              onClick={() => fetchRecordings()}
              disabled={isLoadingRecordings}
            >
              <RefreshCw className={`h-4 w-4 mr-2 ${isLoadingRecordings ? 'animate-spin' : ''}`} />
              Refresh
            </Button>
          </div>

          {/* Recording Selector */}
          <Card className="p-4">
            <div className="flex items-center gap-4">
              <div className="flex-1">
                <label className="text-sm font-medium mb-2 block">Select Recording</label>
                <Select value={selectedFilename || 'none'} onValueChange={handleRecordingSelect}>
                  <SelectTrigger>
                    <SelectValue placeholder="Choose a recording..." />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="none">None</SelectItem>
                    {recordings.map((recording) => (
                      <SelectItem key={recording.filename} value={recording.filename}>
                        {recording.name} ({new Date(recording.timestamp).toLocaleString()}) - {recording.num_commands} commands
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>

              {selectedFilename && (
                <div className="flex gap-2 pt-6">
                  <Button variant="outline" size="sm" onClick={handleExport}>
                    <Download className="h-4 w-4 mr-2" />
                    Export
                  </Button>
                  <Button variant="destructive" size="sm" onClick={handleDelete}>
                    <Trash2 className="h-4 w-4 mr-2" />
                    Delete
                  </Button>
                </div>
              )}
            </div>
          </Card>

          {/* Recording Charts */}
          {selectedRecording && (
            <>
              {/* Phase Breakdown Bar Chart */}
              <Card className="p-6">
                <h3 className="text-lg font-semibold mb-4">Phase Breakdown (Stacked)</h3>
                <ChartContainer
                  config={{
                    network: {
                      label: "Network",
                      color: "hsl(var(--chart-1))",
                    },
                    processing: {
                      label: "Processing",
                      color: "hsl(var(--chart-2))",
                    },
                    execution: {
                      label: "Execution",
                      color: "hsl(var(--chart-3))",
                    },
                    serial: {
                      label: "Serial",
                      color: "hsl(var(--chart-4))",
                    },
                    ik_manipulability: {
                      label: "IK Manipulability",
                      color: "hsl(var(--chart-5))",
                    },
                    ik_solve: {
                      label: "IK Solve",
                      color: "hsl(30, 80%, 50%)",
                    },
                  }}
                  className="h-[400px] w-full"
                >
                  <BarChart data={barData}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="group"
                      label={{ value: 'Cycle Group', position: 'insideBottom', offset: -5 }}
                    />
                    <YAxis
                      label={{ value: 'Time (ms)', angle: -90, position: 'insideLeft' }}
                    />
                    <ChartTooltip content={<ChartTooltipContent />} />
                    <ChartLegend content={<ChartLegendContent />} />
                    <Bar dataKey="network" stackId="a" fill="var(--color-network)" />
                    <Bar dataKey="processing" stackId="a" fill="var(--color-processing)" />
                    <Bar dataKey="execution" stackId="a" fill="var(--color-execution)" />
                    <Bar dataKey="ik_manipulability" stackId="a" fill="var(--color-ik_manipulability)" />
                    <Bar dataKey="ik_solve" stackId="a" fill="var(--color-ik_solve)" />
                    <Bar dataKey="serial" stackId="a" fill="var(--color-serial)" />
                  </BarChart>
                </ChartContainer>
              </Card>

              {/* Recording Hz Chart */}
              <Card className="p-6">
                <h3 className="text-lg font-semibold mb-4">Frequency During Recording</h3>
                <ChartContainer
                  config={{
                    hz: {
                      label: "Frequency",
                      color: "hsl(var(--chart-5))",
                    },
                  }}
                  className="h-[300px] w-full"
                >
                  <LineChart data={recordingHzData}>
                    <CartesianGrid strokeDasharray="3 3" />
                    <XAxis
                      dataKey="progress"
                      label={{ value: 'Progress', position: 'insideBottom', offset: -5 }}
                    />
                    <YAxis
                      domain={[0, 120]}
                      label={{ value: 'Hz', angle: -90, position: 'insideLeft' }}
                    />
                    <ChartTooltip content={<ChartTooltipContent />} />
                    <ReferenceLine
                      y={100}
                      stroke="hsl(var(--muted-foreground))"
                      strokeDasharray="5 5"
                      strokeWidth={2}
                      label={{ value: 'Target (100 Hz)', position: 'right', fill: 'hsl(var(--muted-foreground))' }}
                    />
                    <Line
                      type="monotone"
                      dataKey="hz"
                      stroke="var(--color-hz)"
                      strokeWidth={2}
                      dot={false}
                      isAnimationActive={false}
                    />
                  </LineChart>
                </ChartContainer>
              </Card>

              {/* Recording Statistics */}
              <Card className="p-4">
                <h3 className="text-lg font-semibold mb-3">Recording Details</h3>
                <div className="grid grid-cols-3 gap-4 text-sm">
                  <div>
                    <div className="text-muted-foreground">Total Commands</div>
                    <div className="text-lg font-semibold">{selectedRecording.commands.length}</div>
                  </div>
                  <div>
                    <div className="text-muted-foreground">Total Duration</div>
                    <div className="text-lg font-semibold">
                      {selectedRecording.commands.reduce((sum, cmd) => sum + cmd.duration_s, 0).toFixed(2)}s
                    </div>
                  </div>
                  <div>
                    <div className="text-muted-foreground">Total Cycles</div>
                    <div className="text-lg font-semibold">
                      {selectedRecording.commands.reduce((sum, cmd) => sum + cmd.num_cycles, 0)}
                    </div>
                  </div>
                </div>
              </Card>
            </>
          )}

          {!selectedRecording && recordings.length > 0 && (
            <Card className="p-8">
              <div className="text-center text-muted-foreground">
                Select a recording to view detailed analysis
              </div>
            </Card>
          )}

          {recordings.length === 0 && !isLoadingRecordings && (
            <Card className="p-8">
              <div className="text-center text-muted-foreground">
                No recordings available. Start recording to capture performance data.
              </div>
            </Card>
          )}
        </div>
      </div>
    </main>
  );
}
