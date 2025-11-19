'use client';

import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from '@/components/ui/dialog';
import { Button } from '@/components/ui/button';

interface Tool {
  id: string;
  name: string;
  description: string;
  tcp_offset: {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
  };
}

interface ToolMountDialogProps {
  open: boolean;
  onOpenChange: (open: boolean) => void;
  currentTool: Tool | null;
  newTool: Tool | null;
  onConfirm: () => void;
}

export default function ToolMountDialog({
  open,
  onOpenChange,
  currentTool,
  newTool,
  onConfirm,
}: ToolMountDialogProps) {
  if (!newTool) return null;

  return (
    <Dialog open={open} onOpenChange={onOpenChange}>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Mount &quot;{newTool.name}&quot;?</DialogTitle>
          <DialogDescription>
            This will update the robot&apos;s TCP offset calculations.
          </DialogDescription>
        </DialogHeader>

        <div className="space-y-4 py-4">
          {currentTool && (
            <div>
              <p className="text-sm font-medium mb-1">Current: {currentTool.name}</p>
              <p className="text-xs text-muted-foreground">
                TCP: ({currentTool.tcp_offset.x.toFixed(1)}, {currentTool.tcp_offset.y.toFixed(1)}, {currentTool.tcp_offset.z.toFixed(1)}) mm
              </p>
              <p className="text-xs text-muted-foreground">
                Orientation: ({currentTool.tcp_offset.rx.toFixed(1)}, {currentTool.tcp_offset.ry.toFixed(1)}, {currentTool.tcp_offset.rz.toFixed(1)})°
              </p>
            </div>
          )}

          <div>
            <p className="text-sm font-medium mb-1">New: {newTool.name}</p>
            <p className="text-xs text-muted-foreground">
              TCP: ({newTool.tcp_offset.x.toFixed(1)}, {newTool.tcp_offset.y.toFixed(1)}, {newTool.tcp_offset.z.toFixed(1)}) mm
            </p>
            <p className="text-xs text-muted-foreground">
              Orientation: ({newTool.tcp_offset.rx.toFixed(1)}, {newTool.tcp_offset.ry.toFixed(1)}, {newTool.tcp_offset.rz.toFixed(1)})°
            </p>
          </div>
        </div>

        <DialogFooter>
          <Button variant="outline" onClick={() => onOpenChange(false)}>
            Cancel
          </Button>
          <Button onClick={onConfirm}>Mount Tool</Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}
