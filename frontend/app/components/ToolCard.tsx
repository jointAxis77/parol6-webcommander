'use client';

import { Badge } from '@/components/ui/badge';
import { Button } from '@/components/ui/button';
import { cn } from '@/lib/utils';
import { Pencil, Trash2 } from 'lucide-react';

interface Tool {
  id: string;
  name: string;
  description: string;
  mesh_file: string | null;
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

interface ToolCardProps {
  tool: Tool;
  isMounted: boolean;
  isSelected: boolean;
  onSelect: () => void;
  onDelete: () => void;
}

export default function ToolCard({
  tool,
  isMounted,
  isSelected,
  onSelect,
  onDelete,
}: ToolCardProps) {
  return (
    <div
      className={cn(
        'p-2 rounded-md border cursor-pointer transition-colors hover:bg-accent',
        isSelected && 'border-primary bg-accent',
        isMounted && 'border-green-500 bg-green-500/5'
      )}
      onClick={onSelect}
    >
      <div className="flex items-center justify-between gap-2">
        <div className="flex-1 min-w-0 flex items-center gap-2">
          <h3 className="font-medium text-sm truncate">{tool.name}</h3>
          {isMounted && (
            <Badge variant="default" className="text-[10px] h-4 px-1.5 bg-green-600 shrink-0">
              Mounted
            </Badge>
          )}
        </div>
        <div className="flex items-center gap-1 shrink-0">
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7"
            onClick={(e) => {
              e.stopPropagation();
              onSelect();
            }}
          >
            <Pencil className="h-3.5 w-3.5" />
          </Button>
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7 text-destructive hover:text-destructive"
            onClick={(e) => {
              e.stopPropagation();
              onDelete();
            }}
            disabled={isMounted}
          >
            <Trash2 className="h-3.5 w-3.5" />
          </Button>
        </div>
      </div>
      {tool.description && (
        <p className="text-xs text-muted-foreground mt-1 line-clamp-1">
          {tool.description}
        </p>
      )}
    </div>
  );
}
