/**
 * Tool Management Utilities
 *
 * Functions for loading, managing, and switching tools.
 */

import { Tool } from './types';

/**
 * Load tools from config object
 *
 * @param config - Configuration object from config.yaml
 * @returns Array of Tool objects
 */
export function loadToolsFromConfig(config: any): Tool[] {
  if (!config?.ui?.tools || !Array.isArray(config.ui.tools)) {
    return [];
  }

  return config.ui.tools.map((t: any) => ({
    id: t.id,
    name: t.name,
    description: t.description || '',
    mesh_file: t.mesh_file || null,
    mesh_units: t.mesh_units || 'mm',
    mesh_offset: t.mesh_offset || {
      x: 0,
      y: 0,
      z: 0,
      rx: 0,
      ry: 0,
      rz: 0
    },
    tcp_offset: t.tcp_offset || {
      x: 0,
      y: 0,
      z: 0,
      rx: 0,
      ry: 0,
      rz: 0
    },
    gripper_config: t.gripper_config
  }));
}

/**
 * Get default tool (first in list or fallback)
 *
 * @param tools - Array of available tools
 * @returns Default tool
 */
export function getDefaultTool(tools: Tool[]): Tool {
  if (tools.length > 0) {
    return tools[0];
  }

  // Fallback: J6 flange with no offset
  return {
    id: 'default_j6_tip',
    name: 'Default (J6 Tip)',
    description: 'No tool attached - TCP at J6 flange tip',
    mesh_file: null,
    mesh_units: 'mm',
    mesh_offset: {
      x: 0,
      y: 0,
      z: 0,
      rx: 0,
      ry: 0,
      rz: 0
    },
    tcp_offset: {
      x: 0,
      y: 0,
      z: 0,
      rx: 0,
      ry: 0,
      rz: 0
    }
  };
}

/**
 * Find tool by ID
 *
 * @param tools - Array of available tools
 * @param toolId - Tool ID to find
 * @returns Tool object or null if not found
 */
export function findToolById(tools: Tool[], toolId: string): Tool | null {
  return tools.find(t => t.id === toolId) || null;
}

/**
 * Get active tool ID from config
 *
 * @param config - Configuration object from config.yaml
 * @returns Active tool ID or null
 */
export function getActiveToolId(config: any): string | null {
  return config?.ui?.active_tool || null;
}
