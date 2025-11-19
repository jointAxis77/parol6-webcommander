/**
 * Dynamic API configuration based on window.location
 *
 * This module provides utility functions to construct API URLs dynamically
 * based on the hostname that the frontend is accessed from.
 *
 * Example:
 * - Frontend at http://192.168.1.100:3000 -> API at http://192.168.1.100:3001
 * - Frontend at http://parol6.local:3000 -> API at http://parol6.local:3001
 */

const API_PORT = 3001;

/**
 * Get the base API URL dynamically based on window.location
 *
 * @returns The HTTP API base URL (e.g., "http://192.168.1.100:3001")
 */
export function getApiBaseUrl(): string {
  // Server-side rendering or build time - return default fallback
  if (typeof window === 'undefined') {
    return 'http://parol6.local:3001';
  }

  // Dynamic URL construction from current location
  const { protocol, hostname } = window.location;

  return `${protocol}//${hostname}:${API_PORT}`;
}

/**
 * Get the WebSocket URL dynamically based on window.location
 *
 * @returns The WebSocket URL (e.g., "ws://192.168.1.100:3001/ws")
 */
export function getWsUrl(): string {
  // Server-side rendering or build time - return default fallback
  if (typeof window === 'undefined') {
    return 'ws://parol6.local:3001/ws';
  }

  // Dynamic WebSocket URL construction
  const { protocol, hostname } = window.location;
  const wsProtocol = protocol === 'https:' ? 'wss:' : 'ws:';

  return `${wsProtocol}//${hostname}:${API_PORT}/ws`;
}
