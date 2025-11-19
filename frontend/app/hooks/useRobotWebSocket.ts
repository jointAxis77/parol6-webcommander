/**
 * WebSocket hook for PAROL6 Robot API
 * Provides real-time log streaming and robot data updates
 */

import { useEffect, useState, useRef, useCallback } from 'react';
import { getWsUrl } from '../lib/apiConfig';
import { logger } from '../lib/logger';

// Log entry interface matching backend
export interface LogEntry {
  timestamp: string;
  level: 'DEBUG' | 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
  source: string;
  message: string;
  module: string;
  function: string;
  line: number;
  details?: any;
}

// WebSocket message types
interface WSMessage {
  type: 'connected' | 'log' | 'status' | 'pose' | 'joints' | 'speeds' | 'io' | 'gripper' | 'error';
  data?: any;
  client_id?: string;
  message?: string;
  timestamp?: string;
  error?: string;
}

// Connection states
export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';

// Subscription options
export interface SubscriptionOptions {
  topics?: string[];
  rateHz?: number;
  logLevel?: 'DEBUG' | 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';
}

// Hook return type
export interface UseRobotWebSocketReturn {
  // Connection state
  connectionState: ConnectionState;
  isConnected: boolean;
  error: string | null;
  clientId: string | null;

  // Log data
  logs: LogEntry[];

  // Robot data (optional, if subscribed)
  robotData: {
    status?: any;
    pose?: any;
    joints?: any;
    speeds?: any;
    io?: any;
    gripper?: any;
  };

  // Control functions
  subscribe: (options: SubscriptionOptions) => void;
  clearLogs: () => void;
  reconnect: () => void;
}

/**
 * Custom hook for WebSocket connection to PAROL6 Robot API
 *
 * @param url - WebSocket URL (defaults to dynamic detection based on window.location)
 * @param options - Initial subscription options
 * @param autoReconnect - Whether to automatically reconnect on disconnect
 * @returns WebSocket connection state and data
 */
export function useRobotWebSocket(
  url?: string,
  options?: SubscriptionOptions,
  autoReconnect: boolean = true
): UseRobotWebSocketReturn {
  // WebSocket URL with fallback to dynamic detection
  const wsUrl = url || getWsUrl();

  // State
  const [connectionState, setConnectionState] = useState<ConnectionState>('disconnected');
  const [error, setError] = useState<string | null>(null);
  const [clientId, setClientId] = useState<string | null>(null);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [robotData, setRobotData] = useState<any>({});

  // Refs
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout>();
  const reconnectAttemptsRef = useRef(0);
  const optionsRef = useRef(options);
  const maxReconnectAttempts = 10;
  const reconnectDelayBase = 1000; // 1 second base delay

  // Update options ref when they change
  optionsRef.current = options;

  // Subscribe to topics
  const subscribe = useCallback((subscriptionOptions: SubscriptionOptions) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      const message: any = {};

      if (subscriptionOptions.topics) {
        message.subscribe = subscriptionOptions.topics;
      }

      if (subscriptionOptions.rateHz) {
        message.rate_hz = subscriptionOptions.rateHz;
      }

      if (subscriptionOptions.logLevel) {
        message.log_level = subscriptionOptions.logLevel;
      }

      wsRef.current.send(JSON.stringify(message));
    }
  }, []);

  // Clear logs
  const clearLogs = useCallback(() => {
    setLogs([]);
  }, []);

  // Connect to WebSocket
  const connect = useCallback(() => {
    // Clean up existing connection
    if (wsRef.current) {
      wsRef.current.close();
    }

    setConnectionState('connecting');
    setError(null);

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        logger.debug('Connected to PAROL6 Robot API', 'WebSocket');
        setConnectionState('connected');
        setError(null);
        reconnectAttemptsRef.current = 0;

        // Initialize logger with WebSocket connection
        logger.setWebSocket(ws);
        logger.setConnected(true);

        // Send initial subscription if provided
        const opts = optionsRef.current;
        if (opts && ws.readyState === WebSocket.OPEN) {
          const message: any = {};
          if (opts.topics) message.subscribe = opts.topics;
          if (opts.rateHz) message.rate_hz = opts.rateHz;
          if (opts.logLevel) message.log_level = opts.logLevel;
          ws.send(JSON.stringify(message));
        }
      };

      ws.onmessage = (event) => {
        try {
          const message: WSMessage = JSON.parse(event.data);

          switch (message.type) {
            case 'connected':
              // Save client ID
              if (message.client_id) {
                setClientId(message.client_id);
              }
              break;

            case 'log':
              // Add log entry with sliding window (max 1000 logs to prevent memory leak)
              if (message.data) {
                setLogs(prev => {
                  const newLogs = [...prev, message.data as LogEntry];
                  // Keep only last 1000 logs to prevent unbounded growth
                  return newLogs.length > 1000 ? newLogs.slice(-1000) : newLogs;
                });
              }
              break;

            case 'status':
              setRobotData(prev => ({ ...prev, status: message.data }));
              break;

            case 'pose':
              setRobotData(prev => ({ ...prev, pose: message.data }));
              break;

            case 'joints':
              setRobotData(prev => ({ ...prev, joints: message.data }));
              break;

            case 'speeds':
              setRobotData(prev => ({ ...prev, speeds: message.data }));
              break;

            case 'io':
              setRobotData(prev => ({ ...prev, io: message.data }));
              break;

            case 'gripper':
              setRobotData(prev => ({ ...prev, gripper: message.data }));
              break;

            case 'error':
              setError(message.error || 'Unknown WebSocket error');
              break;
          }
        } catch (err) {
          logger.error('Message parsing error', 'WebSocket', err);
        }
      };

      ws.onerror = (event) => {
        logger.error('WebSocket error', 'WebSocket', event);
        setError('WebSocket connection error');
        setConnectionState('error');
        logger.setConnected(false);
      };

      ws.onclose = (event) => {
        logger.debug(`Disconnected: ${event.code} - ${event.reason}`, 'WebSocket');
        setConnectionState('disconnected');
        wsRef.current = null;

        // Disconnect logger
        logger.setWebSocket(null);
        logger.setConnected(false);

        // Auto-reconnect if enabled
        if (autoReconnect && reconnectAttemptsRef.current < maxReconnectAttempts) {
          const delay = Math.min(
            reconnectDelayBase * Math.pow(2, reconnectAttemptsRef.current),
            30000 // Max 30 seconds
          );

          logger.debug(`Reconnecting in ${delay}ms (attempt ${reconnectAttemptsRef.current + 1}/${maxReconnectAttempts})`, 'WebSocket');

          reconnectTimeoutRef.current = setTimeout(() => {
            reconnectAttemptsRef.current += 1;
            connect();
          }, delay);
        } else if (reconnectAttemptsRef.current >= maxReconnectAttempts) {
          setError('Max reconnection attempts reached');
        }
      };
    } catch (err) {
      logger.error('Connection error', 'WebSocket', err);
      setError('Failed to create WebSocket connection');
      setConnectionState('error');
    }
  }, [wsUrl, autoReconnect]);

  // Manual reconnect
  const reconnect = useCallback(() => {
    reconnectAttemptsRef.current = 0;
    connect();
  }, [connect]);

  // Initialize connection
  useEffect(() => {
    connect();

    // Cleanup
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }

      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, [connect]);

  return {
    connectionState,
    isConnected: connectionState === 'connected',
    error,
    clientId,
    logs,
    robotData,
    subscribe,
    clearLogs,
    reconnect
  };
}
