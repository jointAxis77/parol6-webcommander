/**
 * Frontend logger that sends logs to backend via WebSocket
 * Falls back to console.* when WebSocket is not connected
 */

type LogLevel = 'DEBUG' | 'INFO' | 'WARNING' | 'ERROR' | 'CRITICAL';

interface LogMessage {
  type: 'frontend_log';
  level: LogLevel;
  message: string;
  source: string;
  timestamp?: string;
  details?: any;
}

class FrontendLogger {
  private wsInstance: WebSocket | null = null;
  private isConnected: boolean = false;
  private logQueue: LogMessage[] = [];
  private maxQueueSize: number = 100;

  /**
   * Set the WebSocket instance for sending logs
   * Should be called when WebSocket connects
   */
  setWebSocket(ws: WebSocket | null) {
    this.wsInstance = ws;
    this.isConnected = ws !== null && ws.readyState === WebSocket.OPEN;

    // Flush queued logs when connection is established
    if (this.isConnected && this.logQueue.length > 0) {
      const queuedLogs = [...this.logQueue];
      this.logQueue = [];
      queuedLogs.forEach(log => this.sendToBackend(log));
    }
  }

  /**
   * Update connection state
   * Should be called when WebSocket state changes
   */
  setConnected(connected: boolean) {
    this.isConnected = connected;
  }

  /**
   * Send log message to backend via WebSocket
   */
  private sendToBackend(logMessage: LogMessage) {
    if (this.isConnected && this.wsInstance && this.wsInstance.readyState === WebSocket.OPEN) {
      try {
        this.wsInstance.send(JSON.stringify(logMessage));
      } catch (error) {
        // If sending fails, fall back to console
        console.error('[Logger] Failed to send log to backend:', error);
        this.fallbackToConsole(logMessage);
      }
    } else {
      // Queue log if not connected with FIFO eviction to prevent unbounded growth
      if (this.logQueue.length >= this.maxQueueSize) {
        this.logQueue.shift(); // Remove oldest log when queue is full
      }
      this.logQueue.push(logMessage);
      // Always fall back to console
      this.fallbackToConsole(logMessage);
    }
  }

  /**
   * Fall back to browser console
   */
  private fallbackToConsole(logMessage: LogMessage) {
    const prefix = `[${logMessage.source}]`;
    const message = `${prefix} ${logMessage.message}`;

    switch (logMessage.level) {
      case 'DEBUG':
        console.debug(message, logMessage.details);
        break;
      case 'INFO':
        console.log(message, logMessage.details);
        break;
      case 'WARNING':
        console.warn(message, logMessage.details);
        break;
      case 'ERROR':
      case 'CRITICAL':
        console.error(message, logMessage.details);
        break;
    }
  }

  /**
   * Create log message with automatic source detection
   */
  private createLogMessage(
    level: LogLevel,
    message: string,
    source?: string,
    details?: any
  ): LogMessage {
    // Try to extract source from Error stack if not provided
    let logSource = source || 'frontend';
    if (!source) {
      try {
        const stack = new Error().stack;
        if (stack) {
          // Extract filename from stack trace
          const match = stack.match(/at.*\((.+):(\d+):(\d+)\)/);
          if (match) {
            const fullPath = match[1];
            const filename = fullPath.split('/').pop()?.replace('.tsx', '').replace('.ts', '');
            if (filename) {
              logSource = filename;
            }
          }
        }
      } catch (e) {
        // Ignore errors in source detection
      }
    }

    return {
      type: 'frontend_log',
      level,
      message,
      source: logSource,
      timestamp: new Date().toISOString(),
      details
    };
  }

  /**
   * Log at DEBUG level
   */
  debug(message: string, source?: string, details?: any) {
    const logMessage = this.createLogMessage('DEBUG', message, source, details);
    this.sendToBackend(logMessage);
  }

  /**
   * Log at INFO level
   */
  info(message: string, source?: string, details?: any) {
    const logMessage = this.createLogMessage('INFO', message, source, details);
    this.sendToBackend(logMessage);
  }

  /**
   * Log at WARNING level
   */
  warn(message: string, source?: string, details?: any) {
    const logMessage = this.createLogMessage('WARNING', message, source, details);
    this.sendToBackend(logMessage);
  }

  /**
   * Log at ERROR level
   */
  error(message: string, source?: string, details?: any) {
    const logMessage = this.createLogMessage('ERROR', message, source, details);
    this.sendToBackend(logMessage);
  }

  /**
   * Log at CRITICAL level
   */
  critical(message: string, source?: string, details?: any) {
    const logMessage = this.createLogMessage('CRITICAL', message, source, details);
    this.sendToBackend(logMessage);
  }

  /**
   * Clear queued logs
   */
  clearQueue() {
    this.logQueue = [];
  }

  /**
   * Get current queue size
   */
  getQueueSize(): number {
    return this.logQueue.length;
  }
}

// Export singleton instance
export const logger = new FrontendLogger();

// Export type for external use
export type { LogLevel };
