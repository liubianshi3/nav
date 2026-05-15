import { useEffect, useRef, useState } from "react";

import type { BackendEvent } from "../types";

interface SocketHandlers {
  onEvent: (event: BackendEvent<unknown>) => void;
  onError: (message: string) => void;
}

function resolveSocketUrl(): string {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${protocol}//${window.location.host}/ws`;
}

export function useBackendSocket(handlers: SocketHandlers) {
  const handlersRef = useRef(handlers);
  const [connected, setConnected] = useState(false);
  const [lastError, setLastError] = useState<string | null>(null);

  handlersRef.current = handlers;

  useEffect(() => {
    let socket: WebSocket | null = null;
    let reconnectTimer: number | null = null;
    let closedByHook = false;

    const connect = () => {
      socket = new WebSocket(resolveSocketUrl());
      socket.onopen = () => {
        setConnected(true);
        setLastError(null);
      };
      socket.onmessage = (message) => {
        try {
          const event = JSON.parse(message.data) as BackendEvent<unknown>;
          handlersRef.current.onEvent(event);
        } catch (error) {
          const reason = error instanceof Error ? error.message : "无法解析 WebSocket 消息";
          handlersRef.current.onError(reason);
        }
      };
      socket.onerror = () => {
        setLastError("WebSocket 连接异常");
      };
      socket.onclose = () => {
        setConnected(false);
        if (!closedByHook) {
          reconnectTimer = window.setTimeout(connect, 1500);
        }
      };
    };

    connect();

    return () => {
      closedByHook = true;
      setConnected(false);
      if (reconnectTimer !== null) {
        window.clearTimeout(reconnectTimer);
      }
      socket?.close();
    };
  }, []);

  return { connected, lastError };
}
