import { useMemo, useState } from "react";
import { debugGetLightStatus, debugSetLight } from "../api";
import type { LightStatusPayload } from "../types";
import { formatNullable } from "../utils/format";
import { StatusMini } from "./ControlSidebar";

function clampInt(value: string, min: number, max: number): number {
  const parsed = Number.parseInt(value, 10);
  if (!Number.isFinite(parsed)) return min;
  return Math.max(min, Math.min(max, parsed));
}

function colorModeLabel(value: number): string {
  if (value === 1) return "WHITE";
  if (value === 2) return "RGB";
  if (value === 3) return "CCT";
  return "UNSPECIFIED";
}

export function LightDebugPanel() {
  const [deviceId, setDeviceId] = useState("a2");
  const [on, setOn] = useState(true);
  const [intensity, setIntensity] = useState("128");
  const [colorMode, setColorMode] = useState("2");
  const [r, setR] = useState("255");
  const [g, setG] = useState("255");
  const [b, setB] = useState("255");
  const [cct, setCct] = useState("4000");
  const [busy, setBusy] = useState(false);
  const [lastStatus, setLastStatus] = useState<LightStatusPayload | null>(null);
  const [lastError, setLastError] = useState<string | null>(null);
  const [lastMessage, setLastMessage] = useState<string | null>(null);

  const preview = useMemo(() => {
    const safeR = clampInt(r, 0, 255);
    const safeG = clampInt(g, 0, 255);
    const safeB = clampInt(b, 0, 255);
    return `rgb(${safeR}, ${safeG}, ${safeB})`;
  }, [r, g, b]);

  return (
    <section className="panel">
      <h2>灯光调试</h2>
      <label className="form-label" htmlFor="light-device-id">
        device_id
      </label>
      <input
        id="light-device-id"
        className="text-input"
        value={deviceId}
        onChange={(event) => setDeviceId(event.target.value)}
        placeholder="a2"
      />
      <div className="button-group">
        <label className="form-label" htmlFor="light-on">
          on
        </label>
        <input id="light-on" type="checkbox" checked={on} onChange={(event) => setOn(event.target.checked)} />
      </div>
      <label className="form-label" htmlFor="light-intensity">
        intensity (0-255)
      </label>
      <input
        id="light-intensity"
        className="text-input"
        inputMode="numeric"
        value={intensity}
        onChange={(event) => setIntensity(event.target.value)}
        placeholder="128"
      />
      <label className="form-label" htmlFor="light-color-mode">
        color_mode
      </label>
      <select
        id="light-color-mode"
        className="select-input"
        value={colorMode}
        onChange={(event) => setColorMode(event.target.value)}
      >
        <option value="0">0 - UNSPECIFIED</option>
        <option value="1">1 - WHITE</option>
        <option value="2">2 - RGB</option>
        <option value="3">3 - CCT</option>
      </select>
      <div className="button-group">
        <div style={{ flex: 1 }}>
          <label className="form-label" htmlFor="light-r">
            rgb.r
          </label>
          <input
            id="light-r"
            className="text-input"
            inputMode="numeric"
            value={r}
            onChange={(event) => setR(event.target.value)}
            placeholder="255"
          />
        </div>
        <div style={{ flex: 1 }}>
          <label className="form-label" htmlFor="light-g">
            rgb.g
          </label>
          <input
            id="light-g"
            className="text-input"
            inputMode="numeric"
            value={g}
            onChange={(event) => setG(event.target.value)}
            placeholder="255"
          />
        </div>
        <div style={{ flex: 1 }}>
          <label className="form-label" htmlFor="light-b">
            rgb.b
          </label>
          <input
            id="light-b"
            className="text-input"
            inputMode="numeric"
            value={b}
            onChange={(event) => setB(event.target.value)}
            placeholder="255"
          />
        </div>
      </div>
      <div className="button-group">
        <div style={{ flex: 1 }}>
          <label className="form-label" htmlFor="light-cct">
            cct kelvin (0-65535)
          </label>
          <input
            id="light-cct"
            className="text-input"
            inputMode="numeric"
            value={cct}
            onChange={(event) => setCct(event.target.value)}
            placeholder="4000"
          />
        </div>
        <div style={{ width: 54, height: 54, borderRadius: 8, border: "1px solid rgba(255,255,255,0.12)", background: preview }} />
      </div>
      <div className="button-group">
        <button
          type="button"
          className="primary-button"
          disabled={busy}
          onClick={() => {
            setBusy(true);
            setLastError(null);
            setLastMessage(null);
            void (async () => {
              try {
                const response = await debugSetLight({
                  device_id: deviceId.trim() || "a2",
                  on,
                  intensity: clampInt(intensity, 0, 255),
                  color_mode: clampInt(colorMode, 0, 255),
                  rgb: { r: clampInt(r, 0, 255), g: clampInt(g, 0, 255), b: clampInt(b, 0, 255) },
                  color_temperature_kelvin: clampInt(cct, 0, 65535),
                });
                setLastMessage(`${response.success ? "✓" : "✗"} ${response.message || "ok"} (${colorModeLabel(response.status.color_mode)})`);
                setLastStatus(response.status);
              } catch (error) {
                setLastError(error instanceof Error ? error.message : "SetLight 调用失败");
              } finally {
                setBusy(false);
              }
            })();
          }}
        >
          发送 SetLight
        </button>
        <button
          type="button"
          className="secondary-button"
          disabled={busy}
          onClick={() => {
            setBusy(true);
            setLastError(null);
            setLastMessage(null);
            void (async () => {
              try {
                const status = await debugGetLightStatus(deviceId.trim() || "a2");
                setLastStatus(status);
                setLastMessage(`已读取 GetLightStatus (${colorModeLabel(status.color_mode)})`);
              } catch (error) {
                setLastError(error instanceof Error ? error.message : "GetLightStatus 调用失败");
              } finally {
                setBusy(false);
              }
            })();
          }}
        >
          读取 GetLightStatus
        </button>
      </div>
      <p className="panel-message">{formatNullable(lastMessage, "使用 gRPC 接口下发灯光命令，并读取本地缓存状态")}</p>
      {lastError ? <p className="notice notice-error">{lastError}</p> : null}
      {lastStatus ? (
        <div className="map-asset-card">
          <StatusMini label="device_id" value={lastStatus.device_id} />
          <StatusMini label="on" value={String(lastStatus.on)} />
          <StatusMini label="intensity" value={String(lastStatus.intensity)} />
          <StatusMini label="color_mode" value={`${lastStatus.color_mode} (${colorModeLabel(lastStatus.color_mode)})`} />
          <StatusMini label="rgb" value={`${lastStatus.rgb.r}, ${lastStatus.rgb.g}, ${lastStatus.rgb.b}`} />
          <StatusMini label="cct" value={String(lastStatus.color_temperature_kelvin)} />
          <StatusMini label="timestamp" value={String(lastStatus.timestamp)} />
        </div>
      ) : null}
    </section>
  );
}

