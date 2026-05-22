import { useEffect, useMemo, useRef, useState } from "react";
import type { DiagnosticsSnapshot, LogEntry } from "../types";
import { fetchLogs } from "../api";

interface Props {
  diagnostics: DiagnosticsSnapshot | null;
  diagnosticsError: string | null;
  onRefresh: () => void;
}

type LogSource = "all" | "nav" | "mapping" | "localization" | "control" | "web" | "system";

const LOG_SOURCE_LABELS: Record<LogSource, string> = {
  all: "全部",
  nav: "导航",
  mapping: "建图",
  localization: "定位",
  control: "控制",
  web: "Web",
  system: "系统",
};

const SEVERITY_COLORS: Record<string, string> = {
  ok: "#22c55e",
  warn: "#eab308",
  error: "#ef4444",
  unknown: "#6b7280",
  skipped: "#9ca3af",
};

function severityDot(state: string): string {
  return SEVERITY_COLORS[state] || SEVERITY_COLORS.unknown;
}

export function DiagnosticsPanel({ diagnostics, diagnosticsError, onRefresh }: Props) {
  const [collapsed, setCollapsed] = useState(false);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [logSource, setLogSource] = useState<LogSource>("all");
  const [logLevel, setLogLevel] = useState("all");
  const [logSearch, setLogSearch] = useState("");
  const [autoScroll, setAutoScroll] = useState(true);
  const [selectedItem, setSelectedItem] = useState<string | null>(null);
  const [diagTab, setDiagTab] = useState<"navigation" | "mapping">("navigation");
  const logEndRef = useRef<HTMLDivElement>(null);
  const pollRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const apiFailed = diagnosticsError !== null && diagnostics === null;
  const summary = apiFailed
    ? {
        severity: "error",
        title: "诊断接口不可用",
        reason: diagnosticsError ?? "",
        evidence: [] as string[],
        suggestion: "检查 web_console backend 是否启动",
      }
    : (diagnostics?.summary ?? null);
  const items = diagTab === "mapping" ? (diagnostics?.mapping ?? []) : (diagnostics?.navigation ?? []);
  const sectionTitle = diagTab === "mapping" ? "建图链路检查" : "导航链路检查";

  useEffect(() => {
    const load = async () => {
      try {
        const source = logSource === "nav" ? "navigation" : logSource;
        const entries = await fetchLogs(source, 200, logLevel, logSearch);
        setLogs(entries);
      } catch {
        // backend might not be ready
      }
    };
    load();
    pollRef.current = setInterval(load, 3000);
    return () => {
      if (pollRef.current) clearInterval(pollRef.current);
    };
  }, [logSource, logLevel, logSearch]);

  useEffect(() => {
    if (autoScroll && logEndRef.current) {
      logEndRef.current.scrollIntoView({ behavior: "smooth" });
    }
  }, [logs, autoScroll]);

  useEffect(() => {
    if (diagnostics?.mode === "mapping") {
      setDiagTab("mapping");
    } else if (diagnostics?.mode === "navigation" || diagnostics?.mode === "nav") {
      setDiagTab("navigation");
    }
  }, [diagnostics?.mode]);

  const toggleItem = (key: string) => {
    setSelectedItem((prev) => (prev === key ? null : key));
  };

  const filteredLogs = useMemo(() => {
    return logs;
  }, [logs]);

  const clearLogs = () => setLogs([]);

  return (
    <div className={`diag-panel${collapsed ? " diag-collapsed" : ""}`}>
      {collapsed ? (
        <div className="diag-collapsed-bar" onClick={() => setCollapsed(false)} title="展开诊断面板">
          <span className="diag-collapsed-arrow">▶</span>
          {summary && (
            <span className="diag-collapsed-dot" style={{ background: severityDot(summary.severity) }} />
          )}
        </div>
      ) : (
        <>
          <div className="diag-header">
            <span className="diag-title">诊断面板</span>
            <button className="diag-refresh-btn" onClick={onRefresh} title="刷新">↻</button>
            <button className="diag-collapse-btn" onClick={() => setCollapsed(true)} title="折叠">◀</button>
          </div>

          {/* Summary */}
          {summary && (
            <div className="diag-summary" style={{ borderLeftColor: severityDot(summary.severity) }}>
              <div className="diag-summary-title" style={{ color: severityDot(summary.severity) }}>
                {summary.title}
              </div>
              {summary.reason && <div className="diag-summary-reason">{summary.reason}</div>}
              {summary.evidence.length > 0 && (
                <div className="diag-summary-evidence">
                  {summary.evidence.map((e, i) => (
                    <code key={i} className="diag-evidence-chip">{e}</code>
                  ))}
                </div>
              )}
              {summary.suggestion && <div className="diag-summary-suggestion">建议：{summary.suggestion}</div>}
            </div>
          )}

          {/* Checklist */}
          <div className="diag-log-tabs" style={{ margin: "0 0 4px 0" }}>
            <button
              className={`diag-log-tab${diagTab === "navigation" ? " active" : ""}`}
              onClick={() => setDiagTab("navigation")}
            >
              导航
            </button>
            <button
              className={`diag-log-tab${diagTab === "mapping" ? " active" : ""}`}
              onClick={() => setDiagTab("mapping")}
            >
              建图
            </button>
          </div>
          <div className="diag-checklist">
            <div className="diag-section-title">{sectionTitle}</div>
            {items.map((item) => (
              <div key={item.key} className="diag-item">
                <div
                  className="diag-item-header"
                  onClick={() => toggleItem(item.key)}
                  style={{ borderLeftColor: severityDot(item.state) }}
                >
                  <span className="diag-item-dot" style={{ background: severityDot(item.state) }} />
                  <span className="diag-item-label">{item.label}</span>
                  <span className="diag-item-state" style={{ color: severityDot(item.state) }}>
                    {item.state === "error" ? "异常" : item.state === "warn" ? "警告" : item.state === "ok" ? "正常" : item.state === "skipped" ? "跳过" : "未知"}
                  </span>
                  <span className="diag-item-toggle">{selectedItem === item.key ? "▼" : "▶"}</span>
                </div>
                {selectedItem === item.key && (
                  <div className="diag-item-detail">
                    {item.reason && <div className="diag-detail-row"><strong>原因：</strong>{item.reason}</div>}
                    {item.evidence.length > 0 && (
                      <div className="diag-detail-row">
                        <strong>证据：</strong>
                        {item.evidence.map((e, i) => (
                          <code key={i} className="diag-evidence-chip">{e}</code>
                        ))}
                      </div>
                    )}
                    {item.suggestion && <div className="diag-detail-row"><strong>建议：</strong>{item.suggestion}</div>}
                  </div>
                )}
              </div>
            ))}
          </div>

          {/* Log Viewer */}
          <div className="diag-log-viewer">
            <div className="diag-log-header">
              <span className="diag-section-title">日志</span>
              <div className="diag-log-actions">
                <label className="diag-auto-scroll">
                  <input type="checkbox" checked={autoScroll} onChange={(e) => setAutoScroll(e.target.checked)} />
                  自动滚动
                </label>
                <button className="diag-log-btn" onClick={clearLogs}>清空</button>
              </div>
            </div>
            <div className="diag-log-tabs">
              {(Object.keys(LOG_SOURCE_LABELS) as LogSource[]).map((src) => (
                <button
                  key={src}
                  className={`diag-log-tab${logSource === src ? " active" : ""}`}
                  onClick={() => setLogSource(src)}
                >
                  {LOG_SOURCE_LABELS[src]}
                </button>
              ))}
            </div>
            <div className="diag-log-filters">
              <select value={logLevel} onChange={(e) => setLogLevel(e.target.value)} className="diag-log-filter-select">
                <option value="all">全部级别</option>
                <option value="error">ERROR</option>
                <option value="warn">WARN</option>
                <option value="info">INFO</option>
              </select>
              <input
                type="text"
                className="diag-log-search"
                placeholder="搜索关键词..."
                value={logSearch}
                onChange={(e) => setLogSearch(e.target.value)}
              />
            </div>
            <div className="diag-log-entries">
              {filteredLogs.length === 0 ? (
                <div className="diag-log-empty">暂无日志</div>
              ) : (
                filteredLogs.map((entry, i) => (
                  <div key={i} className={`diag-log-entry diag-log-${entry.level.toLowerCase()}`}>
                    <span className="diag-log-level">{entry.level}</span>
                    <span className="diag-log-source">{entry.source}</span>
                    <span className="diag-log-msg">{entry.message}</span>
                  </div>
                ))
              )}
              <div ref={logEndRef} />
            </div>
          </div>
        </>
      )}
    </div>
  );
}
