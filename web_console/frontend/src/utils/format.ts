import type { TextStatus } from "../types";

export function formatNullable(value: unknown, fallback = "暂无数据"): string {
  if (value === null || value === undefined || value === "") {
    return fallback;
  }
  return String(value);
}

export function formatNumber(value: number | null | undefined, digits = 3): string {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "暂无数据";
  }
  return value.toFixed(digits);
}

export function formatStatusSummary(status: TextStatus | null | undefined): string {
  if (!status || !status.raw) {
    return "暂无数据";
  }
  const state = status.state ?? "unknown";
  const reason = status.reason ? ` / ${status.reason}` : "";
  return `${state}${reason}`;
}
