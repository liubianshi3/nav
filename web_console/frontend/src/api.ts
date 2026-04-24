import type {
  DashboardSnapshot,
  InitialPoseResult,
  NavigationGoal,
  NavigationTaskState,
  SavedMapInfo,
  StackStatus,
  SystemHealth,
} from "./types";

async function handleJson<T>(response: Response): Promise<T> {
  if (!response.ok) {
    const payload = await response.json().catch(() => ({ detail: response.statusText }));
    throw new Error(payload.detail ?? response.statusText);
  }
  return response.json() as Promise<T>;
}

export async function fetchSnapshot(): Promise<DashboardSnapshot> {
  return handleJson<DashboardSnapshot>(await fetch("/api/snapshot"));
}

export async function fetchHealth(): Promise<SystemHealth> {
  return handleJson<SystemHealth>(await fetch("/api/health"));
}

export async function sendNavigationGoal(goal: NavigationGoal): Promise<NavigationTaskState> {
  const response = await fetch("/api/navigation/goal", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ goal }),
  });
  const payload = await handleJson<{ ok: boolean; navigation: NavigationTaskState }>(response);
  return payload.navigation;
}

export async function cancelNavigationGoal(): Promise<NavigationTaskState> {
  const response = await fetch("/api/navigation/cancel", {
    method: "POST",
  });
  const payload = await handleJson<{ ok: boolean; navigation: NavigationTaskState }>(response);
  return payload.navigation;
}

export async function sendInitialPose(goal: NavigationGoal): Promise<InitialPoseResult> {
  const response = await fetch("/api/localization/initialpose", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ pose: goal }),
  });
  return handleJson<InitialPoseResult & { ok: boolean }>(response);
}

export async function fetchStackStatus(): Promise<StackStatus> {
  return handleJson<StackStatus>(await fetch("/api/stack/status"));
}

export async function startMappingStack(): Promise<StackStatus> {
  const response = await fetch("/api/stack/start-mapping", { method: "POST" });
  const payload = await handleJson<{ ok: boolean; message: string; stack: StackStatus }>(response);
  return payload.stack;
}

export async function startNavigationStack(mapId: string): Promise<StackStatus> {
  const response = await fetch("/api/stack/start-navigation", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ map_id: mapId }),
  });
  const payload = await handleJson<{ ok: boolean; message: string; stack: StackStatus }>(response);
  return payload.stack;
}

export async function stopStack(): Promise<StackStatus> {
  const response = await fetch("/api/stack/stop", { method: "POST" });
  const payload = await handleJson<{ ok: boolean; message: string; stack: StackStatus }>(response);
  return payload.stack;
}

export async function fetchMaps(): Promise<SavedMapInfo[]> {
  const payload = await handleJson<{ maps: SavedMapInfo[] }>(await fetch("/api/maps"));
  return payload.maps;
}

export async function saveCurrentMap(mapId: string): Promise<{ map: SavedMapInfo; maps: SavedMapInfo[] }> {
  const response = await fetch("/api/maps/save", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ map_id: mapId }),
  });
  return handleJson<{ ok: boolean; map: SavedMapInfo; maps: SavedMapInfo[] }>(response);
}
