import type {
  DashboardSnapshot,
  InitialPoseRequestPayload,
  InitialPoseResult,
  LightStatusPayload,
  MapMediaListing,
  NavigationGoal,
  NavigationTaskState,
  SavedMapInfo,
  SetLightDebugResponse,
  SetLightRequestPayload,
  StackStatus,
  SystemHealth,
  TaskRouteDetail,
  TaskRouteListing,
  TaskRouteRunRequestPayload,
  TaskRouteStatus,
  VirtualObstacleListing,
  VirtualObstacleUpsertPayload,
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

export async function sendNavigationGoal(goal: NavigationGoal, mapId?: string | null): Promise<NavigationTaskState> {
  const response = await fetch("/api/navigation/goal", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ goal, map_id: mapId ?? null }),
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

export async function sendInitialPose(payload: InitialPoseRequestPayload): Promise<InitialPoseResult> {
  const response = await fetch("/api/localization/initialpose", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
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

export async function startNavigationStack(
  mapId: string,
  options?: {
    localization_mode?: string;
    motion_mode?: string;
    enable_nav2_3d?: boolean;
    collision_monitor_profile?: string;
  },
): Promise<StackStatus> {
  const response = await fetch("/api/stack/start-navigation", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      map_id: mapId,
      localization_mode: options?.localization_mode ?? "ndt",
      motion_mode: options?.motion_mode ?? "dry_run",
      enable_nav2_3d: options?.enable_nav2_3d ?? true,
      collision_monitor_profile: options?.collision_monitor_profile ?? "strict",
    }),
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

export async function fetchMapMedia(mapId: string): Promise<MapMediaListing> {
  return handleJson<MapMediaListing>(await fetch(`/api/maps/${encodeURIComponent(mapId)}/media`));
}

export async function fetchMapObstacles(mapId: string): Promise<VirtualObstacleListing> {
  return handleJson<VirtualObstacleListing>(await fetch(`/api/maps/${encodeURIComponent(mapId)}/obstacles`));
}

export async function saveMapObstacle(mapId: string, payload: VirtualObstacleUpsertPayload): Promise<VirtualObstacleListing> {
  const response = await fetch(`/api/maps/${encodeURIComponent(mapId)}/obstacles`, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });
  return handleJson<VirtualObstacleListing>(response);
}

export async function deleteMapObstacle(mapId: string, obstacleId: string): Promise<VirtualObstacleListing> {
  const response = await fetch(
    `/api/maps/${encodeURIComponent(mapId)}/obstacles/${encodeURIComponent(obstacleId)}`,
    { method: "DELETE" },
  );
  return handleJson<VirtualObstacleListing>(response);
}

export function buildMapFileUrl(mapId: string, relativePath: string): string {
  const encodedPath = relativePath
    .split("/")
    .filter((segment) => segment.length > 0)
    .map((segment) => encodeURIComponent(segment))
    .join("/");
  return `/api/maps/${encodeURIComponent(mapId)}/files/${encodedPath}`;
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

export async function projectPcdTo2d(mapId: string): Promise<{ ok: boolean; map_id: string; map_yaml: string; navigation_ready: boolean; stdout: string }> {
  const response = await fetch("/api/maps/project-2d", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ map_id: mapId }),
  });
  return handleJson<{ ok: boolean; map_id: string; map_yaml: string; navigation_ready: boolean; stdout: string }>(response);
}

export async function fetchTaskRoutes(): Promise<TaskRouteListing> {
  return handleJson<TaskRouteListing>(await fetch("/api/tasks/routes"));
}

export async function fetchTaskRoute(routeId: string): Promise<TaskRouteDetail> {
  return handleJson<TaskRouteDetail>(await fetch(`/api/tasks/routes/${encodeURIComponent(routeId)}`));
}

export async function saveTaskRoute(routeId: string, routeYaml: string, mapId?: string | null): Promise<TaskRouteDetail> {
  const response = await fetch("/api/tasks/routes", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ route_id: routeId, route_yaml: routeYaml, map_id: mapId ?? null }),
  });
  return handleJson<TaskRouteDetail>(response);
}

export async function deleteTaskRoute(routeId: string): Promise<{ ok: boolean; items: string[]; status: TaskRouteStatus }> {
  const response = await fetch(`/api/tasks/routes/${encodeURIComponent(routeId)}`, { method: "DELETE" });
  return handleJson<{ ok: boolean; items: string[]; status: TaskRouteStatus }>(response);
}

export async function runTaskRoute(payload: TaskRouteRunRequestPayload): Promise<TaskRouteStatus> {
  const response = await fetch("/api/tasks/routes/run", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });
  return handleJson<TaskRouteStatus>(response);
}

export async function stopTaskRoute(): Promise<TaskRouteStatus> {
  const response = await fetch("/api/tasks/routes/stop", { method: "POST" });
  return handleJson<TaskRouteStatus>(response);
}

export async function fetchTaskRouteStatus(): Promise<TaskRouteStatus> {
  return handleJson<TaskRouteStatus>(await fetch("/api/tasks/routes/status"));
}

export async function debugSetLight(payload: SetLightRequestPayload): Promise<SetLightDebugResponse> {
  const response = await fetch("/api/debug/light/set", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });
  return handleJson<SetLightDebugResponse>(response);
}

export async function debugGetLightStatus(deviceId: string): Promise<LightStatusPayload> {
  return handleJson<LightStatusPayload>(await fetch(`/api/debug/light/status?device_id=${encodeURIComponent(deviceId)}`));
}
