import { useEffect, useMemo, useState } from "react";

import { buildMapFileUrl, fetchMapMedia } from "../api";
import type { CameraFrame, MapMediaEntry, MapMediaListing, SavedMapInfo } from "../types";
import { formatNullable } from "../utils/format";
import { CameraPanel } from "./CameraPanel";

type MediaTab = "live" | "images" | "pointclouds";

interface MediaDockProps {
  camera: CameraFrame | null;
  selectedMap: SavedMapInfo | null;
  selectedPointcloudPath: string | null;
  onSelectPointcloudPath: (path: string | null) => void;
}

export function MediaDock({
  camera,
  selectedMap,
  selectedPointcloudPath,
  onSelectPointcloudPath,
}: MediaDockProps) {
  const [tab, setTab] = useState<MediaTab>("live");
  const [listing, setListing] = useState<MapMediaListing | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedImagePath, setSelectedImagePath] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;

    if (!selectedMap) {
      setListing(null);
      setSelectedImagePath(null);
      setError(null);
      return () => {
        cancelled = true;
      };
    }

    setLoading(true);
    setError(null);
    fetchMapMedia(selectedMap.map_id)
      .then((payload) => {
        if (cancelled) {
          return;
        }
        setListing(payload);
      })
      .catch((fetchError) => {
        if (cancelled) {
          return;
        }
        setListing(null);
        setError(fetchError instanceof Error ? fetchError.message : "媒体目录加载失败");
      })
      .finally(() => {
        if (!cancelled) {
          setLoading(false);
        }
      });

    return () => {
      cancelled = true;
    };
  }, [selectedMap]);

  const imageEntries = useMemo(
    () => (listing?.entries ?? []).filter((entry) => entry.kind === "image"),
    [listing],
  );
  const pointcloudEntries = useMemo(
    () => (listing?.entries ?? []).filter((entry) => entry.kind === "pointcloud"),
    [listing],
  );
  const otherEntries = useMemo(
    () =>
      (listing?.entries ?? []).filter((entry) => entry.kind !== "image" && entry.kind !== "pointcloud"),
    [listing],
  );

  const defaultPointcloudPath = useMemo(
    () =>
      selectedMap?.artifacts.find((artifact) => artifact.kind === "pointcloud_snapshot_3d")?.path ??
      selectedMap?.artifacts.find((artifact) => artifact.kind === "native_pointcloud_map_3d")?.path ??
      selectedMap?.artifacts.find((artifact) => artifact.kind === "pointcloud_map_3d")?.path ??
      null,
    [selectedMap],
  );

  useEffect(() => {
    if (imageEntries.length === 0) {
      setSelectedImagePath(null);
      return;
    }
    if (!selectedImagePath || !imageEntries.some((entry) => entry.path === selectedImagePath)) {
      setSelectedImagePath(imageEntries[0].path);
    }
  }, [imageEntries, selectedImagePath]);

  const selectedImage = imageEntries.find((entry) => entry.path === selectedImagePath) ?? null;
  const selectedPointcloud =
    pointcloudEntries.find((entry) => entry.path === selectedPointcloudPath) ??
    pointcloudEntries.find((entry) => entry.path === defaultPointcloudPath) ??
    null;

  return (
    <section className="media-dock-panel">
      <div className="media-dock-header">
        <div>
          <h2>媒体与历史资产</h2>
          <p>{selectedMap ? `${selectedMap.map_id} / ROS2 地图资产目录` : "未选择地图，等待地图资产"}</p>
        </div>
        <div className="media-dock-summary">
          <span>{`图片 ${imageEntries.length}`}</span>
          <span>{`点云 ${pointcloudEntries.length}`}</span>
          <span>{`其他 ${otherEntries.length}`}</span>
        </div>
      </div>

      <div className="media-tab-strip">
        <button
          type="button"
          className={`media-tab-button ${tab === "live" ? "media-tab-button-active" : ""}`}
          onClick={() => setTab("live")}
        >
          实时相机
        </button>
        <button
          type="button"
          className={`media-tab-button ${tab === "images" ? "media-tab-button-active" : ""}`}
          onClick={() => setTab("images")}
          disabled={!selectedMap}
        >
          图片目录
        </button>
        <button
          type="button"
          className={`media-tab-button ${tab === "pointclouds" ? "media-tab-button-active" : ""}`}
          onClick={() => setTab("pointclouds")}
          disabled={!selectedMap}
        >
          历史点云
        </button>
      </div>

      <div className="media-dock-body">
        {tab === "live" ? (
          <CameraPanel camera={camera} />
        ) : null}

        {tab === "images" ? (
          <div className="media-browser-grid">
            <AssetList
              title="图片目录"
              entries={imageEntries}
              selectedPath={selectedImagePath}
              loading={loading}
              error={error}
              emptyText="当前地图目录下还没有图片资产"
              onSelect={setSelectedImagePath}
              selectedMapId={selectedMap?.map_id ?? null}
              badgeResolver={(entry) => {
                if (!entry.linked_pointcloud_path) {
                  return null;
                }
                return entry.link_source === "metadata" ? "显式关联 PCD" : "推断关联 PCD";
              }}
            />
            <div className="media-preview-card">
              {selectedImage && selectedMap ? (
                <>
                  <div className="media-preview-header">
                    <div>
                      <h3>{selectedImage.name}</h3>
                      <p>{selectedImage.group ?? "root"}</p>
                    </div>
                    {selectedImage.linked_pointcloud_path ? (
                      <button
                        type="button"
                        className="secondary-button"
                        onClick={() => {
                          onSelectPointcloudPath(selectedImage.linked_pointcloud_path);
                          setTab("pointclouds");
                        }}
                      >
                        打开关联点云
                      </button>
                    ) : null}
                  </div>
                  <div className="media-image-frame">
                    <img
                      src={buildMapFileUrl(selectedMap.map_id, selectedImage.path)}
                      alt={selectedImage.name}
                    />
                  </div>
                  <div className="media-preview-meta">
                    <span>{`path=${selectedImage.path}`}</span>
                    <span>{`size=${formatFileSize(selectedImage.size_bytes)}`}</span>
                    <span>{`source=${formatNullable(selectedImage.link_source, "none")}`}</span>
                    <span>{`linked=${formatNullable(selectedImage.linked_pointcloud_path, "none")}`}</span>
                  </div>
                </>
              ) : (
                <EmptyPreview message={selectedMap ? "从左侧选择一张图片查看" : "先选择一张地图"} />
              )}
            </div>
          </div>
        ) : null}

        {tab === "pointclouds" ? (
          <div className="media-browser-grid">
            <AssetList
              title="历史点云"
              entries={pointcloudEntries}
              selectedPath={selectedPointcloudPath ?? defaultPointcloudPath}
              loading={loading}
              error={error}
              emptyText="当前地图目录下还没有可浏览的历史点云"
              onSelect={onSelectPointcloudPath}
              selectedMapId={selectedMap?.map_id ?? null}
              badgeResolver={(entry) => {
                if (entry.path === selectedPointcloudPath) {
                  return "主视图已加载";
                }
                if (!selectedPointcloudPath && entry.path === defaultPointcloudPath) {
                  return "默认点云";
                }
                if (entry.link_source === "metadata") {
                  return "显式关联";
                }
                return null;
              }}
            />
            <div className="media-preview-card media-preview-card-dark">
              <div className="media-preview-header">
                <div>
                  <h3>{selectedPointcloud?.name ?? "等待点云选择"}</h3>
                  <p>{selectedPointcloud?.group ?? "root"}</p>
                </div>
                <div className="media-pointcloud-actions">
                  <button
                    type="button"
                    className="secondary-button"
                    disabled={!defaultPointcloudPath}
                    onClick={() => onSelectPointcloudPath(null)}
                  >
                    回到默认点云
                  </button>
                </div>
              </div>
              {selectedPointcloud ? (
                <>
                  <div className="media-pointcloud-summary">
                    <StatusLine label="当前主视图" value={selectedPointcloudPath ? "历史点云" : "默认地图点云"} />
                    <StatusLine label="path" value={selectedPointcloud.path} />
                    <StatusLine label="artifact" value={formatNullable(selectedPointcloud.artifact_kind, "none")} />
                    <StatusLine label="size" value={formatFileSize(selectedPointcloud.size_bytes)} />
                    <StatusLine label="link source" value={formatNullable(selectedPointcloud.link_source, "none")} />
                    <StatusLine label="linked image" value={formatNullable(selectedPointcloud.linked_image_path, "none")} />
                  </div>
                  {selectedPointcloud.linked_image_path && selectedMap ? (
                    <div className="media-linked-image">
                      <img
                        src={buildMapFileUrl(selectedMap.map_id, selectedPointcloud.linked_image_path)}
                        alt={selectedPointcloud.linked_image_path}
                      />
                    </div>
                  ) : (
                    <EmptyPreview message="该点云暂时没有关联图片，可直接在 3D 主视图中查看。" />
                  )}
                </>
              ) : (
                <EmptyPreview message={selectedMap ? "从左侧选择一个历史点云加载到主视图" : "先选择一张地图"} />
              )}
            </div>
          </div>
        ) : null}
      </div>

      {otherEntries.length > 0 ? (
        <div className="media-dock-footer">
          <span>其他资产</span>
          {otherEntries.slice(0, 4).map((entry) => (
            <span key={entry.path} className="media-dock-footer-chip">
              {entry.name}
            </span>
          ))}
          {otherEntries.length > 4 ? <span className="media-dock-footer-chip">{`+${otherEntries.length - 4}`}</span> : null}
        </div>
      ) : null}
    </section>
  );
}

function AssetList({
  title,
  entries,
  selectedPath,
  loading,
  error,
  emptyText,
  onSelect,
  selectedMapId,
  badgeResolver,
}: {
  title: string;
  entries: MapMediaEntry[];
  selectedPath: string | null;
  loading: boolean;
  error: string | null;
  emptyText: string;
  onSelect: (path: string) => void;
  selectedMapId: string | null;
  badgeResolver: (entry: MapMediaEntry) => string | null;
}) {
  return (
    <div className="media-asset-list-card">
      <div className="media-asset-list-header">
        <h3>{title}</h3>
        <span>{selectedMapId ?? "未选地图"}</span>
      </div>
      {loading ? <div className="media-placeholder">正在读取地图目录...</div> : null}
      {error ? <div className="media-placeholder media-placeholder-error">{error}</div> : null}
      {!loading && !error && entries.length === 0 ? <div className="media-placeholder">{emptyText}</div> : null}
      <div className="media-asset-list">
        {entries.map((entry) => {
          const badge = badgeResolver(entry);
          return (
            <button
              key={entry.path}
              type="button"
              className={`media-asset-button ${selectedPath === entry.path ? "media-asset-button-active" : ""}`}
              onClick={() => onSelect(entry.path)}
            >
              <div className="media-asset-topline">
                <span>{entry.name}</span>
                {badge ? <span className="media-asset-badge">{badge}</span> : null}
              </div>
              <div className="media-asset-subline">
                <span>{entry.group ?? "root"}</span>
                <span>{formatFileSize(entry.size_bytes)}</span>
              </div>
            </button>
          );
        })}
      </div>
    </div>
  );
}

function EmptyPreview({ message }: { message: string }) {
  return <div className="media-placeholder media-placeholder-large">{message}</div>;
}

function StatusLine({ label, value }: { label: string; value: string }) {
  return (
    <div className="media-status-line">
      <span>{label}</span>
      <span>{value}</span>
    </div>
  );
}

function formatFileSize(bytes: number | null | undefined): string {
  if (bytes === null || bytes === undefined || Number.isNaN(bytes)) {
    return "未知大小";
  }
  if (bytes < 1024) {
    return `${bytes} B`;
  }
  if (bytes < 1024 * 1024) {
    return `${(bytes / 1024).toFixed(1)} KB`;
  }
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}
