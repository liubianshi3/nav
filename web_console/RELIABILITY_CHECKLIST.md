# Web Console Reliability Checklist

Run this checklist before using the web console for robot motion.

## Backend Contract

- `GET /api/health` returns 200.
- `GET /api/snapshot` includes `map`, `pose`, `status`, `navigation`, `camera`, and `health`.
- WebSocket `/ws` sends an initial `snapshot` event.
- WebSocket reconnect works after backend restart.
- `camera_received=false` is displayed as a warning, not a frontend crash.

## Map And Localization Gates

- If `/map` is unavailable, Send Navigation is disabled.
- If `/a2/localization_ok=false`, Send Navigation is disabled.
- If `/navigate_to_pose` action server is unavailable, Send Navigation is disabled.
- If AMCL pose is stale, Send Navigation is disabled.

## Navigation Controls

- Clicking map selects a goal but does not immediately move the robot.
- Send Navigation requires explicit button press.
- Stop Navigation calls action cancel and shows success or error.
- A second goal is rejected while a goal is active.

## Camera Panel

- Prefer `/camera/image_raw/compressed`.
- If only `/camera/image_raw` exists, backend requires Pillow to convert raw frames to JPEG.
- If the camera topic is missing, the panel must show `暂无相机图像`.
- Camera stream is throttled by `camera.max_broadcast_hz`.

## Failure Injection

Use mock or manual topic changes to verify:

- backend offline
- WebSocket disconnected
- map missing
- localization false
- action server unavailable
- stop navigation rejected
- camera topic missing
