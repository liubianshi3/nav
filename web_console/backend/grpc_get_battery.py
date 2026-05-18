from __future__ import annotations

import argparse
import asyncio
import json

import grpc

from .grpc_codegen import ensure_grpc_generated


async def _main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", default="127.0.0.1:50051")
    parser.add_argument("--device-id", default="a2")
    args = parser.parse_args()

    ensure_grpc_generated()
    from device import robot_dog_pb2, robot_dog_pb2_grpc

    try:
        async with grpc.aio.insecure_channel(str(args.target)) as channel:
            stub = robot_dog_pb2_grpc.RobotDogServiceStub(channel)
            response = await stub.GetBattery(robot_dog_pb2.BatteryRequest(device_id=str(args.device_id)))
    except grpc.aio.AioRpcError as exc:
        payload = {
            "ok": False,
            "code": exc.code().name if exc.code() is not None else "UNKNOWN",
            "details": str(exc.details() or ""),
        }
        print(json.dumps(payload, ensure_ascii=False))
        return 2
    payload = {
        "percentage": int(getattr(response, "percentage", 0) or 0),
        "is_charging": bool(getattr(response, "is_charging", False)),
        "estimated_minutes": int(getattr(response, "estimated_minutes", 0) or 0),
        "health": int(getattr(response, "health", 0) or 0),
    }
    payload["ok"] = payload["percentage"] >= 0
    print(json.dumps(payload, ensure_ascii=False))
    return 0


def main() -> None:
    raise SystemExit(asyncio.run(_main()))


if __name__ == "__main__":
    main()
