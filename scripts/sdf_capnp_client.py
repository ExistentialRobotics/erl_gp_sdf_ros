#!/usr/bin/env python3
"""Cap'n Proto client for the SDF / occupancy query interface exposed by
sdf_mapping_node.

The server is launched by sdf_mapping_node when ``capnp_rpc.enabled: true`` is
set in its YAML configuration. This script connects to ``host:port`` and runs
a few example SDF and occupancy queries.

Example:
    python3 sdf_capnp_client.py --host 127.0.0.1 --port 51111 \
        --points "0,0,0; 0.5,0.0,0.1; 1.0,0.0,0.1"

Requires ``pycapnp`` (``pip install pycapnp``).
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import capnp  # type: ignore

# pycapnp requires this so the threaded reactor can run in non-main threads,
# which is what TwoPartyClient uses internally.
capnp.remove_event_loop(True)
capnp.create_event_loop(threaded=True)


def _find_schema(explicit: str | None) -> Path:
    """Locate sdf_query.capnp on disk.

    Order of preference:
    1. ``--schema`` CLI argument.
    2. ``ERL_GP_SDF_ROS_CAPNP_SCHEMA`` env var.
    3. ``share/erl_gp_sdf_ros/capnp/sdf_query.capnp`` under each entry of
       ``AMENT_PREFIX_PATH`` (so a sourced ROS2 install just works).
    4. ``../capnp/sdf_query.capnp`` relative to this script (works in the
       source tree).
    """
    if explicit:
        return Path(explicit).resolve()
    env = os.environ.get("ERL_GP_SDF_ROS_CAPNP_SCHEMA")
    if env:
        return Path(env).resolve()
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        candidate = Path(prefix) / "share" / "erl_gp_sdf_ros" / "capnp" / "sdf_query.capnp"
        if candidate.is_file():
            return candidate.resolve()
    fallback = Path(__file__).resolve().parent.parent / "capnp" / "sdf_query.capnp"
    if fallback.is_file():
        return fallback
    raise FileNotFoundError(
        "Could not locate sdf_query.capnp. Pass --schema, set "
        "ERL_GP_SDF_ROS_CAPNP_SCHEMA, or source the colcon install."
    )


def _parse_points(spec: str) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(",")]
        if len(parts) != 3:
            raise ValueError(f"point '{chunk}' must have 3 comma-separated values")
        points.append((float(parts[0]), float(parts[1]), float(parts[2])))
    if not points:
        raise ValueError("no query points provided")
    return points


def _set_query_points(request_builder, points):
    pts = request_builder.init("queryPoints", len(points))
    for i, (x, y, z) in enumerate(points):
        pts[i].x = float(x)
        pts[i].y = float(y)
        pts[i].z = float(z)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=51111)
    parser.add_argument(
        "--points",
        default="0,0,0; 0.5,0.0,0.1",
        help='Semicolon-separated list of "x,y,z" world-frame points',
    )
    parser.add_argument(
        "--mode",
        choices=("logodd", "prob"),
        default="prob",
        help="Occupancy query mode",
    )
    parser.add_argument(
        "--gradient",
        action="store_true",
        help="Request occupancy gradients (BHM only)",
    )
    parser.add_argument(
        "--skip-sdf",
        action="store_true",
        help="Skip the SDF query",
    )
    parser.add_argument(
        "--skip-occ",
        action="store_true",
        help="Skip the occupancy query",
    )
    parser.add_argument(
        "--schema",
        help="Override path to sdf_query.capnp",
    )
    args = parser.parse_args()

    schema_path = _find_schema(args.schema)
    print(f"Loading schema from {schema_path}", file=sys.stderr)
    sdf_query = capnp.load(str(schema_path))

    points = _parse_points(args.points)

    address = f"{args.host}:{args.port}"
    print(f"Connecting to {address}", file=sys.stderr)
    client = capnp.TwoPartyClient(address)
    mapper = client.bootstrap().cast_as(sdf_query.SdfMapper)

    if not args.skip_sdf:
        req_builder = sdf_query.SdfQueryRequest.new_message()
        _set_query_points(req_builder, points)
        response = mapper.querySdf(request=req_builder).wait().response
        print("=== SDF ===")
        print(f"success={response.success} dim={response.dim}")
        print(
            "flags: gradient={} gradient_variance={} covariance={}".format(
                response.computeGradient,
                response.computeGradientVariance,
                response.computeCovariance,
            )
        )
        for i, (pt, d) in enumerate(zip(points, response.signedDistances)):
            line = f"  [{i}] pt={pt} d={d:.4f}"
            if response.computeGradient and i < len(response.gradients):
                g = response.gradients[i]
                line += f" grad=({g.x:.3f}, {g.y:.3f}, {g.z:.3f})"
            print(line)

    if not args.skip_occ:
        req_builder = sdf_query.OccQueryRequest.new_message()
        _set_query_points(req_builder, points)
        req_builder.mode = args.mode
        req_builder.computeGradient = args.gradient
        response = mapper.queryOcc(request=req_builder).wait().response
        print("=== OCC ===")
        print(
            f"success={response.success} dim={response.dim} "
            f"reason={response.reason!r}"
        )
        if response.success:
            for i, (pt, val) in enumerate(zip(points, response.results)):
                line = f"  [{i}] pt={pt} {args.mode}={val:.4f}"
                if args.gradient and i < len(response.gradients):
                    g = response.gradients[i]
                    line += f" grad=({g.x:.3f}, {g.y:.3f}, {g.z:.3f})"
                print(line)

    return 0


if __name__ == "__main__":
    sys.exit(main())
