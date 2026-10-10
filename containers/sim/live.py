"""Serve the --view page and stream poses while the policy runs.

./sim.sh starts this before the container. Start writes a flag the shell waits
on. Frames are read from the runner's live pose file and turned into the same
geom matrices the replay page uses.
"""

import argparse
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np
import view
from plant import Plant


class LiveSession:
    def __init__(self, directory, live_path, start_path, report_path, pose, scene):
        self.directory = Path(directory)
        self.live_path = Path(live_path)
        self.start_path = Path(start_path)
        self.report_path = Path(report_path)
        plant = Plant(pose, scene=scene or None, check_fall=False)
        self.model = plant.model
        self.data = plant.data
        self.base, self.geoms = view.visible_geoms(self.model)
        self.frames = []
        self.offset = 0
        self.pending = ""
        self.inode = None
        self.lock = threading.Lock()

    def start(self):
        self.start_path.parent.mkdir(parents=True, exist_ok=True)
        self.start_path.write_text("start\n")
        self.report_path.unlink(missing_ok=True)

    def _rewind(self):
        self.inode = None
        self.offset = 0
        self.pending = ""
        self.frames.clear()

    def pull(self):
        """Read new qpos lines. A replaced file returns True so the page resets."""
        path = self.live_path
        try:
            stat = path.stat()
        except FileNotFoundError:
            if self.inode is not None or self.frames:
                self._rewind()
                return True
            return False
        reset = False
        if stat.st_ino != self.inode or stat.st_size < self.offset:
            self.inode = stat.st_ino
            self.offset = 0
            self.pending = ""
            self.frames.clear()
            reset = True
        try:
            with path.open("rb") as handle:
                handle.seek(self.offset)
                raw = handle.read()
                end = handle.tell()
                inode = os.fstat(handle.fileno()).st_ino
        except FileNotFoundError:
            self._rewind()
            return True
        if inode != stat.st_ino:
            self._rewind()
            return True
        self.offset = end
        self.inode = inode
        if not raw:
            return reset
        lines = (self.pending + raw.decode("utf-8", "replace")).split("\n")
        self.pending = lines.pop()
        for line in lines:
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                # The runner replaces this file at the first walking step.
                # A read that lands mid-line is discarded and the next poll
                # starts over, instead of failing the request.
                self._rewind()
                return True
            qpos = np.asarray(item["qpos"], dtype=float)
            if qpos.shape != (self.model.nq,):
                continue
            frame = view.pose_frame(self.model, self.data, self.base, self.geoms, qpos, item["t"])
            frame["thrash"] = item.get("thrash")
            frame["stop"] = view.describe_stop(item.get("stop"))
            self.frames.append(frame)
        return reset

    def snapshot(self, after):
        with self.lock:
            reset = self.pull()
            start = 0 if reset else max(0, after)
            done = self.report_path.is_file() and not self.pending
            summary = None
            stop = None
            if done:
                try:
                    report = json.loads(self.report_path.read_text())
                    summary = view.report_summary(report)
                    stop = view.describe_stop(view.stop_from_report(report))
                except (OSError, json.JSONDecodeError):
                    done = False
            return {
                "frames": self.frames[start:],
                "reset": reset,
                "done": done,
                "summary": summary,
                "stop": stop,
            }


def serve(session, host, port):
    session_box = session

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            if urlparse(self.path).path != "/start":
                self.send_error(404)
                return
            session_box.start()
            body = b'{"ok":true}\n'
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/frames":
                after = int(parse_qs(parsed.query).get("after", ["0"])[0] or 0)
                body = json.dumps(session_box.snapshot(after)).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self._file(parsed.path)

        def _file(self, request_path):
            relative = request_path.lstrip("/") or "index.html"
            target = (session_box.directory / relative).resolve()
            root = session_box.directory.resolve()
            if root not in target.parents and target != root:
                self.send_error(404)
                return
            if not target.is_file():
                self.send_error(404)
                return
            data = target.read_bytes()
            kind = {
                ".html": "text/html; charset=utf-8",
                ".js": "text/javascript; charset=utf-8",
                ".json": "application/json",
                ".stl": "model/stl",
            }.get(target.suffix, "application/octet-stream")
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *args):
            if args and str(args[0]).startswith(("GET /frames", "GET /meshes")):
                return
            super().log_message(fmt, *args)

    try:
        server = ThreadingHTTPServer((host, port), Handler)
    except OSError as exc:
        raise SystemExit(f"Cannot listen on {host}:{port}: {exc}") from exc
    print(f"Open http://{host}:{port}", flush=True)
    server.serve_forever()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", required=True)
    parser.add_argument("--live", required=True)
    parser.add_argument("--start", required=True)
    parser.add_argument("--report", required=True)
    parser.add_argument("--pose", default="standing")
    parser.add_argument("--scene", default="")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    session = LiveSession(
        args.directory,
        args.live,
        args.start,
        args.report,
        args.pose,
        args.scene or None,
    )
    serve(session, args.bind, args.port)


if __name__ == "__main__":
    main()
