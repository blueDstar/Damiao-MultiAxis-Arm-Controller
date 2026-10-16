"""Snapshot exports in a background process; no control or serial operations."""
from __future__ import annotations

import json
import os
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path


SAIGON = timezone(timedelta(hours=7))
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "motor_feedback"


def export_runtime():
    runtime = Path.home() / ".cache/codex-runtimes/codex-primary-runtime/dependencies/node"
    node = Path(os.getenv("DAMIAO_EXPORT_NODE", str(runtime / "bin/node.exe")))
    modules = Path(os.getenv("DAMIAO_EXPORT_MODULES", str(runtime / "node_modules")))
    if not node.is_file() or not (modules / "@oai/artifact-tool/package.json").is_file():
        raise RuntimeError("Không tìm thấy runtime xuất ảnh/Excel. Xem cấu hình xuất trong README.")
    return node, modules


def export_feedback(motor_id, samples, zero_offset, formats=("png", "xlsx"), output_dir=None, saved_at=None, source="CAN feedback"):
    if not samples:
        raise ValueError("Chưa có feedback để lưu.")
    if not set(formats) <= {"png", "xlsx"} or not formats:
        raise ValueError("Định dạng phải là png hoặc xlsx.")
    saved_at = (saved_at or datetime.now(SAIGON)).astimezone(SAIGON)
    directory = Path(output_dir) if output_dir else OUTPUT_DIR
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"motor_feedback_motor_0x{motor_id:02X}_{saved_at:%Y-%m-%d_%H-%M-%S_%f}"
    base, suffix = directory / stem, 1
    while any(base.with_suffix("." + f).exists() for f in formats):
        base = directory / f"{stem}_{suffix:03d}"
        suffix += 1
    rows = []
    start = samples[0][0]
    for s in samples:
        rows.append([
            (s[9] / 86400 + 25569 + 7 / 24) if len(s) > 9 else None,
            s[0] - start, s[1] - zero_offset, s[2],
            s[4] if len(s) > 4 else None, s[5] if len(s) > 5 else None,
            s[3], s[10] if len(s) > 10 else None,
            s[7] if len(s) > 7 else None, s[8] if len(s) > 8 else "",
            s[6] - zero_offset if len(s) > 6 and s[6] is not None else None,
        ])
    payload = {"motor_id": motor_id, "saved_local": saved_at.isoformat(), "source": source,
               "zero_offset_rad": zero_offset, "rows": rows, "base": str(base), "formats": list(formats)}
    node, modules = export_runtime()
    with tempfile.TemporaryDirectory(prefix="motor-feedback-") as temporary:
        snapshot = Path(temporary) / "snapshot.json"
        snapshot.write_text(json.dumps(payload, allow_nan=False), encoding="utf-8")
        result = subprocess.run([str(node), str(Path(__file__).with_suffix(".mjs")), str(snapshot), str(modules)],
                                capture_output=True, text=True, timeout=180,
                                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
    if result.returncode:
        raise RuntimeError(result.stderr.strip()[-1600:] or "Không xuất được dữ liệu.")
    outputs = [base.with_suffix("." + f) for f in formats]
    if any(not f.is_file() for f in outputs):
        raise RuntimeError("Quá trình xuất chưa tạo đủ các file yêu cầu.")
    return outputs
