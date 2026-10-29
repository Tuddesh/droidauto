"""Persistent JSONL event recording shared by future reset orchestration.

Events are flushed as they occur. Log naming is independent of any experimental
GUI/task; generated records belong in ignored runs/ directories.
"""

from datetime import datetime, timezone
import json
from pathlib import Path
import time


class EventLog:
    """Append complete events; preserve failed attempts and completed operations."""

    def __init__(self, path, *, echo=True):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("x")
        self.echo = echo

    def emit(self, event, **fields):
        record = {"time_s": time.time(), "event": event, **fields}
        self._file.write(json.dumps(record, allow_nan=False) + "\n")
        self._file.flush()
        if self.echo:
            print(event, json.dumps(fields), flush=True)

    def close(self):
        self._file.close()


def new_log_path(directory, prefix="reset"):
    """Choose a timestamped path for a reset or named diagnostic operation."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    return Path(directory) / f"{prefix}_{stamp}.jsonl"
