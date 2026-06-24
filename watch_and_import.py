#!/usr/bin/env python3
"""
Watch a folder (e.g. ~/Pictures/Screenshots or an iCloud Drive folder) for new
screenshots dropped there from an iPad, then automatically stitch and import
groups of them into DEVONthink.

A "group" is any set of images whose filenames share the same date prefix and
arrive within GROUPING_WINDOW_SECS of each other.

Usage:
    python3 watch_and_import.py ~/Pictures/Screenshots
    python3 watch_and_import.py ~/Library/Mobile\ Documents/com~apple~CloudDocs/ScreenShots --group "Fire Defence"

Requirements:
    pip3 install Pillow numpy watchdog
"""

import sys
import time
import threading
import argparse
from pathlib import Path
from collections import defaultdict

try:
    from watchdog.observers import Observer
    from watchdog.events import FileSystemEventHandler, FileCreatedEvent
except ImportError:
    print("Missing dependency. Run:  pip3 install watchdog")
    sys.exit(1)

from stitch_to_devonthink import stitch_images, save_as_pdf, import_to_devonthink

IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".heic", ".tiff", ".bmp"}
GROUPING_WINDOW_SECS = 8   # images arriving within this window are stitched together
SETTLE_DELAY_SECS   = 3    # wait this long after the last file before processing


class ScreenshotHandler(FileSystemEventHandler):
    def __init__(self, devonthink_group: str, dpi: int, detect_overlap: bool) -> None:
        self.group = devonthink_group
        self.dpi = dpi
        self.detect_overlap = detect_overlap
        self._pending: list[Path] = []
        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None

    def on_created(self, event: FileCreatedEvent) -> None:
        path = Path(event.src_path)
        if path.suffix.lower() not in IMAGE_EXTENSIONS:
            return
        with self._lock:
            self._pending.append(path)
            if self._timer:
                self._timer.cancel()
            self._timer = threading.Timer(SETTLE_DELAY_SECS, self._process)
            self._timer.start()

    def _process(self) -> None:
        with self._lock:
            batch = list(self._pending)
            self._pending.clear()
            self._timer = None

        if not batch:
            return

        batch.sort()
        print(f"\n[{time.strftime('%H:%M:%S')}] Processing {len(batch)} screenshot(s)…")

        try:
            stitched = stitch_images(batch, detect_overlap=self.detect_overlap)
            output = batch[0].parent / (batch[0].stem + "_stitched.pdf")
            save_as_pdf(stitched, output, dpi=self.dpi)
            print(f"  → {output.name}  ({output.stat().st_size // 1024} KB)")
            import_to_devonthink(output, self.group)
            print("  → Imported into DEVONthink.")
        except Exception as exc:
            print(f"  ERROR: {exc}", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Watch a folder for iPad screenshots and import stitched PDFs to DEVONthink."
    )
    parser.add_argument("folder", type=Path, help="Folder to watch for new screenshots")
    parser.add_argument("--group", default="", metavar="GROUP",
                        help="DEVONthink group to import into")
    parser.add_argument("--dpi", type=int, default=150, help="PDF resolution (default: 150)")
    parser.add_argument("--no-overlap", action="store_true",
                        help="Disable scroll-overlap trimming")
    args = parser.parse_args()

    if not args.folder.is_dir():
        parser.error(f"Not a directory: {args.folder}")

    handler = ScreenshotHandler(
        devonthink_group=args.group,
        dpi=args.dpi,
        detect_overlap=not args.no_overlap,
    )
    observer = Observer()
    observer.schedule(handler, str(args.folder), recursive=False)
    observer.start()
    print(f"Watching {args.folder}  (Ctrl-C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()


if __name__ == "__main__":
    main()
