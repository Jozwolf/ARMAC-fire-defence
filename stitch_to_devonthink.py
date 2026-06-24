#!/usr/bin/env python3
"""
Stitch multiple iPad screenshots into a single PDF and import it into DEVONthink.

Usage:
    python3 stitch_to_devonthink.py screenshot1.png screenshot2.png screenshot3.png
    python3 stitch_to_devonthink.py *.png --import
    python3 stitch_to_devonthink.py *.png -o output.pdf --import --group "Fire Defence"

Requirements:
    pip3 install Pillow numpy
"""

import sys
import os
import subprocess
import argparse
from pathlib import Path

try:
    from PIL import Image
    import numpy as np
except ImportError:
    print("Missing dependencies. Run:  pip3 install Pillow numpy")
    sys.exit(1)


def find_overlap(img1: Image.Image, img2: Image.Image, max_overlap_px: int = 300) -> int:
    """
    Find how many pixels at the bottom of img1 duplicate the top of img2.
    Returns 0 if no reliable overlap is found.
    """
    arr1 = np.array(img1.convert("L"), dtype=float)
    arr2 = np.array(img2.convert("L"), dtype=float)

    best_offset = 0
    best_score = float("inf")

    probe_limit = min(max_overlap_px, img1.height, img2.height)
    for offset in range(10, probe_limit):
        strip1 = arr1[-offset:, :]
        strip2 = arr2[:offset, :]
        score = float(np.mean(np.abs(strip1 - strip2)))
        if score < best_score:
            best_score = score
            best_offset = offset

    # Only trim if the match is visually very close (mean pixel diff < 8/255)
    return best_offset if best_score < 8.0 else 0


def stitch_images(image_paths: list, detect_overlap: bool = True) -> Image.Image:
    """Vertically concatenate images, trimming duplicate scroll regions."""
    images = [Image.open(p) for p in image_paths]
    if not images:
        raise ValueError("No images supplied.")

    # Normalise to the width of the first image
    w = images[0].width
    normalised = []
    for img in images:
        if img.width != w:
            h = int(img.height * w / img.width)
            img = img.resize((w, h), Image.LANCZOS)
        normalised.append(img.convert("RGB"))

    strips = [normalised[0]]
    for img in normalised[1:]:
        if detect_overlap:
            trim = find_overlap(strips[-1], img)
            strips.append(img.crop((0, trim, img.width, img.height)))
        else:
            strips.append(img)

    total_h = sum(s.height for s in strips)
    canvas = Image.new("RGB", (w, total_h), (255, 255, 255))
    y = 0
    for strip in strips:
        canvas.paste(strip, (0, y))
        y += strip.height

    return canvas


def save_as_pdf(image: Image.Image, path: Path, dpi: int = 150) -> None:
    image.save(str(path), "PDF", resolution=dpi, save_all=False)


def import_to_devonthink(pdf_path: Path, group: str = "") -> None:
    """
    Import a file into DEVONthink 3 / DEVONthink Pro via osascript.
    Runs on macOS only.
    """
    posix = str(pdf_path.resolve())
    if group:
        script = f'''
tell application id "DNtp"
    activate
    set theRecord to import POSIX file "{posix}"
    move record theRecord to (get group with name "{group}")
end tell
'''
    else:
        script = f'''
tell application id "DNtp"
    activate
    import POSIX file "{posix}"
end tell
'''
    result = subprocess.run(
        ["osascript", "-e", script],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"DEVONthink import failed:\n{result.stderr.strip()}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stitch iPad screenshots into a PDF and optionally import to DEVONthink."
    )
    parser.add_argument(
        "images",
        nargs="+",
        type=Path,
        metavar="IMAGE",
        help="Screenshot files in scroll order (PNG, JPEG, HEIC, etc.)",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        metavar="FILE",
        help="Output PDF path (default: <first_image>_stitched.pdf)",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="PDF resolution in DPI (default: 150)",
    )
    parser.add_argument(
        "--no-overlap",
        action="store_true",
        help="Disable automatic overlap/duplicate-region trimming",
    )
    parser.add_argument(
        "--import",
        dest="do_import",
        action="store_true",
        help="Import the finished PDF into DEVONthink (macOS only)",
    )
    parser.add_argument(
        "--group",
        default="",
        metavar="GROUP",
        help='DEVONthink group/folder name to file the document in',
    )
    args = parser.parse_args()

    paths = sorted(args.images)
    missing = [p for p in paths if not p.exists()]
    if missing:
        parser.error(f"Files not found: {', '.join(str(p) for p in missing)}")

    print(f"Stitching {len(paths)} image(s)…")
    stitched = stitch_images(paths, detect_overlap=not args.no_overlap)

    output: Path = args.output or paths[0].parent / (paths[0].stem + "_stitched.pdf")
    save_as_pdf(stitched, output, dpi=args.dpi)
    print(f"Saved: {output}  ({output.stat().st_size // 1024} KB)")

    if args.do_import:
        print("Importing into DEVONthink…")
        import_to_devonthink(output, args.group)
        print("Import complete.")


if __name__ == "__main__":
    main()
