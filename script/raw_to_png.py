"""PC-only, uncompressed uint16 Bayer -> color and grayscale PNGs."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def gray_output_path(output: Path) -> Path:
    return output.with_name(output.stem + "_gray.png")


def check_capture_metadata(source: Path, width: int, height: int) -> None:
    # Recognize the project's cameras/camN/file.raw layout, if still present.
    if source.parent.parent.name != "cameras":
        return
    root = source.parent.parent.parent
    index = root / "meta/pairs.jsonl"
    if not index.is_file():
        return
    relative = source.relative_to(root).as_posix()
    with index.open(encoding="utf-8") as stream:
        for line in stream:
            pair = json.loads(line)
            for side, filename in zip(("left", "right"), pair["files"]):
                if filename != relative:
                    continue
                frame = pair[side]
                if frame.get("compress_mode") != 0:
                    raise ValueError("capture metadata marks compressed RAW; SDK decompression is required")
                if frame.get("format") != 0x84:
                    raise ValueError("capture metadata is not RAW16 (0x84); unpack/convert separately")
                if (frame["width"], frame["height"]) != (width, height):
                    raise ValueError("dimensions conflict with capture metadata")
                return
    raise ValueError("RAW file is missing from capture meta/pairs.jsonl")


def convert(source, output=None, *, width=3840, height=2160, pattern="RGGB",
            endian="little", stride_bytes=None, bits=16, alignment="lsb", png_bits=8):
    import cv2
    import numpy as np

    source = Path(source).expanduser().resolve()
    output = Path(output).expanduser().resolve() if output else source.with_suffix(".png")
    if width < 4 or height < 4 or width % 2 or height % 2:
        raise ValueError("width/height must be even and >= 4")
    if not 1 <= bits <= 16 or png_bits not in (8, 16):
        raise ValueError("bits must be 1..16; png_bits must be 8 or 16")
    if pattern not in ("RGGB", "BGGR", "GRBG", "GBRG"):
        raise ValueError("unsupported Bayer pattern")
    if endian not in ("little", "big") or alignment not in ("lsb", "msb"):
        raise ValueError("invalid byte order or bit alignment")
    stride = width * 2 if stride_bytes is None else stride_bytes
    if stride < width * 2 or stride % 2:
        raise ValueError("stride_bytes must be even and >= width*2")
    if output == source or output.suffix.lower() != ".png":
        raise ValueError("output must be a separate .png file")
    gray_output = gray_output_path(output)
    if gray_output.resolve() == source:
        raise ValueError("grayscale output must not replace input")
    for destination in (output, gray_output):
        if destination.exists():
            raise FileExistsError(f"output already exists: {destination}")
    check_capture_metadata(source, width, height)
    expected = stride * height
    if source.stat().st_size != expected:
        raise ValueError(f"RAW16 requires exactly {expected} bytes; got {source.stat().st_size}. "
                         "Check dimensions/stride; compressed or packed RAW is not supported.")
    raw = np.fromfile(source, dtype="<u2" if endian == "little" else ">u2")
    mosaic = np.ascontiguousarray(raw.reshape(height, stride // 2)[:, :width], dtype=np.uint16)
    if alignment == "msb" and bits < 16:
        mosaic = mosaic >> (16 - bits)
    maximum = (1 << bits) - 1
    if int(mosaic.max()) > maximum:
        raise ValueError("pixel values exceed --bits; check bit depth/alignment/endian")
    # Use explicit four-letter names: OpenCV's legacy two-letter aliases are confusing.
    bgr = cv2.cvtColor(mosaic, getattr(cv2, f"COLOR_Bayer{pattern}2BGR"))
    peak = (1 << png_bits) - 1
    pixels = ((bgr.astype(np.uint32) * peak + maximum // 2) // maximum).astype(
        np.uint8 if png_bits == 8 else np.uint16)
    gray = cv2.cvtColor(pixels, cv2.COLOR_BGR2GRAY)
    encoded_images = []
    for image in (pixels, gray):
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise RuntimeError("OpenCV PNG encoding failed")
        encoded_images.append(encoded)
    # Exclusive creation protects existing results; source is always read-only.
    output.parent.mkdir(parents=True, exist_ok=True)
    created = []
    try:
        for destination, encoded in zip((output, gray_output), encoded_images):
            with destination.open("xb") as stream:
                created.append(destination)
                stream.write(encoded.tobytes())
    except BaseException:
        # Remove only files exclusively created by this attempt, never old results.
        for destination in created:
            destination.unlink()
        raise
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="one uncompressed RAW16 frame")
    parser.add_argument("--output", "-o", help="color PNG path; also writes <stem>_gray.png; no overwrite")
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=2160)
    parser.add_argument("--pattern", choices=("RGGB", "BGGR", "GRBG", "GBRG"), default="RGGB")
    parser.add_argument("--endian", choices=("little", "big"), default="little")
    parser.add_argument("--stride-bytes", type=int, help="bytes per row (default width*2)")
    parser.add_argument("--bits", type=int, default=16, help="effective bits in uint16 container (default 16)")
    parser.add_argument("--alignment", choices=("lsb", "msb"), default="lsb")
    parser.add_argument("--png-bits", type=int, choices=(8, 16), default=8)
    args = vars(parser.parse_args(argv))
    args["source"] = args.pop("input")
    try:
        output = convert(**args)
        print(output)
        print(gray_output_path(output))
        return 0
    except (ValueError, OSError, ImportError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
