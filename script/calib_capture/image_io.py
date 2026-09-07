"""Grayscale storage codecs; BMP needs no optional dependencies."""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import time

from .bmp import inspect_mono8_bmp, read_mono8_bmp, write_mono8_bmp


@dataclass(frozen=True)
class ImageInfo:
    width: int
    height: int


def image_suffix(image_format: str) -> str:
    if image_format == "bmp":
        return ".bmp"
    if image_format == "jpeg":
        return ".jpg"
    raise ValueError(f"unsupported image_format: {image_format}")


@lru_cache(maxsize=1)
def _jpeg_modules():
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("JPEG requires OpenCV (cv2) and NumPy; use an existing environment or select BMP") from exc
    if not cv2.haveImageWriter(".jpg"):
        raise RuntimeError("OpenCV was built without JPEG encoding support")
    return cv2, np


def ensure_image_support(image_format: str) -> None:
    image_suffix(image_format)
    if image_format == "jpeg":
        _jpeg_modules()


def write_mono8_image(path: Path, width: int, height: int, pixels: bytes,
                      image_format: str, jpeg_quality: int = 95,
                      bottom_up: bool = False) -> dict[str, int]:
    start = time.perf_counter_ns()
    if image_format == "bmp":
        write_mono8_bmp(path, width, height, pixels, bottom_up)
        return {"file_write": time.perf_counter_ns() - start}
    image_suffix(image_format)
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("jpeg_quality must be between 1 and 100")
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError("invalid mono8 image dimensions or pixel length")
    cv2, np = _jpeg_modules()
    gray = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width)
    if bottom_up:
        raise ValueError("JPEG input must use top-down row order")
    # Encode before writing: paired commits use .tmp paths, not .jpg paths.
    success, encoded = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not success or encoded is None or not encoded.size:
        raise RuntimeError("OpenCV JPEG encoding failed")
    encoded_at = time.perf_counter_ns()
    with path.open("wb") as stream:
        stream.write(memoryview(encoded))
    return {"jpeg_encode": encoded_at - start,
            "file_write": time.perf_counter_ns() - encoded_at}


def read_mono8_image(path: str | Path) -> tuple[ImageInfo, bytes]:
    target = Path(path)
    if target.suffix.lower() == ".bmp":
        info, pixels = read_mono8_bmp(target)
        return ImageInfo(info.width, info.height), pixels
    gray = _decode_jpeg(target)
    height, width = gray.shape
    return ImageInfo(width, height), gray.tobytes()


def _decode_jpeg(target: Path):
    if target.suffix.lower() not in (".jpg", ".jpeg"):
        raise ValueError(f"unsupported image extension: {target}")
    cv2, np = _jpeg_modules()
    encoded = target.read_bytes()
    if not encoded.startswith(b"\xff\xd8"):
        raise ValueError(f"invalid JPEG header: {target}")
    if not encoded.endswith(b"\xff\xd9"):
        raise ValueError(f"truncated JPEG: {target}")
    try:
        gray = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    except cv2.error as exc:
        raise ValueError(f"JPEG decode failed: {target}: {exc}") from exc
    if gray is None:
        raise ValueError(f"JPEG decode failed: {target}")
    if gray.ndim != 2 or gray.dtype != np.uint8:
        raise ValueError(f"JPEG must be single-channel 8-bit grayscale: {target}")
    return gray


def inspect_mono8_image(path: str | Path) -> ImageInfo:
    if Path(path).suffix.lower() == ".bmp":
        info = inspect_mono8_bmp(path)
        return ImageInfo(info.width, info.height)
    # Decode JPEG completely: file length alone cannot validate compressed data.
    gray = _decode_jpeg(Path(path))
    height, width = gray.shape
    return ImageInfo(width, height)
