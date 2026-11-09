"""BMP-only capture image storage with no OpenCV dependency."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

from .bmp import (
    inspect_mono8_bmp,
    inspect_rgb8_bmp,
    read_mono8_bmp,
    read_rgb8_bmp,
    write_mono8_bmp,
    write_rgb8_bmp,
)


@dataclass(frozen=True)
class ImageInfo:
    width: int
    height: int


def image_suffix(image_format: str) -> str:
    if image_format == "bmp":
        return ".bmp"
    raise ValueError(f"unsupported image_format: {image_format}; only bmp is supported")


def ensure_image_support(image_format: str, pixel_format: str = "mono8") -> None:
    image_suffix(image_format)
    if pixel_format not in {"mono8", "rgb"}:
        raise ValueError(f"unsupported pixel_format: {pixel_format}")


def write_mono8_image(path: Path, width: int, height: int, pixels: bytes,
                      image_format: str,
                      bottom_up: bool = False) -> dict[str, int]:
    start = time.perf_counter_ns()
    image_suffix(image_format)
    write_mono8_bmp(path, width, height, pixels, bottom_up)
    return {"file_write": time.perf_counter_ns() - start}


def write_image(
    path: Path,
    width: int,
    height: int,
    pixels: bytes | bytearray,
    image_format: str,
    pixel_format: str,
    bottom_up: bool = False,
) -> dict[str, int]:
    if pixel_format == "mono8":
        return write_mono8_image(
            path, width, height, pixels, image_format, bottom_up
        )
    ensure_image_support(image_format, pixel_format)
    if bottom_up:
        raise ValueError("RGB8 input must use top-down row order")
    return write_rgb8_bmp(path, width, height, pixels)


def read_mono8_image(path: str | Path) -> tuple[ImageInfo, bytes]:
    target = Path(path)
    if target.suffix.lower() != ".bmp":
        raise ValueError(f"unsupported image extension: {target}; only BMP is supported")
    info, pixels = read_mono8_bmp(target)
    return ImageInfo(info.width, info.height), pixels


def read_rgb8_image(path: str | Path) -> tuple[ImageInfo, bytes]:
    target = Path(path)
    if target.suffix.lower() != ".bmp":
        raise ValueError("RGB8 storage currently supports BMP only")
    info, pixels = read_rgb8_bmp(target)
    return ImageInfo(info.width, info.height), pixels


def inspect_mono8_image(path: str | Path) -> ImageInfo:
    if Path(path).suffix.lower() != ".bmp":
        raise ValueError(f"unsupported image extension: {path}; only BMP is supported")
    info = inspect_mono8_bmp(path)
    return ImageInfo(info.width, info.height)


def inspect_image(path: str | Path, pixel_format: str) -> ImageInfo:
    if pixel_format == "mono8":
        return inspect_mono8_image(path)
    if pixel_format == "rgb":
        info = inspect_rgb8_bmp(path)
        return ImageInfo(info.width, info.height)
    raise ValueError(f"unsupported pixel_format: {pixel_format}")
