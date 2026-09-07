from __future__ import annotations

import os
import struct
from dataclasses import dataclass
from pathlib import Path


BMP_FILE_HEADER = struct.Struct("<2sIHHI")
BMP_INFO_HEADER = struct.Struct("<IiiHHIIiiII")
BMP_PALETTE = b"".join(bytes((value, value, value, 0)) for value in range(256))
BMP_PIXEL_OFFSET = BMP_FILE_HEADER.size + BMP_INFO_HEADER.size + len(BMP_PALETTE)


@dataclass(frozen=True)
class BmpInfo:
    width: int
    height: int
    bits_per_pixel: int
    row_stride: int
    pixel_offset: int
    file_size: int
    top_down: bool


def write_mono8_bmp(
    path: str | Path,
    width: int,
    height: int,
    pixels: bytes | bytearray | memoryview,
) -> None:
    """Write a bottom-up mono8 BMP in large blocks, without payload hashing."""

    if width <= 0 or height <= 0:
        raise ValueError("BMP dimensions must be positive")
    view = memoryview(pixels).cast("B")
    expected = width * height
    if len(view) != expected:
        raise ValueError(f"pixel length {len(view)} does not match {width}x{height}")

    row_stride = (width + 3) & ~3
    padding = b"\x00" * (row_stride - width)
    pixel_bytes = row_stride * height
    file_size = BMP_PIXEL_OFFSET + pixel_bytes
    file_header = BMP_FILE_HEADER.pack(b"BM", file_size, 0, 0, BMP_PIXEL_OFFSET)
    info_header = BMP_INFO_HEADER.pack(
        BMP_INFO_HEADER.size,
        width,
        height,
        1,
        8,
        0,
        pixel_bytes,
        2835,
        2835,
        256,
        256,
    )

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    # Keep the existing byte layout, including row padding, while reducing
    # thousands of per-row file writes to one header and one pixel-block write.
    if padding:
        payload = b"".join(
            bytes(view[row * width : (row + 1) * width]) + padding
            for row in range(height - 1, -1, -1)
        )
    else:
        payload = b"".join(
            view[row * width : (row + 1) * width]
            for row in range(height - 1, -1, -1)
        )
    with target.open("wb") as stream:
        stream.write(file_header + info_header + BMP_PALETTE)
        stream.write(payload)
        stream.flush()
    if target.stat().st_size != file_size:
        raise OSError(f"BMP size mismatch after write: {target}")


def inspect_mono8_bmp(path: str | Path) -> BmpInfo:
    target = Path(path)
    file_size = target.stat().st_size
    with target.open("rb") as stream:
        file_raw = stream.read(BMP_FILE_HEADER.size)
        info_raw = stream.read(BMP_INFO_HEADER.size)
        palette_raw = stream.read(len(BMP_PALETTE))
    if len(file_raw) != BMP_FILE_HEADER.size or len(info_raw) != BMP_INFO_HEADER.size:
        raise ValueError(f"truncated BMP header: {target}")
    magic, declared_size, _, _, pixel_offset = BMP_FILE_HEADER.unpack(file_raw)
    (
        info_size,
        width,
        signed_height,
        planes,
        bits,
        compression,
        image_size,
        _,
        _,
        _,
        _,
    ) = BMP_INFO_HEADER.unpack(info_raw)
    if magic != b"BM" or info_size != BMP_INFO_HEADER.size:
        raise ValueError(f"unsupported BMP header: {target}")
    if width <= 0 or signed_height == 0 or planes != 1 or bits != 8 or compression != 0:
        raise ValueError(f"BMP is not uncompressed mono8: {target}")
    if signed_height < 0:
        raise ValueError(f"BMP must use the canonical bottom-up row order: {target}")
    height = abs(signed_height)
    if pixel_offset != BMP_PIXEL_OFFSET:
        raise ValueError(f"BMP does not use the canonical mono8 palette layout: {target}")
    if palette_raw != BMP_PALETTE:
        raise ValueError(f"BMP does not use the canonical grayscale palette: {target}")
    row_stride = (width + 3) & ~3
    expected_image_size = row_stride * height
    expected_size = pixel_offset + expected_image_size
    if image_size not in (0, expected_image_size):
        raise ValueError(f"invalid BMP pixel size: {target}")
    if declared_size not in (0, file_size) or file_size != expected_size:
        raise ValueError(f"invalid BMP file size: {target}")
    return BmpInfo(
        width=width,
        height=height,
        bits_per_pixel=bits,
        row_stride=row_stride,
        pixel_offset=pixel_offset,
        file_size=file_size,
        top_down=signed_height < 0,
    )


def read_mono8_bmp(path: str | Path) -> tuple[BmpInfo, bytes]:
    info = inspect_mono8_bmp(path)
    rows: list[bytes] = []
    with Path(path).open("rb") as stream:
        stream.seek(info.pixel_offset, os.SEEK_SET)
        for _ in range(info.height):
            raw = stream.read(info.row_stride)
            if len(raw) != info.row_stride:
                raise ValueError(f"truncated BMP pixels: {path}")
            rows.append(raw[: info.width])
    if not info.top_down:
        rows.reverse()
    return info, b"".join(rows)
