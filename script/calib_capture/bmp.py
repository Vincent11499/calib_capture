from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass
from pathlib import Path


BMP_FILE_HEADER = struct.Struct("<2sIHHI")
BMP_INFO_HEADER = struct.Struct("<IiiHHIIiiII")
BMP_PALETTE = b"".join(bytes((value, value, value, 0)) for value in range(256))
BMP_PIXEL_OFFSET = BMP_FILE_HEADER.size + BMP_INFO_HEADER.size + len(BMP_PALETTE)
RGB_BMP_PIXEL_OFFSET = BMP_FILE_HEADER.size + BMP_INFO_HEADER.size


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
    bottom_up: bool = False,
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
    if bottom_up and not padding:
        payload = view
    elif padding:
        payload = b"".join(
            bytes(view[row * width : (row + 1) * width]) + padding
            for row in (range(height) if bottom_up else range(height - 1, -1, -1))
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


def _rgb_to_bgr_payload(
    pixels: memoryview, width: int, height: int, row_stride: int,
) -> bytes | bytearray:
    row_bytes = width * 3
    try:
        import numpy as np
    except ImportError:
        payload = bytearray(row_stride * height)
        for row in range(height):
            source_start = row * row_bytes
            target_start = row * row_stride
            for column in range(width):
                source = source_start + column * 3
                target = target_start + column * 3
                payload[target] = pixels[source + 2]
                payload[target + 1] = pixels[source + 1]
                payload[target + 2] = pixels[source]
        return payload

    source = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3)
    if row_stride == row_bytes:
        return np.ascontiguousarray(source[:, :, ::-1]).tobytes()
    payload = np.zeros((height, row_stride), dtype=np.uint8)
    payload[:, :row_bytes].reshape(height, width, 3)[:] = source[:, :, ::-1]
    return payload.tobytes()


def write_rgb8_bmp(
    path: str | Path,
    width: int,
    height: int,
    pixels: bytes | bytearray | memoryview,
) -> dict[str, int]:
    """Write top-down logical RGB8 pixels as a standard 24-bit BMP."""
    if width <= 0 or height <= 0:
        raise ValueError("BMP dimensions must be positive")
    view = memoryview(pixels).cast("B")
    expected = width * height * 3
    if len(view) != expected:
        raise ValueError(
            f"RGB pixel length {len(view)} does not match {width}x{height}x3"
        )
    row_stride = (width * 3 + 3) & ~3
    pixel_bytes = row_stride * height
    file_size = RGB_BMP_PIXEL_OFFSET + pixel_bytes
    file_header = BMP_FILE_HEADER.pack(
        b"BM", file_size, 0, 0, RGB_BMP_PIXEL_OFFSET
    )
    # A negative height is the BMP top-down convention.  It avoids reversing
    # the captured rows; only the required RGB-to-BGR byte order changes.
    info_header = BMP_INFO_HEADER.pack(
        BMP_INFO_HEADER.size,
        width,
        -height,
        1,
        24,
        0,
        pixel_bytes,
        2835,
        2835,
        0,
        0,
    )
    started = time.perf_counter_ns()
    payload = _rgb_to_bgr_payload(view, width, height, row_stride)
    encoded_at = time.perf_counter_ns()
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("wb") as stream:
        stream.write(file_header + info_header)
        stream.write(payload)
        stream.flush()
    if target.stat().st_size != file_size:
        raise OSError(f"BMP size mismatch after write: {target}")
    return {
        "rgb_to_bgr": encoded_at - started,
        "file_write": time.perf_counter_ns() - encoded_at,
    }


def inspect_rgb8_bmp(path: str | Path) -> BmpInfo:
    target = Path(path)
    file_size = target.stat().st_size
    with target.open("rb") as stream:
        file_raw = stream.read(BMP_FILE_HEADER.size)
        info_raw = stream.read(BMP_INFO_HEADER.size)
    if len(file_raw) != BMP_FILE_HEADER.size or len(info_raw) != BMP_INFO_HEADER.size:
        raise ValueError(f"truncated BMP header: {target}")
    magic, declared_size, _, _, pixel_offset = BMP_FILE_HEADER.unpack(file_raw)
    (
        info_size, width, signed_height, planes, bits, compression, image_size,
        _, _, _, _,
    ) = BMP_INFO_HEADER.unpack(info_raw)
    if magic != b"BM" or info_size != BMP_INFO_HEADER.size:
        raise ValueError(f"unsupported BMP header: {target}")
    if width <= 0 or signed_height >= 0 or planes != 1 or bits != 24 or compression != 0:
        raise ValueError(f"BMP is not canonical top-down RGB8 storage: {target}")
    if pixel_offset != RGB_BMP_PIXEL_OFFSET:
        raise ValueError(f"BMP does not use the canonical RGB8 layout: {target}")
    height = -signed_height
    row_stride = (width * 3 + 3) & ~3
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
        top_down=True,
    )


def read_rgb8_bmp(path: str | Path) -> tuple[BmpInfo, bytes]:
    info = inspect_rgb8_bmp(path)
    output = bytearray(info.width * info.height * 3)
    row_bytes = info.width * 3
    with Path(path).open("rb") as stream:
        stream.seek(info.pixel_offset, os.SEEK_SET)
        for row in range(info.height):
            raw = stream.read(info.row_stride)
            if len(raw) != info.row_stride:
                raise ValueError(f"truncated BMP pixels: {path}")
            target = row * row_bytes
            for column in range(info.width):
                source = column * 3
                destination = target + source
                output[destination] = raw[source + 2]
                output[destination + 1] = raw[source + 1]
                output[destination + 2] = raw[source]
    return info, bytes(output)
