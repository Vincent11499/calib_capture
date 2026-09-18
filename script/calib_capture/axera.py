from __future__ import annotations

import ctypes
import hashlib
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .config import SDK_HEADER_SHA256
from .performance import Timings


AX_FORMAT_YUV400 = 0x0
AX_FORMAT_YUV420_PLANAR = 0x1
AX_FORMAT_YUV420_PLANAR_VU = 0x2
AX_FORMAT_YUV420_SEMIPLANAR = 0x3
AX_FORMAT_YUV420_SEMIPLANAR_VU = 0x4
AX_FORMAT_RGB888 = 0xA1  # Vendor header: BGR byte order in memory.
AX_FORMAT_BGR888 = 0xA5  # Vendor header: RGB byte order in memory.
AX_COMPRESS_MODE_NONE = 0

FORMAT_NAMES = {
    AX_FORMAT_YUV400: "YUV400",
    AX_FORMAT_YUV420_PLANAR: "YUV420_PLANAR",
    AX_FORMAT_YUV420_PLANAR_VU: "YUV420_PLANAR_VU",
    AX_FORMAT_YUV420_SEMIPLANAR: "NV12",
    AX_FORMAT_YUV420_SEMIPLANAR_VU: "NV21",
    AX_FORMAT_RGB888: "RGB888(memory=BGR)",
    AX_FORMAT_BGR888: "BGR888(memory=RGB)",
}


class AxFrameCompressInfo(ctypes.Structure):
    _fields_ = [
        ("enCompressMode", ctypes.c_int),
        ("u32CompressLevel", ctypes.c_uint32),
    ]


_AX_VIDEO_FRAME_FIELDS = [
    ("u32Width", ctypes.c_uint32),
    ("u32Height", ctypes.c_uint32),
    ("enImgFormat", ctypes.c_int),
    ("enVscanFormat", ctypes.c_int),
    ("stCompressInfo", AxFrameCompressInfo),
    ("stDynamicRange", ctypes.c_int),
    ("stColorGamut", ctypes.c_int),
    ("u32PicStride", ctypes.c_uint32 * 3),
    ("u32ExtStride", ctypes.c_uint32 * 3),
    ("u64PhyAddr", ctypes.c_uint64 * 3),
    ("u64VirAddr", ctypes.c_uint64 * 3),
    ("u64ExtPhyAddr", ctypes.c_uint64 * 3),
    ("u64ExtVirAddr", ctypes.c_uint64 * 3),
    ("u32HeaderSize", ctypes.c_uint32 * 3),
    ("u32BlkId", ctypes.c_uint32 * 3),
    ("s16CropX", ctypes.c_int16),
    ("s16CropY", ctypes.c_int16),
    ("s16CropWidth", ctypes.c_int16),
    ("s16CropHeight", ctypes.c_int16),
    ("u32TimeRef", ctypes.c_uint32),
    ("u64PTS", ctypes.c_uint64),
    ("u64SeqNum", ctypes.c_uint64),
    ("u64UserData", ctypes.c_uint64),
    ("u64PrivateData", ctypes.c_uint64),
    ("u32FrameFlag", ctypes.c_uint32),
    ("u32FrameSize", ctypes.c_uint32),
]


class AxVideoFrameBase(ctypes.Structure):
    """Exact 232-byte AX_VIDEO_FRAME_T used inside larger SDK structures."""

    _fields_ = _AX_VIDEO_FRAME_FIELDS


class AxVideoFrame(ctypes.Structure):
    """Standalone AX_VIDEO_FRAME_T plus an append-only ABI guard."""

    _fields_ = _AX_VIDEO_FRAME_FIELDS + [
        ("_abi_tail_guard", ctypes.c_ubyte * 512),
    ]


class AxVideoFrameInfo(ctypes.Structure):
    """AX_VIDEO_FRAME_INFO_T from ax_global_type.h."""

    _fields_ = [
        ("stVFrame", AxVideoFrameBase),
        ("enModId", ctypes.c_int),
        ("bEndOfStream", ctypes.c_int),
    ]


class AxImgInfo(ctypes.Structure):
    """AX_IMG_INFO_T plus a 512-byte guard for append-only SDK revisions.

    The capture path only consumes ``tFrameInfo.stVFrame``. ISP metadata is
    retained byte-for-byte so AX_VIN_ReleaseYuvFrame receives the exact object
    filled by AX_VIN_GetYuvFrame.
    """

    _fields_ = [
        ("tFrameInfo", AxVideoFrameInfo),
        ("tIspInfo", ctypes.c_ubyte * 336),
        ("_abi_tail_guard", ctypes.c_ubyte * 512),
    ]


AX_VIDEO_FRAME_BASE_SIZE = AxVideoFrame._abi_tail_guard.offset
AX_VIDEO_FRAME_EXPECTED_BASE_SIZE = 232
AX_VIDEO_FRAME_EXPECTED_PTS_OFFSET = 192
AX_VIDEO_FRAME_EXPECTED_SEQ_OFFSET = 200
AX_VIDEO_FRAME_INFO_EXPECTED_SIZE = 240
AX_IMG_INFO_BASE_SIZE = AxImgInfo._abi_tail_guard.offset
AX_IMG_INFO_EXPECTED_BASE_SIZE = 576
ABI_GUARD_BYTE = 0xA5


def assert_ctypes_abi() -> None:
    actual = (
        AX_VIDEO_FRAME_BASE_SIZE,
        AxVideoFrame.u64PTS.offset,
        AxVideoFrame.u64SeqNum.offset,
    )
    expected = (
        AX_VIDEO_FRAME_EXPECTED_BASE_SIZE,
        AX_VIDEO_FRAME_EXPECTED_PTS_OFFSET,
        AX_VIDEO_FRAME_EXPECTED_SEQ_OFFSET,
    )
    if actual != expected:
        raise RuntimeError(f"AX_VIDEO_FRAME_T ctypes ABI mismatch: {actual} != {expected}")
    vin_actual = (
        ctypes.sizeof(AxVideoFrameBase),
        ctypes.sizeof(AxVideoFrameInfo),
        AxVideoFrameInfo.enModId.offset,
        AxVideoFrameInfo.bEndOfStream.offset,
        AX_IMG_INFO_BASE_SIZE,
        AxImgInfo.tIspInfo.offset,
    )
    vin_expected = (
        AX_VIDEO_FRAME_EXPECTED_BASE_SIZE,
        AX_VIDEO_FRAME_INFO_EXPECTED_SIZE,
        232,
        236,
        AX_IMG_INFO_EXPECTED_BASE_SIZE,
        240,
    )
    if vin_actual != vin_expected:
        raise RuntimeError(f"AX_IMG_INFO_T ctypes ABI mismatch: {vin_actual} != {vin_expected}")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def inspect_sdk_headers(include_dir: str) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": include_dir,
        "present": False,
        "contract_match": False,
        "hashes": {},
        "errors": [],
    }
    if not include_dir:
        result["errors"].append("SDK include directory was not found")
        return result
    root = Path(include_dir)
    if not root.is_dir():
        result["errors"].append(f"SDK include directory does not exist: {root}")
        return result
    result["present"] = True
    for name, expected in SDK_HEADER_SHA256.items():
        path = root / name
        if not path.is_file():
            result["errors"].append(f"missing header: {name}")
            continue
        actual = _sha256(path)
        result["hashes"][name] = actual
        if actual.lower() != expected.lower():
            result["errors"].append(f"header hash mismatch: {name}")

    global_header = root / "ax_global_type.h"
    if global_header.is_file():
        text = global_header.read_text(encoding="utf-8", errors="strict")
        compact = "".join(text.split())
        start = compact.find("typedefstructaxVIDEO_FRAME_T{")
        end = compact.find("}AX_VIDEO_FRAME_T;", start)
        ordered = [
            "u32Width", "u32Height", "enImgFormat", "enVscanFormat",
            "stCompressInfo", "stDynamicRange", "stColorGamut", "u32PicStride",
            "u32ExtStride", "u64PhyAddr", "u64VirAddr", "u64ExtPhyAddr",
            "u64ExtVirAddr", "u32HeaderSize", "u32BlkId", "s16CropX",
            "s16CropY", "s16CropWidth", "s16CropHeight", "u32TimeRef",
            "u64PTS", "u64SeqNum", "u64UserData", "u64PrivateData",
            "u32FrameFlag", "u32FrameSize",
        ]
        if "#defineAX_MAX_COLOR_COMPONENT(3)" not in compact or start < 0 or end < 0:
            result["errors"].append("AX_VIDEO_FRAME_T contract was not found")
        else:
            block = compact[start:end]
            cursor = 0
            for field in ordered:
                position = block.find(field, cursor)
                if position < 0:
                    result["errors"].append(f"missing or reordered field: {field}")
                    break
                cursor = position + len(field)
        frame_info_contract = (
            "typedefstructaxVIDEO_FRAME_INFO_T{"
            "AX_VIDEO_FRAME_TstVFrame;AX_MOD_ID_EenModId;"
            "AX_BOOLbEndOfStream;}AX_VIDEO_FRAME_INFO_T;"
        )
        if frame_info_contract not in compact:
            result["errors"].append("AX_VIDEO_FRAME_INFO_T contract was not found")

    vin_header = root / "ax_vin_api.h"
    if vin_header.is_file():
        text = vin_header.read_text(encoding="utf-8", errors="strict")
        compact = "".join(text.split())
        img_start = compact.find("typedefstruct_AX_IMG_INFO_T_{")
        img_end = compact.find("}AX_IMG_INFO_T;", img_start)
        if img_start < 0 or img_end < 0:
            result["errors"].append("AX_IMG_INFO_T contract was not found")
        else:
            block = compact[img_start:img_end]
            frame_position = block.find("AX_VIDEO_FRAME_INFO_TtFrameInfo")
            isp_position = block.find("AX_ISP_FRAME_TtIspInfo")
            if frame_position < 0 or isp_position <= frame_position:
                result["errors"].append("AX_IMG_INFO_T fields are missing or reordered")
        get_contract = (
            "AX_S32AX_VIN_GetYuvFrame(AX_U8nPipeId,AX_VIN_CHN_ID_EeChnId,"
            "AX_IMG_INFO_T*pImgInfo,AX_S32nTimeOutMs);"
        )
        release_contract = (
            "AX_S32AX_VIN_ReleaseYuvFrame(AX_U8nPipeId,AX_VIN_CHN_ID_EeChnId,"
            "constAX_IMG_INFO_T*pImgInfo);"
        )
        if get_contract not in compact:
            result["errors"].append("AX_VIN_GetYuvFrame signature mismatch")
        if release_contract not in compact:
            result["errors"].append("AX_VIN_ReleaseYuvFrame signature mismatch")
        for name, suffix in (("GetRawFrame", "AX_IMG_INFO_T*pImgInfo,AX_S32nTimeOutMs);"),
                             ("ReleaseRawFrame", "constAX_IMG_INFO_T*pImgInfo);")):
            signature = (f"AX_S32AX_VIN_{name}(AX_U8nPipeId,AX_VIN_PIPE_DUMP_NODE_EeRawId,"
                         f"AX_SNS_HDR_FRAME_EeSnsFrame,{suffix}")
            if signature not in compact:
                result["errors"].append(f"AX_VIN_{name} signature mismatch")
    result["contract_match"] = not result["errors"]
    return result


def ax_return_code_text(value: int) -> str:
    return f"0x{ctypes.c_uint32(value).value:08x}"


def _resolve_ax_library_dir(explicit_dir: str) -> Path | None:
    candidates: list[Path] = []
    if explicit_dir:
        candidates.append(Path(explicit_dir).expanduser())
    for variable in ("BSP_MSP_DIR", "AX_CAPTURE_LIB_DIR"):
        if os.environ.get(variable):
            candidates.append(Path(os.environ[variable]).expanduser())
    candidates.append(Path("/soc/lib"))
    seen: set[str] = set()
    for candidate in candidates:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        if (candidate / "libax_sys.so").is_file() and (candidate / "libax_proton.so").is_file():
            return candidate.resolve()
    if explicit_dir:
        raise RuntimeError(
            f"AX library directory lacks libax_sys.so/libax_proton.so: {explicit_dir}"
        )
    return None


def _load_ax_libraries(
    resolved: Path | None,
) -> tuple[Any, list[Any], Any]:
    """Load VIN dependencies globally before libax_proton."""
    mode = getattr(ctypes, "RTLD_GLOBAL", 0)

    def library_name(name: str) -> str:
        return str(resolved / name) if resolved is not None else name

    sys_library = ctypes.CDLL(library_name("libax_sys.so"), mode=mode)
    dependencies: list[Any] = []
    for name in ("libax_interpreter.so", "libax_engine.so"):
        path = resolved / name if resolved is not None else None
        if path is not None and not path.is_file():
            continue
        try:
            dependencies.append(ctypes.CDLL(library_name(name), mode=mode))
        except OSError:
            if resolved is not None:
                raise
    vin_library = ctypes.CDLL(library_name("libax_proton.so"), mode=mode)
    return sys_library, dependencies, vin_library


class AxVinApi:
    """Pure-Python consumer binding for an already-running AX VIN pipeline."""

    def __init__(self, lib_dir: str, sdk_include_dir: str = "") -> None:
        assert_ctypes_abi()
        self.ax_errors: list[dict[str, Any]] = []
        self.header_contract = inspect_sdk_headers(sdk_include_dir)
        if sdk_include_dir and not self.header_contract["contract_match"]:
            raise RuntimeError(
                "SDK header contract mismatch: " + "; ".join(self.header_contract["errors"])
            )

        resolved = _resolve_ax_library_dir(lib_dir)
        if resolved is None:
            self.library_dir = "dynamic-loader search path"
        else:
            self.library_dir = str(resolved)
        try:
            if resolved is None:
                self._sys, self._dependency_handles, self._vin = _load_ax_libraries(None)
            else:
                self._sys, self._dependency_handles, self._vin = _load_ax_libraries(
                    resolved
                )
        except OSError as exc:
            raise RuntimeError(
                "failed to load AX libraries; set --ax-lib-dir or LD_LIBRARY_PATH: " + str(exc)
            ) from exc

        self._sys.AX_SYS_Init.argtypes = []
        self._sys.AX_SYS_Init.restype = ctypes.c_int32
        self._sys.AX_SYS_Deinit.argtypes = []
        self._sys.AX_SYS_Deinit.restype = ctypes.c_int32
        self._sys.AX_SYS_MmapCache.argtypes = [ctypes.c_uint64, ctypes.c_uint32]
        self._sys.AX_SYS_MmapCache.restype = ctypes.c_void_p
        self._sys.AX_SYS_MinvalidateCache.argtypes = [
            ctypes.c_uint64, ctypes.c_void_p, ctypes.c_uint32
        ]
        self._sys.AX_SYS_MinvalidateCache.restype = ctypes.c_int32
        self._sys.AX_SYS_Munmap.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self._sys.AX_SYS_Munmap.restype = ctypes.c_int32
        self._vin.AX_VIN_Init.argtypes = []
        self._vin.AX_VIN_Init.restype = ctypes.c_int32
        self._vin.AX_VIN_Deinit.argtypes = []
        self._vin.AX_VIN_Deinit.restype = ctypes.c_int32
        self._vin.AX_VIN_GetYuvFrame.argtypes = [
            ctypes.c_uint8, ctypes.c_int32, ctypes.POINTER(AxImgInfo), ctypes.c_int32
        ]
        self._vin.AX_VIN_GetYuvFrame.restype = ctypes.c_int32
        self._vin.AX_VIN_ReleaseYuvFrame.argtypes = [
            ctypes.c_uint8, ctypes.c_int32, ctypes.POINTER(AxImgInfo)
        ]
        self._vin.AX_VIN_ReleaseYuvFrame.restype = ctypes.c_int32

        self.sys_initialized = False
        self.vin_initialized = False
        result = int(self._sys.AX_SYS_Init())
        if result != 0:
            self._record_error("AX_SYS_Init", result)
            raise RuntimeError(f"AX_SYS_Init failed: {ax_return_code_text(result)}")
        self.sys_initialized = True
        result = int(self._vin.AX_VIN_Init())
        if result != 0:
            self._record_error("AX_VIN_Init", result)
            cleanup_result = int(self._sys.AX_SYS_Deinit())
            self.sys_initialized = False
            if cleanup_result != 0:
                self._record_error("AX_SYS_Deinit", cleanup_result, during="VIN init cleanup")
            raise RuntimeError(f"AX_VIN_Init failed: {ax_return_code_text(result)}")
        self.vin_initialized = True

    def _record_error(self, call: str, result: int | None, **context: Any) -> None:
        item: dict[str, Any] = {"call": call, **context}
        if result is None:
            item["result"] = "NULL"
        else:
            item["result"] = int(result)
            item["result_hex"] = ax_return_code_text(result)
        self.ax_errors.append(item)

    def get_frame(self, pipe: int, chn: int, timeout_ms: int, frame: AxVideoFrame) -> int:
        owner = AxImgInfo()
        for index in range(len(owner._abi_tail_guard)):
            owner._abi_tail_guard[index] = ABI_GUARD_BYTE
        result = int(
            self._vin.AX_VIN_GetYuvFrame(pipe, chn, ctypes.byref(owner), timeout_ms)
        )
        if result != 0:
            self._record_error(
                "AX_VIN_GetYuvFrame", result, pipe=pipe, chn=chn, timeout_ms=timeout_ms
            )
            return result
        ctypes.memmove(
            ctypes.addressof(frame),
            ctypes.addressof(owner.tFrameInfo.stVFrame),
            AX_VIDEO_FRAME_BASE_SIZE,
        )
        frame._vin_owner = owner
        if any(value != ABI_GUARD_BYTE for value in owner._abi_tail_guard):
            self._record_error(
                "AX_IMG_INFO_T_tail_guard", None, pipe=pipe, chn=chn
            )
            release_result = self.release_frame(pipe, chn, frame)
            raise RuntimeError(
                "AX_VIN_GetYuvFrame wrote beyond the pinned 576-byte AX_IMG_INFO_T"
                + (
                    f"; release returned {ax_return_code_text(release_result)}"
                    if release_result != 0
                    else ""
                )
            )
        return result

    def release_frame(self, pipe: int, chn: int, frame: AxVideoFrame) -> int:
        owner = getattr(frame, "_vin_owner", None)
        if owner is None:
            raise RuntimeError("VIN frame has no AX_IMG_INFO_T owner or was already released")
        result = int(
            self._vin.AX_VIN_ReleaseYuvFrame(pipe, chn, ctypes.byref(owner))
        )
        frame._vin_owner = None
        if result != 0:
            self._record_error(
                "AX_VIN_ReleaseYuvFrame",
                result,
                pipe=pipe,
                chn=chn,
                sequence=int(frame.u64SeqNum),
                timestamp_us=int(frame.u64PTS),
            )
        return result

    def map_frame(self, frame: AxVideoFrame) -> tuple[int, int, bool]:
        map_size = frame_map_size(frame)
        if map_size <= 0 or int(frame.u64PhyAddr[0]) == 0:
            raise RuntimeError("frame has no mappable physical buffer")
        # u64VirAddr belongs to the producer's address space and must not be
        # dereferenced by this Python consumer. Always map the physical frame.
        raw_pointer = self._sys.AX_SYS_MmapCache(int(frame.u64PhyAddr[0]), map_size)
        if not raw_pointer:
            self._record_error(
                "AX_SYS_MmapCache",
                None,
                physical_address=int(frame.u64PhyAddr[0]),
                size=map_size,
            )
            raise RuntimeError("AX_SYS_MmapCache returned NULL")
        pointer = int(raw_pointer)
        mapped_here = True
        result = int(
            self._sys.AX_SYS_MinvalidateCache(
                int(frame.u64PhyAddr[0]), ctypes.c_void_p(pointer), map_size
            )
        )
        if result != 0:
            self._record_error(
                "AX_SYS_MinvalidateCache",
                result,
                physical_address=int(frame.u64PhyAddr[0]),
                size=map_size,
            )
            if mapped_here:
                cleanup_result = int(
                    self._sys.AX_SYS_Munmap(ctypes.c_void_p(pointer), map_size)
                )
                if cleanup_result != 0:
                    self._record_error(
                        "AX_SYS_Munmap",
                        cleanup_result,
                        size=map_size,
                        during="AX_SYS_MinvalidateCache cleanup",
                    )
            raise RuntimeError(f"AX_SYS_MinvalidateCache failed: {ax_return_code_text(result)}")
        return pointer, map_size, mapped_here

    def unmap_frame(self, pointer: int, map_size: int, mapped_here: bool) -> None:
        if not mapped_here:
            return
        result = int(self._sys.AX_SYS_Munmap(ctypes.c_void_p(pointer), map_size))
        if result != 0:
            self._record_error("AX_SYS_Munmap", result, size=map_size)
            raise RuntimeError(f"AX_SYS_Munmap failed: {ax_return_code_text(result)}")

    def close(self) -> int:
        first_error = 0
        if self.vin_initialized:
            self.vin_initialized = False
            result = int(self._vin.AX_VIN_Deinit())
            if result != 0:
                self._record_error("AX_VIN_Deinit", result)
                first_error = result
        if self.sys_initialized:
            self.sys_initialized = False
            result = int(self._sys.AX_SYS_Deinit())
            if result != 0:
                self._record_error("AX_SYS_Deinit", result)
                if first_error == 0:
                    first_error = result
        return first_error

    def __enter__(self) -> "AxVinApi":
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


def frame_map_size(frame: AxVideoFrame) -> int:
    if int(frame.u32FrameSize) > 0:
        return int(frame.u32FrameSize)
    stride = int(frame.u32PicStride[0])
    height = int(frame.u32Height)
    if stride > 0 and height > 0:
        return stride * height
    return 0


def validate_frame_metadata(
    frame: AxVideoFrame,
    expected: tuple[int, int] | None = None,
    expected_format: int | None = None,
) -> None:
    width, height = int(frame.u32Width), int(frame.u32Height)
    if width <= 0 or height <= 0 or int(frame.u64PTS) <= 0:
        raise RuntimeError("invalid AX frame dimensions or PTS")
    if int(frame.u64PhyAddr[0]) == 0 and int(frame.u64VirAddr[0]) == 0:
        raise RuntimeError("AX frame has no physical or virtual address")
    if expected and (width, height) != expected:
        raise RuntimeError(
            f"VIN resolution is {width}x{height}, expected {expected[0]}x{expected[1]}; "
            "configure the producer before recording"
        )
    if int(frame.stCompressInfo.enCompressMode) != AX_COMPRESS_MODE_NONE:
        raise RuntimeError(
            f"compressed AX frame is unsupported: mode={frame.stCompressInfo.enCompressMode}"
        )
    if int(frame.enImgFormat) not in FORMAT_NAMES:
        raise RuntimeError(f"unsupported AX image format: 0x{int(frame.enImgFormat):x}")
    if expected_format is not None and int(frame.enImgFormat) != expected_format:
        raise RuntimeError(
            f"VIN image format is {FORMAT_NAMES.get(int(frame.enImgFormat), hex(int(frame.enImgFormat)))}, "
            f"expected {FORMAT_NAMES.get(expected_format, hex(expected_format))}"
        )
    stride = int(frame.u32PicStride[0])
    required = width * (3 if int(frame.enImgFormat) in (AX_FORMAT_RGB888, AX_FORMAT_BGR888) else 1)
    if stride < required:
        raise RuntimeError(f"invalid AX frame stride {stride}; need at least {required}")
    minimum_map_size = stride * height
    actual_map_size = frame_map_size(frame)
    if actual_map_size < minimum_map_size:
        raise RuntimeError(
            f"AX frame buffer is smaller than its first plane: "
            f"{actual_map_size} < {minimum_map_size}"
        )


def _gray_with_numpy(raw: bytes, width: int, height: int, stride: int, image_format: int) -> bytes:
    import numpy as np

    rows = np.frombuffer(raw, dtype=np.uint8).reshape(height, stride)
    if image_format in {
        AX_FORMAT_YUV400,
        AX_FORMAT_YUV420_PLANAR,
        AX_FORMAT_YUV420_PLANAR_VU,
        AX_FORMAT_YUV420_SEMIPLANAR,
        AX_FORMAT_YUV420_SEMIPLANAR_VU,
    }:
        return np.ascontiguousarray(rows[:, :width]).tobytes()
    pixels = rows[:, : width * 3].reshape(height, width, 3).astype(np.uint16)
    if image_format == AX_FORMAT_RGB888:  # B, G, R bytes.
        gray = (29 * pixels[:, :, 0] + 150 * pixels[:, :, 1] + 77 * pixels[:, :, 2] + 128) >> 8
    else:  # AX_FORMAT_BGR888: R, G, B bytes.
        gray = (77 * pixels[:, :, 0] + 150 * pixels[:, :, 1] + 29 * pixels[:, :, 2] + 128) >> 8
    return gray.astype(np.uint8).tobytes()


def _gray_without_numpy(raw: bytes, width: int, height: int, stride: int, image_format: int) -> bytes:
    output = bytearray(width * height)
    if image_format in {
        AX_FORMAT_YUV400,
        AX_FORMAT_YUV420_PLANAR,
        AX_FORMAT_YUV420_PLANAR_VU,
        AX_FORMAT_YUV420_SEMIPLANAR,
        AX_FORMAT_YUV420_SEMIPLANAR_VU,
    }:
        for row in range(height):
            source = row * stride
            destination = row * width
            output[destination : destination + width] = raw[source : source + width]
        return bytes(output)
    bgr_order = image_format == AX_FORMAT_RGB888
    destination = 0
    for row in range(height):
        source = row * stride
        for column in range(width):
            offset = source + column * 3
            c0, green, c2 = raw[offset], raw[offset + 1], raw[offset + 2]
            if bgr_order:
                blue, red = c0, c2
            else:
                red, blue = c0, c2
            output[destination] = (29 * blue + 150 * green + 77 * red + 128) >> 8
            destination += 1
    return bytes(output)


def packed_frame_to_gray(raw: bytes, width: int, height: int, stride: int, image_format: int) -> bytes:
    required = stride * height
    if len(raw) < required:
        raise ValueError(f"mapped frame is truncated: {len(raw)} < {required}")
    try:
        return _gray_with_numpy(raw[:required], width, height, stride, image_format)
    except ImportError:
        return _gray_without_numpy(raw[:required], width, height, stride, image_format)


def copy_y_plane(pointer: int, width: int, height: int, stride: int,
                 destination: bytearray, bottom_up: bool = False) -> None:
    """Copy mapped Y pixels once; no view of SDK memory escapes this function."""
    if width <= 0 or height <= 0 or stride < width or len(destination) != width * height:
        raise ValueError("invalid Y plane or destination dimensions")
    address = ctypes.addressof(ctypes.c_ubyte.from_buffer(destination))
    if stride == width and not bottom_up:
        ctypes.memmove(address, pointer, width * height)
        return
    try:
        import numpy as np
    except ImportError:
        for row in range(height):
            target_row = height - 1 - row if bottom_up else row
            ctypes.memmove(address + target_row * width, pointer + row * stride, width)
    else:
        mapped = (ctypes.c_ubyte * (stride * height)).from_address(pointer)
        source = np.frombuffer(mapped, dtype=np.uint8).reshape(height, stride)[:, :width]
        target = np.frombuffer(destination, dtype=np.uint8).reshape(height, width)
        np.copyto(target, source[::-1] if bottom_up else source)


@dataclass(frozen=True)
class GrayPair:
    left_timestamp_ns: int
    right_timestamp_ns: int
    left_sequence: int
    right_sequence: int
    width: int
    height: int
    left_pixels: bytes | bytearray
    right_pixels: bytes | bytearray
    left_format: int
    right_format: int
    bottom_up: bool = False

    @property
    def pair_timestamp_ns(self) -> int:
        return (self.left_timestamp_ns + self.right_timestamp_ns) // 2

    @property
    def delta_us(self) -> float:
        return abs(self.left_timestamp_ns - self.right_timestamp_ns) / 1000.0


@dataclass
class StereoFrames:
    left: AxVideoFrame
    right: AxVideoFrame
    retries: int
    released: bool = False


class AcquisitionStats:
    """Constant-space counters over successful SDK gets, before synchronization."""

    def __init__(self) -> None:
        self.previous: dict[str, int] = {}
        self.counts = {
            f"source_{side}_{metric}": 0
            for side in ("left", "right")
            for metric in ("acquired_frames", "sequence_gaps", "sequence_regressions")
        }
        self.counts.update({f"sync_{side}_discarded_frames": 0 for side in ("left", "right")})

    def acquired(self, side: str, sequence: int) -> None:
        previous = self.previous.get(side)
        self.counts[f"source_{side}_acquired_frames"] += 1
        if previous is not None:
            self.counts[f"source_{side}_sequence_gaps"] += max(0, sequence - previous - 1)
            self.counts[f"source_{side}_sequence_regressions"] += int(sequence <= previous)
        self.previous[side] = sequence

    def discarded(self, side: str) -> None:
        self.counts[f"sync_{side}_discarded_frames"] += 1

    def summary(self) -> dict[str, int | str]:
        return {
            **self.counts,
            "source_sequence_accounting": "successful_vin_get_v1",
            "source_sequence_regressions": sum(
                self.counts[f"source_{side}_sequence_regressions"] for side in ("left", "right")
            ),
            "sync_discarded_frames": sum(
                self.counts[f"sync_{side}_discarded_frames"] for side in ("left", "right")
            ),
        }


class StereoGrabber:
    def __init__(
        self,
        api: AxVinApi,
        *,
        left_pipe: int,
        left_chn: int,
        right_pipe: int,
        right_chn: int,
        timeout_ms: int,
        tolerance_us: int,
        max_retries: int,
        expected_resolution: tuple[int, int],
        expected_format: int | None = AX_FORMAT_YUV420_SEMIPLANAR,
    ) -> None:
        self.api = api
        self.left_pipe = left_pipe
        self.left_chn = left_chn
        self.right_pipe = right_pipe
        self.right_chn = right_chn
        self.timeout_ms = timeout_ms
        self.tolerance_us = tolerance_us
        self.max_retries = max_retries
        self.expected_resolution = expected_resolution
        self.expected_format = expected_format
        self.timings = Timings()
        self.acquisition_stats = AcquisitionStats()
        self.sequence_window: AcquisitionStats | None = None
        self.stats: dict[str, int] = {
            "get_left_calls": 0,
            "get_right_calls": 0,
            "get_left_errors": 0,
            "get_right_errors": 0,
            "release_errors": 0,
            "sync_retries": 0,
            "sync_failures": 0,
        }

    def start_sequence_window(self) -> None:
        # The first successful get on each side establishes a fresh baseline.
        self.sequence_window = AcquisitionStats()

    def _sync_discard(self, side: str) -> None:
        self.acquisition_stats.discarded(side)
        if self.sequence_window is not None:
            self.sequence_window.discarded(side)

    def _get(self, side: str) -> AxVideoFrame:
        frame = AxVideoFrame()
        if side == "left":
            pipe, chn = self.left_pipe, self.left_chn
        else:
            pipe, chn = self.right_pipe, self.right_chn
        self.stats[f"get_{side}_calls"] += 1
        result = self.api.get_frame(pipe, chn, self.timeout_ms, frame)
        if result != 0:
            self.stats[f"get_{side}_errors"] += 1
            raise RuntimeError(f"AX_VIN_GetYuvFrame({side}) failed: {ax_return_code_text(result)}")
        self.acquisition_stats.acquired(side, int(frame.u64SeqNum))
        if self.sequence_window is not None:
            self.sequence_window.acquired(side, int(frame.u64SeqNum))
        return frame

    def _release_one(self, side: str, frame: AxVideoFrame) -> None:
        pipe, chn = (
            (self.left_pipe, self.left_chn)
            if side == "left"
            else (self.right_pipe, self.right_chn)
        )
        result = self.api.release_frame(pipe, chn, frame)
        if result != 0:
            self.stats["release_errors"] += 1
            raise RuntimeError(
                f"AX_VIN_ReleaseYuvFrame({side}) failed: {ax_return_code_text(result)}"
            )

    def grab_frames(self) -> StereoFrames:
        started = time.perf_counter_ns()
        left: AxVideoFrame | None = None
        right: AxVideoFrame | None = None
        try:
            left = self._get("left")
            try:
                right = self._get("right")
            except BaseException:
                old_left = left
                left = None
                self._release_one("left", old_left)
                raise
            retries = 0
            while abs(int(left.u64PTS) - int(right.u64PTS)) > self.tolerance_us:
                if retries >= self.max_retries:
                    self.stats["sync_failures"] += 1
                    raise RuntimeError(
                        f"stereo PTS mismatch after {retries} retries: "
                        f"left={left.u64PTS}, right={right.u64PTS}"
                    )
                retries += 1
                self.stats["sync_retries"] += 1
                if int(left.u64PTS) < int(right.u64PTS):
                    self._sync_discard("left")
                    old_left = left
                    left = None
                    self._release_one("left", old_left)
                    left = self._get("left")
                else:
                    self._sync_discard("right")
                    old_right = right
                    right = None
                    self._release_one("right", old_right)
                    right = self._get("right")
            self.validate_frame(left)
            self.validate_frame(right)
            if (int(left.u32Width), int(left.u32Height)) != (
                int(right.u32Width), int(right.u32Height)
            ):
                raise RuntimeError("left/right VIN resolutions differ")
            return StereoFrames(left=left, right=right, retries=retries)
        except BaseException:
            release_error: BaseException | None = None
            if left is not None:
                try:
                    self._release_one("left", left)
                except BaseException as exc:
                    release_error = exc
            if right is not None:
                try:
                    self._release_one("right", right)
                except BaseException as exc:
                    release_error = release_error or exc
            if release_error is not None:
                raise release_error
            raise
        finally:
            self.timings.add("grab_and_sync", time.perf_counter_ns() - started)

    def validate_frame(self, frame: AxVideoFrame) -> None:
        validate_frame_metadata(frame, self.expected_resolution, self.expected_format)

    def release_frames(self, frames: StereoFrames) -> None:
        if frames.released:
            raise RuntimeError("stereo frames were already released")
        errors: list[BaseException] = []
        for side, frame in (("left", frames.left), ("right", frames.right)):
            try:
                self._release_one(side, frame)
            except BaseException as exc:
                errors.append(exc)
        frames.released = True
        if errors:
            for error in errors:
                if not isinstance(error, Exception):
                    raise error
            raise RuntimeError("; ".join(str(error) for error in errors))

    def copy_gray_pair(
        self, frames: StereoFrames, *, release_after_copy: bool = False,
        buffers: tuple[bytearray, bytearray] | None = None,
        bottom_up: bool = False,
    ) -> GrayPair:
        width, height = int(frames.left.u32Width), int(frames.left.u32Height)
        def copy_gray(frame: AxVideoFrame, destination: bytearray) -> bytearray:
            start = time.perf_counter_ns()
            pointer, map_size, mapped_here = self.api.map_frame(frame)
            self.timings.add("map_and_invalidate", time.perf_counter_ns() - start)
            try:
                start = time.perf_counter_ns()
                stride, image_format = int(frame.u32PicStride[0]), int(frame.enImgFormat)
                if image_format in (AX_FORMAT_RGB888, AX_FORMAT_BGR888):
                    raw = ctypes.string_at(pointer, stride * height)
                    gray = packed_frame_to_gray(raw, width, height, stride, image_format)
                    if bottom_up:
                        for row in range(height):
                            offset = (height - 1 - row) * width
                            destination[row * width:(row + 1) * width] = gray[offset:offset + width]
                    else:
                        destination[:] = gray
                else:
                    copy_y_plane(pointer, width, height, stride, destination, bottom_up)
                self.timings.add("copy_y", time.perf_counter_ns() - start)
                return destination
            finally:
                start = time.perf_counter_ns()
                self.api.unmap_frame(pointer, map_size, mapped_here)
                self.timings.add("unmap", time.perf_counter_ns() - start)

        # Allocation and copy failures are also covered by the frame-release path.
        try:
            if buffers is None:
                buffers = (bytearray(width * height), bytearray(width * height))
            return GrayPair(
                left_timestamp_ns=int(frames.left.u64PTS) * 1000,
                right_timestamp_ns=int(frames.right.u64PTS) * 1000,
                left_sequence=int(frames.left.u64SeqNum),
                right_sequence=int(frames.right.u64SeqNum),
                width=width, height=height,
                left_pixels=copy_gray(frames.left, buffers[0]),
                right_pixels=copy_gray(frames.right, buffers[1]),
                left_format=int(frames.left.enImgFormat),
                right_format=int(frames.right.enImgFormat),
                bottom_up=bottom_up,
            )
        finally:
            if release_after_copy:
                self.release_frames(frames)

    def describe_frames(self, frames: StereoFrames) -> dict[str, Any]:
        def one(frame: AxVideoFrame) -> dict[str, Any]:
            return {
                "width": int(frame.u32Width),
                "height": int(frame.u32Height),
                "format": int(frame.enImgFormat),
                "format_name": FORMAT_NAMES.get(int(frame.enImgFormat), "unknown"),
                "stride": int(frame.u32PicStride[0]),
                "frame_size": int(frame.u32FrameSize),
                "compress_mode": int(frame.stCompressInfo.enCompressMode),
                "timestamp_us": int(frame.u64PTS),
                "sequence": int(frame.u64SeqNum),
            }

        return {
            "left": one(frames.left),
            "right": one(frames.right),
            "delta_us": abs(int(frames.left.u64PTS) - int(frames.right.u64PTS)),
            "sync_retries": frames.retries,
        }
