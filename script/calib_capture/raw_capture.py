"""Experimental IFE/L RAW capture; deliberately separate from Kalibr images."""
from __future__ import annotations

import ctypes
import json
import time
from pathlib import Path

from .axera import (ABI_GUARD_BYTE, AX_VIDEO_FRAME_BASE_SIZE, AxImgInfo,
                    AxVinApi, StereoGrabber, ax_return_code_text)
from .config import APP_VERSION

IFE = 0
LONG_EXPOSURE = 0
COMPRESSION_NAMES = {0: "AX_COMPRESS_MODE_NONE", 1: "AX_COMPRESS_MODE_LOSSLESS"}
RAW_FORMATS = dict(enumerate(("RAW8", "RAW10", "RAW12", "RAW14", "RAW16",
                              "RAW10_PACKED", "RAW12_PACKED", "RAW14_PACKED"), 0x80))


class RawApi(AxVinApi):
    def __init__(self, *args):
        super().__init__(*args)
        try:
            self._vin.AX_VIN_GetRawFrame.argtypes = [
                ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32,
                ctypes.POINTER(AxImgInfo), ctypes.c_int32]
            self._vin.AX_VIN_GetRawFrame.restype = ctypes.c_int32
            self._vin.AX_VIN_ReleaseRawFrame.argtypes = [
                ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32, ctypes.POINTER(AxImgInfo)]
            self._vin.AX_VIN_ReleaseRawFrame.restype = ctypes.c_int32
        except BaseException:
            self.close()
            raise

    def get_frame(self, pipe, chn, timeout_ms, frame):
        owner = AxImgInfo()
        owner._abi_tail_guard[:] = bytes([ABI_GUARD_BYTE]) * 512
        result = int(self._vin.AX_VIN_GetRawFrame(
            pipe, IFE, LONG_EXPOSURE, ctypes.byref(owner), timeout_ms))
        if result:
            self._record_error("AX_VIN_GetRawFrame", result, pipe=pipe)
            raise RuntimeError(f"AX_VIN_GetRawFrame(pipe={pipe}, IFE, L): "
                               f"{ax_return_code_text(result)}; check upstream RAW availability")
        ctypes.memmove(ctypes.addressof(frame), ctypes.addressof(owner.tFrameInfo.stVFrame),
                       AX_VIDEO_FRAME_BASE_SIZE)
        frame._vin_owner = owner
        frame._raw_pipe = pipe
        if any(x != ABI_GUARD_BYTE for x in owner._abi_tail_guard):
            self.release_frame(pipe, chn, frame)
            raise RuntimeError("RAW AX_IMG_INFO_T ABI tail guard overwritten")
        return 0

    def release_frame(self, pipe, chn, frame):
        owner = getattr(frame, "_vin_owner", None)
        if owner is None or frame._raw_pipe != pipe:
            raise RuntimeError("RAW ownership mismatch or double release")
        frame._vin_owner = None  # A failed release must not be retried implicitly.
        result = int(self._vin.AX_VIN_ReleaseRawFrame(
            pipe, IFE, LONG_EXPOSURE, ctypes.byref(owner)))
        if result:
            self._record_error("AX_VIN_ReleaseRawFrame", result, pipe=pipe)
            raise RuntimeError(f"AX_VIN_ReleaseRawFrame: {ax_return_code_text(result)}")
        return 0


class RawGrabber(StereoGrabber):
    def validate_frame(self, frame):
        if (frame.u32Width, frame.u32Height) != self.expected_resolution:
            raise RuntimeError("RAW resolution differs from --resolution; producer config is unchanged")
        mode = int(frame.stCompressInfo.enCompressMode)
        if frame.enImgFormat not in RAW_FORMATS or mode not in COMPRESSION_NAMES:
            raise RuntimeError(
                f"expected Bayer RAW(0x80..0x87), NONE(0)/LOSSLESS(1); actual format={frame.enImgFormat:#x} "
                f"({RAW_FORMATS.get(frame.enImgFormat, 'unknown')}), compression={mode} "
                f"({COMPRESSION_NAMES.get(mode, 'unsupported')}); configure the producer first")
        if not 0 < frame.u64PTS * 1000 < 2**63 or not frame.u64PhyAddr[0]:
            raise RuntimeError("invalid RAW timestamp or physical address")
        # Save the SDK buffer verbatim, including row padding. Never infer packing
        # from nominal bit depth or treat stride as a mono8 row length.
        minimum_size = 1 if mode == 1 else frame.u32Width * frame.u32Height
        if not frame.u32PicStride[0] or not (
                minimum_size <= frame.u32FrameSize <= 256 * 1024**2):
            raise RuntimeError("invalid RAW stride/frame size (maximum 256 MiB)")
        if any(frame.u32HeaderSize) or any(frame.u64ExtPhyAddr):
            raise RuntimeError("RAW auxiliary/header buffers are not supported")

    def copy_raw(self, frame):
        pointer, size, mapped = self.api.map_frame(frame)
        try:
            return ctypes.string_at(pointer, size)
        finally:
            self.api.unmap_frame(pointer, size, mapped)


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")
    temporary.replace(path)


def record_raw_triggered(config, *, input_function=input, api_factory=RawApi):
    """One bounded pair in memory, SDK ownership ends before file writes."""
    if config.left_pipe == config.right_pipe or not all(
            0 <= p <= 255 for p in (config.left_pipe, config.right_pipe)):
        raise ValueError("RAW requires two different uint8 pipe IDs; no channel selector")
    final = Path(config.output).expanduser().absolute()
    staging, failed = Path(str(final) + ".inprogress"), Path(str(final) + ".failed")
    if any(p.exists() for p in (final, staging, failed)):
        raise FileExistsError(f"RAW session already exists: {final}")
    staging.mkdir(parents=True)
    api = None
    grabber = None
    session = dict(schema_version=APP_VERSION, kind="raw_capture_session",
                   dataset_id=final.name, mode="stereo-raw-triggered", config=config.to_dict(),
                   raw_node="IFE", exposure="L", saved_pairs=0,
                   storage="SDK u32FrameSize bytes, unchanged; padding retained",
                   compression_policy="NONE or SDK LOSSLESS; opaque bytes, no decompression",
                   decompression_performed=False,
                   observed_compression_modes={"left": [], "right": []},
                   bayer_pattern="unknown (not supplied by AX_VIDEO_FRAME_T)",
                   kalibr_compatible=False)
    try:
        (staging / "meta").mkdir()
        for cam in ("cam0", "cam1"):
            (staging / "cameras" / cam).mkdir(parents=True)
        api = api_factory(config.ax_lib_dir, config.sdk_include_dir)
        grabber = RawGrabber(api, left_pipe=config.left_pipe, left_chn=0,
            right_pipe=config.right_pipe, right_chn=0, timeout_ms=config.capture_timeout_ms,
            tolerance_us=config.sync_tolerance_us, max_retries=config.max_sync_retries,
            expected_resolution=config.resolution)
        previous = [0, 0]
        with (staging / "meta/pairs.jsonl").open("w", encoding="utf-8") as pairs:
            for index in range(config.poses):
                command = input_function(f"[{index+1}/{config.poses}] Enter: save RAW pair; q: abort: ").strip().lower()
                if command in {"q", "quit", "exit"}:
                    raise RuntimeError("operator aborted before all poses completed")
                if command:
                    raise ValueError("expected Enter or q")
                # Consume pending output after Enter without mapping. Waiting at
                # the prompt holds no SDK frame; do not save its oldest queue entry.
                drain_until = time.monotonic() + 0.2
                while time.monotonic() < drain_until:
                    pending = grabber.grab_frames()
                    grabber.release_frames(pending)
                frames = grabber.grab_frames()
                try:
                    metadata = grabber.describe_frames(frames)
                    payloads = []
                    for side, frame in zip(("left", "right"), (frames.left, frames.right)):
                        metadata[side]["format_name"] = RAW_FORMATS[frame.enImgFormat]
                        mode = int(frame.stCompressInfo.enCompressMode)
                        metadata[side]["compress_mode_name"] = COMPRESSION_NAMES[mode]
                        metadata[side]["compress_level"] = int(frame.stCompressInfo.u32CompressLevel)
                        metadata[side]["requires_sdk_decompression"] = mode != 0
                        metadata[side]["header_sizes"] = list(frame.u32HeaderSize)
                        metadata[side]["ext_strides"] = list(frame.u32ExtStride)
                        metadata[side]["strides"] = list(frame.u32PicStride)
                        metadata[side]["crop"] = [frame.s16CropX, frame.s16CropY,
                                                  frame.s16CropWidth, frame.s16CropHeight]
                        payloads.append(grabber.copy_raw(frame))
                finally:
                    grabber.release_frames(frames)
                timestamps = [metadata[s]["timestamp_us"] * 1000 for s in ("left", "right")]
                if any(now <= old for now, old in zip(timestamps, previous)):
                    raise RuntimeError("RAW timestamps are not strictly increasing")
                files = [staging / "cameras" / f"cam{i}" / f"{ts}.raw"
                         for i, ts in enumerate(timestamps)]
                for path, payload in zip(files, payloads):
                    path.with_suffix(".raw.tmp").write_bytes(payload)
                for path in files:
                    path.with_suffix(".raw.tmp").replace(path)
                metadata.update(schema_version=APP_VERSION, kind="raw_capture_pair",
                                pair_index=index, timestamp_ns=timestamps,
                                files=[p.relative_to(staging).as_posix() for p in files])
                pairs.write(json.dumps(metadata) + "\n")
                pairs.flush()
                previous = timestamps
                session["saved_pairs"] += 1
                for side in ("left", "right"):
                    modes = session["observed_compression_modes"][side]
                    if metadata[side]["compress_mode"] not in modes:
                        modes.append(metadata[side]["compress_mode"])
                print(f"Saved RAW pair {index+1}: delta={metadata['delta_us']} us")
        if api.close():
            raise RuntimeError("AX shutdown failed")
        # Read back the index and lengths, not hashes or guessed RAW pixel decoding.
        with (staging / "meta/pairs.jsonl").open(encoding="utf-8") as source:
            records = 0
            for line in source:
                pair = json.loads(line)
                for side, name in zip(("left", "right"), pair["files"]):
                    if (staging / name).stat().st_size != pair[side]["frame_size"]:
                        raise RuntimeError("RAW file size mismatch")
                records += 1
            if records != config.poses:
                raise RuntimeError("RAW pair count mismatch")
        session["status"] = "completed"
    except BaseException as exc:
        session.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        raise RuntimeError(f"RAW capture failed; retained at {failed}: {exc}") from exc
    finally:
        if api is not None:
            try:
                if api.close():
                    session.update(status="failed", shutdown_error="AX shutdown failed")
            except BaseException as exc:
                session.update(status="failed", shutdown_error=str(exc))
            session.update(sdk=api.header_contract, library_dir=api.library_dir, ax_errors=api.ax_errors)
        if grabber is not None:
            session["acquisition_stats"] = grabber.acquisition_stats.summary()
        write_json(staging / "meta/session.json", session)
        staging.rename(final if session.get("status") == "completed" else failed)
    if session.get("status") != "completed":
        raise RuntimeError(f"RAW capture failed; retained at {failed}")
    return final
