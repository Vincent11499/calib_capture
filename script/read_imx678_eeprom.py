#!/usr/bin/env python3
"""Read and decode the stereo IMX678 module EEPROM through i2ctransfer.

The layout is defined by the module EEPROM table supplied with this project:
0x0000..0x0126 contain module identity and calibration data, and the big-endian
CRC_SUM32 checksum is stored at 0x0127..0x012A.  The script is read-only: it never issues an
EEPROM write-data transaction; the two written bytes only select the read
offset before a repeated-start read.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional, Sequence


DEFAULT_EEPROM_ADDRESS = 0x50
DEFAULT_CHUNK_SIZE = 32
CHECKSUM_ADDRESS = 0x0127
CHECKSUM_DATA_START = 0x0000
EEPROM_END = 0x012B  # Exclusive: last stored byte is 0x012A.


class EepromError(RuntimeError):
    """A concise hardware, transport, layout, or data error."""


@dataclass(frozen=True)
class FieldSpec:
    """One field from the authoritative EEPROM address table."""

    name: str
    description: str
    address: int
    length: int
    kind: str
    endian: str = "little"
    expected: Any = None

    @property
    def end(self) -> int:
        return self.address + self.length


# ASCII values preserve byte order as written.  The source table calls that
# order "Big"; numeric byte order is relevant only to uint32/double fields.
FIELD_SPECS: tuple[FieldSpec, ...] = (
    FieldSpec("sn", "module serial number", 0x0000, 32, "ascii", "big"),
    FieldSpec("sensor_model", "sensor model", 0x0020, 20, "ascii", "big", "SONY_IMX678"),
    FieldSpec("lens_model", "lens model", 0x0034, 10, "ascii", "big", "9011"),
    FieldSpec("hardware_revision", "hardware revision", 0x003E, 3, "ascii", "big", "A03"),
    FieldSpec("firmware_version", "firmware version", 0x0041, 20, "ascii", "big", "FW_IMX678_1.4.2"),
    FieldSpec(
        "calibration_algorithm_version", "calibration algorithm version",
        0x0055, 20, "ascii", "big", "CAL_INTRINSIC_2.3.1",
    ),
    FieldSpec("year", "calibration year", 0x0069, 1, "u8"),
    FieldSpec("month", "calibration month", 0x006A, 1, "u8"),
    FieldSpec("day", "calibration day", 0x006B, 1, "u8"),
    FieldSpec("hour", "calibration hour", 0x006C, 1, "u8"),
    FieldSpec("minute", "calibration minute", 0x006D, 1, "u8"),
    FieldSpec("second", "calibration second", 0x006E, 1, "u8"),
    FieldSpec("image_width", "calibration image width", 0x006F, 4, "u32", "little", 3840),
    FieldSpec("image_height", "calibration image height", 0x0073, 4, "u32", "little", 2160),
    FieldSpec("distortion_model", "distortion model", 0x0077, 7, "ascii", "big", "RADTAN8"),
    FieldSpec("coefficient_count", "distortion coefficient count", 0x007E, 1, "u8", "little", 8),
    FieldSpec("fx", "focal length x in pixels", 0x007F, 8, "f64"),
    FieldSpec("fy", "focal length y in pixels", 0x0087, 8, "f64"),
    FieldSpec("cx", "principal point x in pixels", 0x008F, 8, "f64"),
    FieldSpec("cy", "principal point y in pixels", 0x0097, 8, "f64"),
    FieldSpec("k1", "radial distortion k1", 0x009F, 8, "f64"),
    FieldSpec("k2", "radial distortion k2", 0x00A7, 8, "f64"),
    FieldSpec("k3", "radial distortion k3", 0x00AF, 8, "f64"),
    FieldSpec("p1", "tangential distortion p1", 0x00B7, 8, "f64"),
    FieldSpec("p2", "tangential distortion p2", 0x00BF, 8, "f64"),
    FieldSpec("k4", "radial distortion k4", 0x00C7, 8, "f64"),
    FieldSpec("k5", "radial distortion k5", 0x00CF, 8, "f64"),
    FieldSpec("k6", "radial distortion k6", 0x00D7, 8, "f64"),
    FieldSpec("cal_rms_px", "mean reprojection error", 0x00DF, 8, "f64"),
    FieldSpec("cal_p95_px", "reprojection error P95", 0x00E7, 8, "f64"),
    FieldSpec("cal_linearity_p95_px", "max linearity P95", 0x00EF, 8, "f64"),
    FieldSpec("cal_max_px", "max single reprojection error", 0x00F7, 8, "f64"),
    FieldSpec("val_rms_px", "validation mean reprojection error", 0x00FF, 8, "f64"),
    FieldSpec("val_p95_px", "validation reprojection error P95", 0x0107, 8, "f64"),
    FieldSpec("val_linearity_p95_px", "validation max linearity P95", 0x010F, 8, "f64"),
    FieldSpec("val_max_px", "validation max single reprojection error", 0x0117, 8, "f64"),
    FieldSpec("val_detection_rate_pct", "validation detection coverage", 0x011F, 8, "f64"),
    FieldSpec("checksum_sum32", "CRC_SUM32 of bytes 0x0000..0x0126", 0x0127, 4, "u32", "big"),
)


def validate_layout(specs: Sequence[FieldSpec] = FIELD_SPECS) -> None:
    """Fail fast if the static field table has a gap, overlap, or wrong end."""
    cursor = 0
    for spec in specs:
        if spec.address != cursor:
            relation = "gap" if spec.address > cursor else "overlap"
            raise EepromError(
                f"layout {relation} before {spec.name}: expected 0x{cursor:04X}, "
                f"got 0x{spec.address:04X}"
            )
        if spec.length <= 0:
            raise EepromError(f"field {spec.name} has invalid length {spec.length}")
        cursor = spec.end
    if cursor != EEPROM_END:
        raise EepromError(
            f"layout ends at 0x{cursor:04X}, expected 0x{EEPROM_END:04X}"
        )


def parse_int(text: str) -> int:
    """Accept decimal or a 0x-prefixed integer from the CLI."""
    try:
        return int(text, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid integer: {text}") from exc


def _parse_transfer_output(output: bytes, expected_length: int) -> bytes:
    """Parse the normal `i2ctransfer` output: space-separated 0xNN bytes."""
    tokens = re.findall(rb"(?<![0-9A-Fa-f])0[xX]([0-9A-Fa-f]{2})(?![0-9A-Fa-f])", output)
    data = bytes(int(token, 16) for token in tokens)
    if len(data) != expected_length:
        text = output.decode("ascii", "replace").strip()
        raise EepromError(
            f"i2ctransfer returned {len(data)} bytes, expected {expected_length}: {text}"
        )
    return data


class I2cTransferReader:
    """Read a 16-bit-addressed EEPROM in bounded chunks via i2ctransfer."""

    def __init__(
        self,
        bus: int,
        address: int = DEFAULT_EEPROM_ADDRESS,
        chunk_size: int = DEFAULT_CHUNK_SIZE,
        executable: str = "i2ctransfer",
        runner: Callable[..., Any] = subprocess.run,
    ) -> None:
        if bus < 0:
            raise EepromError("I2C bus number must be nonnegative")
        if not 0x03 <= address <= 0x77:
            raise EepromError("EEPROM address must be a usable 7-bit I2C address")
        if not 1 <= chunk_size <= 4096:
            raise EepromError("chunk size must be in the range 1..4096")
        self.bus = bus
        self.address = address
        self.chunk_size = chunk_size
        self.executable = executable
        self.runner = runner

    def _read_chunk(self, offset: int, length: int) -> bytes:
        # w2 only selects the EEPROM's 16-bit internal address.  rN is the
        # actual read, joined by i2ctransfer into one repeated-start transfer.
        command = [
            self.executable,
            "-y",
            str(self.bus),
            f"w2@0x{self.address:02x}",
            f"0x{(offset >> 8) & 0xFF:02x}",
            f"0x{offset & 0xFF:02x}",
            f"r{length}",
        ]
        try:
            completed = self.runner(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
            )
        except FileNotFoundError as exc:
            raise EepromError(
                "i2ctransfer was not found; the board needs the i2c-tools command"
            ) from exc
        except subprocess.CalledProcessError as exc:
            detail = (exc.stdout or b"").decode("utf-8", "replace").strip()
            raise EepromError(
                f"i2ctransfer failed at EEPROM offset 0x{offset:04X}"
                + (f": {detail}" if detail else "")
            ) from exc
        return _parse_transfer_output(completed.stdout, length)

    def read(self, offset: int, length: int) -> bytes:
        if offset < 0 or length < 0 or offset + length > 0x10000:
            raise EepromError("EEPROM read range must fit a 16-bit address space")
        result = bytearray()
        while len(result) < length:
            position = offset + len(result)
            part_length = min(self.chunk_size, length - len(result))
            result.extend(self._read_chunk(position, part_length))
        return bytes(result)


def crc_sum32(data: bytes) -> int:
    """Module CRC_SUM32: unsigned sum of covered bytes, modulo 2**32."""
    return sum(data) & 0xFFFFFFFF


def _decode_ascii(data: bytes) -> str:
    # NUL terminates the string.  Unprogrammed 0xFF and padding spaces are
    # removed only from the end so that internal characters remain visible.
    value = data.split(b"\x00", 1)[0].rstrip(b"\xFF ")
    return value.decode("ascii", "replace")


def decode_field(raw: bytes, spec: FieldSpec) -> Any:
    data = raw[spec.address:spec.end]
    if len(data) != spec.length:
        raise EepromError(f"raw EEPROM is too short for field {spec.name}")
    if spec.kind == "ascii":
        return _decode_ascii(data)
    if spec.kind == "u8":
        return data[0]
    if spec.kind == "u32":
        return int.from_bytes(data, byteorder=spec.endian, signed=False)
    if spec.kind == "f64":
        return struct.unpack("<d" if spec.endian == "little" else ">d", data)[0]
    raise EepromError(f"unsupported field type {spec.kind!r} for {spec.name}")


@dataclass(frozen=True)
class ParsedEeprom:
    fields: dict[str, Any]
    warnings: tuple[str, ...]
    stored_checksum: int
    calculated_checksum: int

    @property
    def checksum_matches(self) -> bool:
        return self.stored_checksum == self.calculated_checksum


def parse_eeprom(raw: bytes) -> ParsedEeprom:
    """Decode all fields and perform non-destructive consistency checks."""
    validate_layout()
    if len(raw) != EEPROM_END:
        raise EepromError(f"EEPROM dump must contain exactly {EEPROM_END} bytes")

    fields = {spec.name: decode_field(raw, spec) for spec in FIELD_SPECS}
    warnings: list[str] = []
    for spec in FIELD_SPECS:
        value = fields[spec.name]
        if spec.expected is not None and value != spec.expected:
            warnings.append(
                f"{spec.name}: expected {spec.expected!r}, read {value!r}"
            )
        if spec.kind == "f64" and not math.isfinite(value):
            warnings.append(f"{spec.name}: value is not finite ({value!r})")

    date_limits = {
        "month": (1, 12), "day": (1, 31), "hour": (0, 23),
        "minute": (0, 59), "second": (0, 59),
    }
    for name, (minimum, maximum) in date_limits.items():
        if not minimum <= fields[name] <= maximum:
            warnings.append(f"{name}: value {fields[name]} is outside {minimum}..{maximum}")

    calculated = crc_sum32(raw[CHECKSUM_DATA_START:CHECKSUM_ADDRESS])
    return ParsedEeprom(
        fields=fields,
        warnings=tuple(warnings),
        stored_checksum=fields["checksum_sum32"],
        calculated_checksum=calculated,
    )


def hexdump(raw: bytes, width: int = 16) -> Iterable[str]:
    for offset in range(0, len(raw), width):
        chunk = raw[offset:offset + width]
        yield f"{offset:04X}: " + " ".join(f"{value:02X}" for value in chunk)


def _text_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.12g}"
    return str(value)


def print_text(result: ParsedEeprom, *, bus: int, address: int) -> None:
    print(
        f"bus=/dev/i2c-{bus} eeprom=0x{address:02X} "
        f"layout=0x0000..0x{EEPROM_END - 1:04X} ({EEPROM_END} bytes)"
    )
    print("---- decoded EEPROM ----")
    for spec in FIELD_SPECS:
        value = result.fields[spec.name]
        suffix = ""
        if spec.expected is not None:
            suffix = "  [MATCH]" if value == spec.expected else f"  [expected {spec.expected}]"
        print(
            f"0x{spec.address:04X}  {spec.name:<32} "
            f"{_text_value(value)}{suffix}"
        )
    print("---- integrity ----")
    print(
        f"CRC_SUM32 stored=0x{result.stored_checksum:08X} "
        f"calculated[0x{CHECKSUM_DATA_START:04X}..0x{CHECKSUM_ADDRESS - 1:04X}]="
        f"0x{result.calculated_checksum:08X} "
        f"{'MATCH' if result.checksum_matches else 'MISMATCH'}"
    )
    for warning in result.warnings:
        print(f"WARN: {warning}")


def print_json(result: ParsedEeprom, *, bus: int, address: int) -> None:
    # Strict JSON has no NaN/Infinity values.  Keep the report machine-readable
    # and expose malformed floating-point EEPROM fields as null plus a warning.
    json_fields = {
        name: (None if isinstance(value, float) and not math.isfinite(value) else value)
        for name, value in result.fields.items()
    }
    document = {
        "bus": bus,
        "device": f"/dev/i2c-{bus}",
        "address_7bit": f"0x{address:02X}",
        "layout": {
            "start": "0x0000",
            "end_inclusive": f"0x{EEPROM_END - 1:04X}",
            "length": EEPROM_END,
        },
        "fields": json_fields,
        "checksum": {
            "algorithm": "CRC_SUM32 (unsigned byte sum modulo 2^32)",
            "data_range": f"0x{CHECKSUM_DATA_START:04X}..0x{CHECKSUM_ADDRESS - 1:04X}",
            "stored": f"0x{result.stored_checksum:08X}",
            "calculated": f"0x{result.calculated_checksum:08X}",
            "matches": result.checksum_matches,
        },
        "warnings": list(result.warnings),
    }
    print(json.dumps(document, indent=2, ensure_ascii=False, allow_nan=False))


def _yaml_scalar(value: Any) -> str:
    """Render the small scalar subset used by our dependency-free YAML file."""
    if isinstance(value, str):
        # A JSON string is also a valid YAML 1.2 quoted scalar.
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        # repr() is Python's shortest round-trip representation: compact while
        # preserving the exact decoded IEEE-754 value when parsed again.
        return repr(value) if math.isfinite(value) else "null"
    if value is None:
        return "null"
    raise EepromError(f"cannot write {type(value).__name__} as a YAML scalar")


def render_camera_yaml(
    side: str,
    bus: int,
    result: ParsedEeprom,
) -> str:
    """Build one concise, self-contained YAML file for a single camera."""
    field = result.fields
    timestamp = (
        f"{field['year']:02d}-{field['month']:02d}-{field['day']:02d}T"
        f"{field['hour']:02d}:{field['minute']:02d}:{field['second']:02d}"
    )
    lines = [
        f"camera: {_yaml_scalar(side)}",
        f"i2c_bus: {bus}",
        f"serial_number: {_yaml_scalar(field['sn'])}",
        f"sensor_model: {_yaml_scalar(field['sensor_model'])}",
        f"lens_model: {_yaml_scalar(field['lens_model'])}",
        f"hardware_revision: {_yaml_scalar(field['hardware_revision'])}",
        f"firmware_version: {_yaml_scalar(field['firmware_version'])}",
        f"calibration_algorithm_version: "
        f"{_yaml_scalar(field['calibration_algorithm_version'])}",
        f"calibration_datetime_raw: {_yaml_scalar(timestamp)}",
        "image:",
        f"  width: {_yaml_scalar(field['image_width'])}",
        f"  height: {_yaml_scalar(field['image_height'])}",
        "intrinsics:",
        f"  fx: {_yaml_scalar(field['fx'])}",
        f"  fy: {_yaml_scalar(field['fy'])}",
        f"  cx: {_yaml_scalar(field['cx'])}",
        f"  cy: {_yaml_scalar(field['cy'])}",
        "distortion:",
        f"  model: {_yaml_scalar(field['distortion_model'])}",
        f"  coefficient_count: {_yaml_scalar(field['coefficient_count'])}",
        f"  k1: {_yaml_scalar(field['k1'])}",
        f"  k2: {_yaml_scalar(field['k2'])}",
        f"  k3: {_yaml_scalar(field['k3'])}",
        f"  p1: {_yaml_scalar(field['p1'])}",
        f"  p2: {_yaml_scalar(field['p2'])}",
        f"  k4: {_yaml_scalar(field['k4'])}",
        f"  k5: {_yaml_scalar(field['k5'])}",
        f"  k6: {_yaml_scalar(field['k6'])}",
        "errors:",
        "  calibration:",
        f"    rms_px: {_yaml_scalar(field['cal_rms_px'])}",
        f"    p95_px: {_yaml_scalar(field['cal_p95_px'])}",
        f"    linearity_p95_px: {_yaml_scalar(field['cal_linearity_p95_px'])}",
        f"    max_px: {_yaml_scalar(field['cal_max_px'])}",
        "  validation:",
        f"    rms_px: {_yaml_scalar(field['val_rms_px'])}",
        f"    p95_px: {_yaml_scalar(field['val_p95_px'])}",
        f"    linearity_p95_px: {_yaml_scalar(field['val_linearity_p95_px'])}",
        f"    max_px: {_yaml_scalar(field['val_max_px'])}",
        f"    detection_rate_pct: {_yaml_scalar(field['val_detection_rate_pct'])}",
        f"crc: {_yaml_scalar(result.checksum_matches)}",
    ]
    return "\n".join(lines) + "\n"


def write_text_atomic(path: Path, text: str, *, force: bool = False) -> None:
    """Commit a completed text file atomically; never leave a partial YAML."""
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not force:
        raise EepromError(f"output already exists: {path}; use --force to replace it")
    temporary_name: Optional[str] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", newline="\n", dir=str(path.parent),
            prefix=f".{path.name}.", suffix=".tmp", delete=False,
        ) as handle:
            temporary_name = handle.name
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def write_camera_yaml_files(
    output_dir: Path,
    left: ParsedEeprom,
    right: ParsedEeprom,
    *,
    left_bus: int,
    right_bus: int,
    force: bool = False,
) -> tuple[Path, Path]:
    """Write left.yaml and right.yaml after checking both destinations."""
    output_dir = output_dir.resolve()
    left_path = output_dir / "left.yaml"
    right_path = output_dir / "right.yaml"
    existing = [path for path in (left_path, right_path) if path.exists()]
    if existing and not force:
        names = ", ".join(str(path) for path in existing)
        raise EepromError(f"output already exists: {names}; use --force to replace it")
    write_text_atomic(
        left_path, render_camera_yaml("left", left_bus, left), force=True,
    )
    write_text_atomic(
        right_path, render_camera_yaml("right", right_bus, right), force=True,
    )
    return left_path, right_path


def run_self_test() -> None:
    validate_layout()
    if crc_sum32(bytes(range(256))) != 0x00007F80:
        raise EepromError("CRC_SUM32 self-test failed")
    if EEPROM_END != 299 or FIELD_SPECS[-1].address != CHECKSUM_ADDRESS:
        raise EepromError("EEPROM layout size self-test failed")
    print("self-test: PASS")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument(
        "--addr", type=parse_int, default=DEFAULT_EEPROM_ADDRESS,
        help="7-bit EEPROM address in decimal or 0x form (default: 0x50)",
    )
    parser.add_argument(
        "--chunk-size", type=int, default=DEFAULT_CHUNK_SIZE,
        help="bytes per i2ctransfer read (default: 32)",
    )
    parser.add_argument(
        "--yaml-dir", type=Path, metavar="DIR",
        help="read both EEPROMs and save DIR/left.yaml and DIR/right.yaml",
    )
    parser.add_argument("--left-bus", type=int, default=2, help="left camera I2C bus (default: 2)")
    parser.add_argument("--right-bus", type=int, default=1, help="right camera I2C bus (default: 1)")
    parser.add_argument("--force", action="store_true", help="replace existing left.yaml/right.yaml files")
    parser.add_argument("--raw", action="store_true", help="print the complete hex dump only")
    parser.add_argument("--json", action="store_true", help="print parsed data as JSON")
    parser.add_argument(
        "--strict-checksum", action="store_true",
        help="return exit code 2 when the stored CRC_SUM32 does not match",
    )
    parser.add_argument(
        "--strict-crc", dest="strict_checksum", action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--self-test", action="store_true", help="test layout and CRC_SUM32 without hardware")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    if shutil.which("i2ctransfer") is None:
        raise EepromError("i2ctransfer was not found; install/provide i2c-tools on the board")

    if args.yaml_dir is not None:
        if args.raw or args.json:
            raise EepromError("--yaml-dir cannot be combined with --raw or --json")
        if args.left_bus == args.right_bus:
            raise EepromError("left and right I2C buses must be different")
        left = parse_eeprom(
            I2cTransferReader(args.left_bus, args.addr, args.chunk_size).read(0, EEPROM_END)
        )
        right = parse_eeprom(
            I2cTransferReader(args.right_bus, args.addr, args.chunk_size).read(0, EEPROM_END)
        )
        if args.strict_checksum and not (left.checksum_matches and right.checksum_matches):
            raise EepromError("left or right EEPROM CRC_SUM32 does not match")
        left_path, right_path = write_camera_yaml_files(
            args.yaml_dir,
            left,
            right,
            left_bus=args.left_bus,
            right_bus=args.right_bus,
            force=args.force,
        )
        print(
            f"saved left EEPROM YAML: {left_path} (i2c-{args.left_bus})\n"
            f"saved right EEPROM YAML: {right_path} (i2c-{args.right_bus})"
        )
        return 0

    reader = I2cTransferReader(args.bus, args.addr, args.chunk_size)
    raw = reader.read(0, EEPROM_END)
    if args.raw:
        print("\n".join(hexdump(raw)))
        return 0

    result = parse_eeprom(raw)
    if args.json:
        print_json(result, bus=args.bus, address=args.addr)
    else:
        print_text(result, bus=args.bus, address=args.addr)
    return 2 if args.strict_checksum and not result.checksum_matches else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (EepromError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
