from __future__ import annotations

import importlib.util
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "read_imx678_eeprom.py"
SPEC = importlib.util.spec_from_file_location("read_imx678_eeprom", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Completed:
    def __init__(self, stdout: bytes):
        self.stdout = stdout


class Imx678EepromTests(unittest.TestCase):
    def sample_dump(self):
        raw = bytearray(MODULE.EEPROM_END)

        def text(address, length, value):
            encoded = value.encode("ascii")
            raw[address:address + length] = encoded.ljust(length, b"\x00")

        text(0x0000, 32, "SN012345")
        text(0x0020, 20, "SONY_IMX678")
        text(0x0034, 10, "9011")
        text(0x003E, 3, "A03")
        text(0x0041, 20, "FW_IMX678_1.4.2")
        text(0x0055, 20, "CAL_INTRINSIC_2.3.1")
        raw[0x0069:0x006F] = bytes((26, 9, 28, 10, 30, 45))
        raw[0x006F:0x0073] = (3840).to_bytes(4, "little")
        raw[0x0073:0x0077] = (2160).to_bytes(4, "little")
        text(0x0077, 7, "RADTAN8")
        raw[0x007E] = 8
        for index, address in enumerate(range(0x007F, 0x0127, 8), start=1):
            raw[address:address + 8] = struct.pack("<d", index / 10.0)
        checksum = MODULE.crc_sum32(raw[:MODULE.CHECKSUM_ADDRESS])
        raw[MODULE.CHECKSUM_ADDRESS:MODULE.EEPROM_END] = checksum.to_bytes(4, "big")
        return bytes(raw)

    def test_authoritative_layout_is_contiguous_and_299_bytes(self):
        MODULE.validate_layout()
        self.assertEqual(MODULE.EEPROM_END, 299)
        self.assertEqual(MODULE.FIELD_SPECS[0].name, "sn")
        self.assertEqual(MODULE.FIELD_SPECS[0].address, 0)
        self.assertEqual(MODULE.FIELD_SPECS[-1].name, "checksum_sum32")
        self.assertEqual(MODULE.FIELD_SPECS[-1].address, 0x0127)

    def test_crc_sum32_known_vector(self):
        self.assertEqual(MODULE.crc_sum32(bytes(range(256))), 0x00007F80)

    def test_decode_types_expected_values_and_crc(self):
        result = MODULE.parse_eeprom(self.sample_dump())
        self.assertEqual(result.fields["sn"], "SN012345")
        self.assertEqual(result.fields["image_width"], 3840)
        self.assertEqual(result.fields["image_height"], 2160)
        self.assertAlmostEqual(result.fields["fx"], 0.1)
        self.assertTrue(result.checksum_matches)
        self.assertEqual(result.warnings, ())

    def test_camera_yaml_is_compact_and_keeps_all_parameter_groups(self):
        parsed = MODULE.parse_eeprom(self.sample_dump())
        document = MODULE.render_camera_yaml("left", 1, parsed)
        self.assertIn('camera: "left"', document)
        self.assertIn("i2c_bus: 1", document)
        self.assertIn("intrinsics:\n  fx: 0.1\n  fy: 0.2", document)
        self.assertIn("distortion:\n  model: \"RADTAN8\"", document)
        self.assertIn("errors:\n  calibration:", document)
        self.assertIn("  validation:", document)
        self.assertIn("crc: true", document)
        for omitted in ("schema_version", "kind:", "parameter_order", "warnings", "stored:", "calculated:"):
            self.assertNotIn(omitted, document)

    def test_two_camera_yaml_files_are_written_separately(self):
        parsed = MODULE.parse_eeprom(self.sample_dump())
        with tempfile.TemporaryDirectory() as temporary:
            left, right = MODULE.write_camera_yaml_files(
                Path(temporary), parsed, parsed, left_bus=2, right_bus=1,
            )
            self.assertEqual(left.name, "left.yaml")
            self.assertEqual(right.name, "right.yaml")
            self.assertIn('camera: "left"', left.read_text(encoding="utf-8"))
            self.assertIn('camera: "right"', right.read_text(encoding="utf-8"))
            with self.assertRaises(MODULE.EepromError):
                MODULE.write_camera_yaml_files(
                    Path(temporary), parsed, parsed, left_bus=2, right_bus=1,
                )

    def test_cli_defaults_match_physical_camera_wiring(self):
        args = MODULE.build_parser().parse_args([])
        self.assertEqual(args.left_bus, 2)
        self.assertEqual(args.right_bus, 1)

    def test_atomic_yaml_refuses_overwrite_without_force(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "stereo.yaml"
            MODULE.write_text_atomic(path, "first\n")
            with self.assertRaises(MODULE.EepromError):
                MODULE.write_text_atomic(path, "second\n")
            MODULE.write_text_atomic(path, "second\n", force=True)
            self.assertEqual(path.read_text(encoding="utf-8"), "second\n")

    def test_reader_uses_16_bit_offsets_and_bounded_chunks(self):
        source = bytes(range(70))
        commands = []

        def runner(command, **kwargs):
            self.assertEqual(kwargs["stdout"], subprocess.PIPE)
            self.assertEqual(kwargs["stderr"], subprocess.STDOUT)
            self.assertTrue(kwargs["check"])
            commands.append(command)
            offset = (int(command[4], 16) << 8) | int(command[5], 16)
            length = int(command[6][1:])
            output = " ".join(f"0x{value:02x}" for value in source[offset:offset + length])
            return Completed(output.encode("ascii"))

        reader = MODULE.I2cTransferReader(2, 0x50, 32, runner=runner)
        self.assertEqual(reader.read(0, 70), source)
        self.assertEqual([command[6] for command in commands], ["r32", "r32", "r6"])
        self.assertEqual(commands[0][3:6], ["w2@0x50", "0x00", "0x00"])
        self.assertEqual(commands[2][3:6], ["w2@0x50", "0x00", "0x40"])

    def test_short_read_and_invalid_parameters_fail(self):
        with self.assertRaises(MODULE.EepromError):
            MODULE._parse_transfer_output(b"0x01", 2)
        with self.assertRaises(MODULE.EepromError):
            MODULE.I2cTransferReader(-1)
        with self.assertRaises(MODULE.EepromError):
            MODULE.I2cTransferReader(1, 0x80)
        with self.assertRaises(MODULE.EepromError):
            MODULE.parse_eeprom(b"short")


if __name__ == "__main__":
    unittest.main()
