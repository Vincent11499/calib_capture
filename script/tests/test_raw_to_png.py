import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

from raw_to_png import convert, gray_output_path


@unittest.skipUnless(importlib.util.find_spec('cv2') and importlib.util.find_spec('numpy'),
                     'OpenCV/NumPy unavailable')
class RawPngTests(unittest.TestCase):
    def test_patterns_endian_padding_and_png_depth(self):
        import cv2
        import numpy as np
        for pattern in ('RGGB', 'BGGR', 'GRBG', 'GBRG'):
            for endian in ('little', 'big'):
                for depth in (8, 16):
                    with self.subTest(pattern=pattern, endian=endian, depth=depth), tempfile.TemporaryDirectory() as d:
                        raw = np.zeros((8, 10), dtype='<u2' if endian == 'little' else '>u2')
                        levels = {'R': 65535, 'G': 32768, 'B': 0}
                        for i, color in enumerate(pattern):
                            raw[i // 2::2, i % 2:8:2] = levels[color]
                        path = Path(d) / '测试.raw'
                        path.write_bytes(raw.tobytes())
                        out = convert(path, width=8, height=8, stride_bytes=20,
                                      pattern=pattern, endian=endian, png_bits=depth)
                        image = cv2.imdecode(np.fromfile(out, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                        self.assertEqual(image.shape, (8, 8, 3))
                        gray = cv2.imdecode(np.fromfile(gray_output_path(out), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                        self.assertEqual(gray.shape, (8, 8))
                        self.assertEqual(gray.dtype, image.dtype)
                        np.testing.assert_array_equal(gray, cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
                        self.assertEqual(image[3, 3].tolist(), [0, 128, 255] if depth == 8 else [0, 32768, 65535])
                        self.assertEqual(path.read_bytes(), raw.tobytes())
                        with self.assertRaises(FileExistsError):
                            convert(path, width=8, height=8)

    def test_effective_bits_and_size(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'image.raw'
            path.write_bytes(np.full((8, 8), 4095 << 4, dtype='<u2').tobytes())
            out = convert(path, width=8, height=8, bits=12, alignment='msb')
            self.assertTrue((cv2.imread(str(out)) == 255).all())
            with self.assertRaises(ValueError):
                convert(path, Path(d) / 'bad.png', width=8, height=8, bits=12)
            path.write_bytes(b'\0' * 127)
            with self.assertRaisesRegex(ValueError, 'exactly 128'):
                convert(path, Path(d) / 'size.png', width=8, height=8)

    def test_compression_metadata_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            path = root / 'cameras/cam0/test.raw'
            path.parent.mkdir(parents=True)
            path.write_bytes(bytes(128))
            (root / 'meta').mkdir()
            pair = {'files': ['cameras/cam0/test.raw'], 'left': {'compress_mode': 1}}
            (root / 'meta/pairs.jsonl').write_text(json.dumps(pair) + '\n')
            with self.assertRaisesRegex(ValueError, 'compressed RAW'):
                convert(path, width=8, height=8)

    def test_existing_gray_prevents_both_outputs(self):
        with tempfile.TemporaryDirectory() as d:
            source = Path(d) / 'input.raw'
            source.write_bytes(bytes(128))
            output = Path(d) / 'custom.png'
            gray_output_path(output).write_bytes(b'existing')
            with self.assertRaises(FileExistsError):
                convert(source, output, width=8, height=8)
            self.assertFalse(output.exists())
            self.assertEqual(gray_output_path(output).read_bytes(), b'existing')


if __name__ == '__main__':
    unittest.main()
