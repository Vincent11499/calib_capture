from __future__ import annotations

import ctypes
import tempfile
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from unittest import mock

from calib_capture.axera import copy_y_plane
from calib_capture.bmp import read_mono8_bmp, write_mono8_bmp
from calib_capture.cli import _capture_config, build_parser
from calib_capture.config import build_capture_config
from calib_capture.dataset import DatasetWriter, ImuRecord
from calib_capture.pipeline import BufferPool, PairDiskWriter
from calib_capture.performance import Timings
from test_calibration_data import make_frame, make_grabber, make_pair, FakeVinApi


def small_writer(root, workers=2):
    config = build_capture_config(mode="camera-imu", overrides={"image_workers": workers})
    return DatasetWriter(Path(root)/"dataset", replace(config, width=5, height=3), True)


class BufferTests(unittest.TestCase):
    def test_pool_reuses_and_bounds_allocations(self):
        pool = BufferPool(15, 2)
        first, second = pool.acquire(), pool.acquire()
        self.assertIsNone(pool.acquire())
        pool.release(first)
        self.assertIs(pool.acquire(), first)
        pool.release(first)
        pool.release(second)
        with self.assertRaises(RuntimeError): pool.release(second)
        self.assertEqual(pool.summary()["allocated_bytes"], 60)
        self.assertEqual(pool.summary()["in_use_pairs"], 0)

    def test_stride_and_bottom_up_with_and_without_numpy(self):
        original_import = __import__
        def no_numpy(name, *args, **kwargs):
            if name == "numpy": raise ImportError("test fallback")
            return original_import(name, *args, **kwargs)
        for stride in (5, 8):
            source = ctypes.create_string_buffer(bytes(range(stride*3)))
            expected = b"".join(bytes(range(row*stride,row*stride+5)) for row in range(3))
            for fallback in (False, True):
                for reverse in (False, True):
                    dest = bytearray(15)
                    with mock.patch("builtins.__import__", side_effect=no_numpy if fallback else original_import):
                        copy_y_plane(ctypes.addressof(source),5,3,stride,dest,reverse)
                    want = b"".join(expected[r*5:(r+1)*5] for r in (range(2,-1,-1) if reverse else range(3)))
                    self.assertEqual(dest, want)

    def test_bottom_up_bmp_is_byte_identical(self):
        with tempfile.TemporaryDirectory() as root:
            for width in (5,8):
                pixels=bytes(range(width*3)); reverse=b"".join(pixels[r*width:(r+1)*width] for r in (2,1,0))
                first,second=Path(root)/"a.bmp",Path(root)/"b.bmp"
                write_mono8_bmp(first,width,3,pixels)
                write_mono8_bmp(second,width,3,reverse,bottom_up=True)
                self.assertEqual(first.read_bytes(),second.read_bytes())
                self.assertEqual(read_mono8_bmp(second)[1],pixels)

    def test_copy_failure_releases_frames_and_mapping(self):
        class MappedApi(FakeVinApi):
            def __init__(self):
                super().__init__({2:[make_frame(100,1)],0:[make_frame(100,2)]})
                self.buffer=ctypes.create_string_buffer(bytes(32));self.unmaps=0
            def map_frame(self, frame): return ctypes.addressof(self.buffer),32,True
            def unmap_frame(self,*args): self.unmaps+=1
        api=MappedApi();grabber=make_grabber(api);frames=grabber.grab_frames()
        with mock.patch("calib_capture.axera.copy_y_plane",side_effect=OSError("copy")):
            with self.assertRaises(OSError):grabber.copy_gray_pair(frames,release_after_copy=True)
        self.assertEqual(api.unmaps,1)
        self.assertEqual(api.releases,[(2,1),(0,2)])


class PipelineTests(unittest.TestCase):
    def test_workers_ini_cli_and_range(self):
        self.assertEqual(build_capture_config(mode="camera-imu").image_workers,2)
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/"config.ini";path.write_text("[capture]\nschema_version = 1.0.0\nimage_workers=4\n")
            args=build_parser().parse_args(["--config",str(path),"camera-imu","--image-workers","6"])
            self.assertEqual(_capture_config(args,args.command).image_workers,6)
        for workers in (0,9):
            with self.assertRaises(ValueError):build_capture_config(mode="camera-imu",overrides={"image_workers":workers})

    def test_multiple_pairs_overlap_but_commit_in_order(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root,4);writer=PairDiskWriter(dataset,2)
            second_done=threading.Event();first_waiting=threading.Event();original=dataset._commit_pair
            commits=[]
            def image(path,*args):
                if path.name.startswith(".100000"):
                    first_waiting.set()
                    if not second_done.wait(3): raise RuntimeError("no cross-pair overlap")
                else:second_done.set()
                path.write_bytes(b"test")
            def commit(job): commits.append(job.pair.left_sequence);original(job)
            with mock.patch("calib_capture.dataset.write_mono8_image",side_effect=image),mock.patch.object(dataset,"_commit_pair",side_effect=commit):
                try:
                    self.assertTrue(writer.submit(make_pair(0)))
                    self.assertTrue(first_waiting.wait(1))
                    self.assertTrue(writer.submit(make_pair(1)))
                    writer.finish()
                    self.assertEqual(commits,[1,2])
                    self.assertEqual(writer.active_high_water,2)
                finally:dataset.seal_payload({})

    def test_failure_waits_before_buffer_reuse_and_cleans_pending_pairs(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root,4);writer=PairDiskWriter(dataset,2)
            waiting=threading.Event();release=threading.Event()
            def image(path,*args):
                if 'cam1' in path.parts:raise OSError("disk failed")
                waiting.set();release.wait(3);path.write_bytes(b"test")
            with mock.patch("calib_capture.dataset.write_mono8_image",side_effect=image):
                slot=writer.reserve();pair=replace(make_pair(0),left_pixels=slot[0],right_pixels=slot[1])
                writer.submit(pair,slot);self.assertTrue(waiting.wait(1))
                self.assertEqual(writer.pool.summary()["in_use_pairs"],1)
                release.set()
                with self.assertRaisesRegex(RuntimeError,"disk failed"):writer.finish()
            self.assertEqual(writer.pool.summary()["in_use_pairs"],0)
            self.assertEqual(dataset.pair_count,0)
            self.assertFalse(list(dataset.staging_path.rglob('*.tmp')))
            dataset.abandon(RuntimeError("test"))

    def test_full_input_queue_cannot_reserve_another_buffer(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root);writer=PairDiskWriter(dataset,1)
            entered=threading.Event();release=threading.Event()
            def image(path,*args):
                entered.set();release.wait(3);path.write_bytes(b"test")
            with mock.patch("calib_capture.dataset.write_mono8_image",side_effect=image):
                try:
                    for index in range(2):
                        slot=writer.reserve();self.assertIsNotNone(slot)
                        writer.submit(replace(make_pair(index),left_pixels=slot[0],right_pixels=slot[1]),slot)
                        if index==0:self.assertTrue(entered.wait(1))
                    self.assertIsNone(writer.reserve())
                    self.assertEqual(writer.pool.allocated,2)
                finally:
                    release.set();writer.finish();dataset.seal_payload({})
            self.assertEqual(writer.pool.summary()["in_use_pairs"],0)

    def test_earlier_failure_cancels_later_completed_pair(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root,4);writer=PairDiskWriter(dataset,2)
            later_started=threading.Event();first_started=threading.Event()
            def image(path,*args):
                if path.name.startswith('.100000'):
                    first_started.set()
                    if not later_started.wait(3):raise RuntimeError("later pair never started")
                    raise OSError("first pair failed")
                later_started.set();path.write_bytes(b"test")
            with mock.patch("calib_capture.dataset.write_mono8_image",side_effect=image):
                writer.submit(make_pair(0));self.assertTrue(first_started.wait(1))
                writer.submit(make_pair(1))
                with self.assertRaisesRegex(RuntimeError,"first pair failed"):writer.finish()
            self.assertEqual(dataset.pair_count,0)
            self.assertFalse(list(dataset.staging_path.rglob('*.tmp')))
            self.assertFalse(list(dataset.staging_path.rglob('*.bmp')))
            dataset.abandon(RuntimeError("test"))

    def test_background_flush_error_propagates_and_closes_streams(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root)
            with mock.patch.object(dataset,"flush_metadata",side_effect=OSError("flush disk full")):
                deadline=time.monotonic()+2
                while dataset._flush_error is None and time.monotonic()<deadline:time.sleep(.02)
                with self.assertRaisesRegex(RuntimeError,"flush disk full"):dataset.write_pair(make_pair(0))
                with self.assertRaisesRegex(RuntimeError,"flush disk full"):dataset.seal_payload({})
            self.assertFalse(dataset._flush_thread.is_alive())
            self.assertTrue(dataset._cam0_stream.closed)
            self.assertTrue(dataset._imu_full_stream.closed)

    def test_submission_failure_waits_for_already_started_worker(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root);original=dataset._image_executor.submit;count=0
            def submit(*args,**kwargs):
                nonlocal count
                count+=1
                if count==2:raise RuntimeError("submit failed")
                return original(*args,**kwargs)
            with mock.patch.object(dataset._image_executor,"submit",side_effect=submit):
                with self.assertRaisesRegex(RuntimeError,"submit failed"):dataset.write_pair(make_pair(0))
            self.assertFalse(list(dataset.staging_path.rglob('*.tmp')))
            dataset.abandon(RuntimeError("test"))

    def test_metadata_periodic_and_final_flush(self):
        with tempfile.TemporaryDirectory() as root:
            dataset=small_writer(root)
            dataset.write_pair(make_pair(0))
            record=ImuRecord(1000000000,"imu",(0,0,0,1),(0,)*9,(0,0,0),(0,)*9,(0,0,0),(0,)*9,1000000000)
            dataset.write_imu(record)
            deadline=time.monotonic()+2
            camera=dataset.staging_path/'cameras/cam0/timestamps.csv'
            imu=dataset.staging_path/'imu/imu0.csv'
            while time.monotonic()<deadline and (camera.stat().st_size==0 or imu.stat().st_size==0):time.sleep(.02)
            self.assertEqual(len(camera.read_text().splitlines()),2)
            self.assertEqual(len(imu.read_text().splitlines()),2)
            dataset.write_pair(make_pair(1))
            dataset.write_imu(replace(record,timestamp_ns=1010000000))
            dataset.seal_payload({})
            self.assertEqual(len(camera.read_text().splitlines()),3)
            self.assertEqual(len(imu.read_text().splitlines()),3)
            self.assertFalse(dataset._flush_thread.is_alive())

    def test_timing_storage_is_constant_size(self):
        timings=Timings()
        for _ in range(10000):timings.add("stage",100)
        self.assertEqual(len(timings._values["stage"]),3)
        self.assertEqual(timings.summary()["stage"]["count"],10000)
