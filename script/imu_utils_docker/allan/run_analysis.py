"""Isolated, count-verified ROS1 imu_utils workflow. Run via bash /scripts/run.sh."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from xmlrpc.client import ServerProxy

from input_format import validate_input, add_time_arguments, time_bounds, time_cli, time_suffix

SCRIPTS = Path(__file__).resolve().parent


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path, help='/data/<session> or /data/<session>/imu/imu0.csv')
    parser.add_argument('--output', type=Path, help='new /output/<run> directory; refuses overwrite')
    parser.add_argument('--topic', default='/imu/data')
    parser.add_argument('--playback-rate', type=float, default=100)
    parser.add_argument('--max-cluster', type=int, default=100)
    parser.add_argument('--analysis-timeout', type=int, default=1800, help='seconds after playback')
    parser.add_argument('--convert-only', action='store_true', help='convert and fully validate bag only')
    parser.add_argument('--frame-id', default='imu_link')
    add_time_arguments(parser)
    args = parser.parse_args()
    start_ns, end_ns = time_bounds(args)
    selection_cli = time_cli(start_ns, end_ns)
    dataset = args.dataset.resolve()
    if dataset.is_file():
        if dataset.name != 'imu0.csv' or dataset.parent.name != 'imu':
            parser.error('CSV path must be <session>/imu/imu0.csv')
        dataset = dataset.parent.parent
    name = dataset.name
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,127}', name):
        parser.error('dataset directory name must use ASCII letters, numbers, _ and -')
    if Path('/data') not in dataset.parents:
        parser.error('input must be under /data')
    name += time_suffix(start_ns, end_ns)
    output = (args.output or Path('/output') / name).resolve()
    if Path('/output') not in output.parents:
        parser.error('output must be a child of /output')
    if (not 0 < args.playback_rate <= 200 or not 10 <= args.max_cluster <= 1000
            or args.analysis_timeout < 1):
        parser.error('invalid rate, cluster count or timeout')
    if not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*', args.topic):
        parser.error('invalid absolute ROS topic')
    # One ROS master and analyser per container, even for concurrent docker exec calls.
    with open('/tmp/imu-utils.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if output.exists():
            raise FileExistsError(output)
        print('Checking complete source:', dataset, flush=True)
        source = validate_input(dataset, args.frame_id, start_ns, end_ns)
        if source['messages'] > 2**31-1:
            raise ValueError('max_samples exceeds ROS int32 parameter limit')
        output.mkdir(parents=True, exist_ok=False)
        work = output / 'work'
        work.mkdir()
        report = dict(schema_version='1.0.0', kind='imu_utils_run', status='running',
                      started_utc=datetime.now(timezone.utc).isoformat(), source=source,
                      options={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()})
        write_json(work / 'run.json', report)
        children, handles = [], []
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11311', ROS_HOSTNAME='localhost',
                   ROS_LOG_DIR=str(work / 'ros_logs'), PYTHONDONTWRITEBYTECODE='1')

        def call(script, *params):
            subprocess.run([sys.executable, '-u', str(SCRIPTS / script), *map(str, params)],
                           check=True, env=env)

        def start(command, log):
            handle = open(log, 'w')
            handles.append(handle)
            process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                       env=env, start_new_session=True)
            children.append(process)
            return process

        def cleanup():
            for process in reversed(children):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
            for handle in handles:
                handle.close()

        try:
            bag = output / (name + '.bag')
            call('csv_to_rosbag.py', dataset, bag, '--topic', args.topic, '--frame-id', args.frame_id, *selection_cli)
            call('validate_rosbag.py', dataset, bag, '--topic', args.topic, '--frame-id', args.frame_id, *selection_cli)
            for filename in ['upstream_revision.txt', 'local.patch']:
                shutil.copyfile(Path('/imu_ws') / filename, work / filename)
            if not args.convert_only:
                result = work
                result.mkdir(exist_ok=True)
                master = start(['roscore'], work / 'roscore.log')
                deadline = time.monotonic() + 30
                while True:
                    try:
                        if ServerProxy(env['ROS_MASTER_URI']).getPid('/imu_utils_runner')[0] == 1:
                            break
                    except OSError:
                        pass
                    if master.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('ROS master failed to start')
                    time.sleep(0.2)
                analyzer = start(['rosrun', 'imu_utils', 'imu_an', '__name:=imu_an',
                                  '_imu_topic:=' + args.topic, '_imu_name:=' + name,
                                  '_data_save_path:=' + str(result) + '/', '_max_time_min:=240',
                                  '_max_samples:=' + str(source['messages']),
                                  '_max_cluster:=' + str(args.max_cluster)], result / 'imu_utils.log')
                player = start(['rosbag', 'play', '--quiet', '--wait-for-subscribers', '--delay=2',
                                '--queue=1000000', '--rate=' + str(args.playback_rate), str(bag)],
                               work / 'rosbag_play.log')
                player.wait(timeout=source['duration_s'] / args.playback_rate + 120)
                if player.returncode:
                    raise RuntimeError('rosbag playback failed')
                print('Playback finished; waiting for six-axis fitting', flush=True)
                analyzer.wait(timeout=args.analysis_timeout)
                if analyzer.returncode:
                    raise RuntimeError('imu_utils failed')
                call('validate_imu_utils_output.py', result, '--expected-messages',
                     source['messages'], '--name', name)
                call('generate_report.py', result, '--name', name)
            report['status'] = 'pass'
        except BaseException as exc:
            report['status'] = 'fail'
            report['error'] = str(exc)
            cleanup()
            raise
        finally:
            cleanup()
            report['finished_utc'] = datetime.now(timezone.utc).isoformat()
            write_json(work / 'run.json', report)
        print('PASS:', output, flush=True)


if __name__ == '__main__':
    main()
