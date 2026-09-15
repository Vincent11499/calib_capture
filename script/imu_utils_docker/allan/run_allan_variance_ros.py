"""Run pinned ori-drs/allan_variance_ros in the shared Noetic container."""
import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy

from input_format import validate_input, selected_rows, add_time_arguments, time_bounds, time_cli, time_suffix

SCRIPTS = Path(__file__).resolve().parent
UPSTREAM = Path('/allan_ws/src/allan_variance_ros')


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def check_curves(path, log, count, dataset, measure_rate, start_ns=0, end_ns=None):
    import numpy as np
    match = re.findall(r'Finished collecting data\. (\d+) measurements', log)
    if match != [str(count)] or 'IMU out of order' in log:
        raise RuntimeError('upstream ingestion count mismatch: ' + str(match))
    curve = np.loadtxt(str(path))
    if curve.shape != (9999, 7) or not np.all(np.isfinite(curve)):
        raise RuntimeError('expected finite 9999 by 7 Allan deviation output')
    if not np.all(curve > 0) or not np.all(np.diff(curve[:, 0]) > 0):
        raise RuntimeError('invalid Allan deviation or tau')
    if not np.allclose(curve[:, 0], np.arange(1, 10000)*0.1, rtol=0, atol=1e-10):
        raise RuntimeError('unexpected upstream tau grid')
    # Check selected scales independently against the exact upstream bin convention.
    raw = np.fromiter((value for _, values in selected_rows(dataset, start_ns, end_ns)
                       for value in (values[3], values[4], values[5], values[0], values[1], values[2])),
                      dtype=float).reshape(-1, 6)
    if raw.shape != (count, 6):
        raise RuntimeError('reference sample count mismatch')
    raw[:, 3:] *= 180 / np.pi
    checks = []
    for index in (0, 9, 99, 999, 9998):
        tau = float(curve[index, 0])
        bin_size = int(tau * measure_rate)
        bins = (count-1)//bin_size
        mean = raw[:bins*bin_size].reshape(bins, bin_size, 6).mean(axis=1)
        reference = np.sqrt(np.mean(np.diff(mean, axis=0)**2, axis=0)/2)
        if not np.allclose(curve[index, 1:], reference, rtol=2e-5, atol=1e-10):
            raise RuntimeError('independent Allan check failed at tau=' + str(tau))
        checks.append(dict(tau_s=tau, bin_samples=bin_size, bins=bins, status='pass'))
    return dict(schema_version='1.0.0', kind='allan_variance_ros_validation', status='pass',
                input_messages=count, consumed_messages=int(match[0]), curve_points=9999,
                axes=['acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z'],
                units=dict(acc='m/s^2', gyr='deg/s'), reference_checks=checks,
                parameter_status='upstream_candidate_not_calibration_acceptance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--bag', type=Path, help='reuse the same bag file without copying; fully check it against the selected CSV interval')
    parser.add_argument('--topic', default='/imu/data')
    parser.add_argument('--frame-id', default='imu_link')
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--timeout', type=int, default=7200)
    add_time_arguments(parser)
    args = parser.parse_args()
    start_ns, end_ns = time_bounds(args)
    selection_cli = time_cli(start_ns, end_ns)
    dataset = args.dataset.resolve()
    if dataset.is_file():
        if dataset.name != 'imu0.csv' or dataset.parent.name != 'imu':
            parser.error('expected <dataset>/imu/imu0.csv')
        dataset = dataset.parent.parent
    if Path('/data') not in dataset.parents or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', dataset.name):
        parser.error('expected a dataset under /data with an ASCII directory name')
    name = dataset.name + time_suffix(start_ns, end_ns)
    output = (args.output or Path('/output') / (name + '_allan_variance_ros')).resolve()
    if Path('/output') not in output.parents or args.threads < 1 or args.timeout < 1:
        parser.error('invalid output, threads or timeout')
    with open('/tmp/imu-utils.lock', 'w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if output.exists():
            raise FileExistsError(output)
        source = validate_input(dataset, args.frame_id, start_ns, end_ns)
        # Upstream always computes through 999.9 s and requires >=2 complete bins.
        rate = source['effective_rate_hz']
        measure_rate = math.floor(rate)
        if measure_rate < 10 or source['messages'] <= 2*int(999.9*measure_rate):
            raise ValueError('upstream fixed tau grid requires >=10 Hz and over 2000 seconds of data')
        if int(rate/measure_rate) != 1:
            raise ValueError('configuration would subsample input')
        output.mkdir(parents=True)
        work = output / 'work'
        work.mkdir()
        record = dict(schema_version='1.0.0', kind='allan_variance_ros_run', status='running',
                      started_utc=datetime.now(timezone.utc).isoformat(), source=source,
                      upstream_revision=(Path('/allan_ws/upstream_revision.txt').read_text().strip()),
                      options={k: str(v) if isinstance(v, Path) else v for k,v in vars(args).items()},
                      measure_rate_hz=measure_rate, imu_skip=1,
                      tau_scale_note='upstream integer measure_rate uses nominal bin durations; actual average rate is recorded separately',
                      fit_note='upstream white fit uses tau <10 s; random walk fit uses all taus with slope constrained to +0.5..+0.501; no credible-interval selection')
        save(work / 'run.json', record)
        master = None
        master_log = None
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:11311', ROS_HOSTNAME='localhost',
                   ROS_LOG_DIR=str(work / 'ros_logs'), MPLBACKEND='Agg',
                   OMP_NUM_THREADS=str(args.threads), OPENBLAS_NUM_THREADS='1',
                   PYTHONDONTWRITEBYTECODE='1')

        def call(command, log=None, cwd=None):
            if log:
                with log.open('w') as stream:
                    subprocess.run(command, check=True, env=env, cwd=cwd, timeout=args.timeout,
                                   stdout=stream, stderr=subprocess.STDOUT)
            else:
                subprocess.run(command, check=True, env=env, cwd=cwd, timeout=args.timeout)

        try:
            bag = output / (name + '.bag')
            if args.bag:
                shared_bag = args.bag.resolve()
                if not shared_bag.is_file():
                    raise FileNotFoundError(shared_bag)
                bag.symlink_to(os.path.relpath(shared_bag, output))
                record['shared_bag'] = str(shared_bag)
                record['bag_copied'] = False
            else:
                call([sys.executable, str(SCRIPTS/'csv_to_rosbag.py'), str(dataset), str(bag),
                      '--topic', args.topic, '--frame-id', args.frame_id, *selection_cli], work/'conversion.log')
            call([sys.executable, str(SCRIPTS/'validate_rosbag.py'), str(dataset), str(bag),
                  '--topic', args.topic, '--frame-id', args.frame_id, *selection_cli], work/'bag_validation.log')
            config = dict(imu_topic=args.topic, imu_rate=rate, measure_rate=measure_rate,
                          sequence_time=math.ceil(source['messages']/rate)+1)
            import yaml
            config_path = work/'config.yaml'
            config_path.write_text(yaml.safe_dump(config, sort_keys=False))
            shutil.copyfile('/allan_ws/upstream_revision.txt', work/'upstream_revision.txt')
            # The upstream binary processes only the first bag in a directory.
            bags = work/'bags'
            bags.mkdir()
            (bags/bag.name).symlink_to(bag)
            master_log = (work/'roscore.log').open('w')
            master = subprocess.Popen(['roscore'], env=env, stdout=master_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30
            while True:
                try:
                    if ServerProxy(env['ROS_MASTER_URI']).getPid('/allan_runner')[0] == 1:
                        break
                except OSError:
                    pass
                if master.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('roscore failed to start')
                time.sleep(0.2)
            print('Computing Allan deviation from', source['messages'], 'messages', flush=True)
            call(['rosrun','allan_variance_ros','allan_variance',str(bags),str(config_path)], work/'computation.log')
            curve = work/'allan_variance.csv'
            (bags/'allan_variance.csv').rename(curve)
            validation = check_curves(curve, (work/'computation.log').read_text(),
                                      source['messages'], dataset, measure_rate, start_ns, end_ns)
            print('Curves validated; running upstream fitting and plotting', flush=True)
            call([sys.executable,str(UPSTREAM/'scripts/analysis.py'),'--data',str(curve),
                  '--config',str(config_path),'--output',str(output/'imu.yaml')], work/'analysis.log', output)
            parameters = yaml.safe_load((output/'imu.yaml').read_text())
            for key in ('accelerometer_noise_density','accelerometer_random_walk',
                        'gyroscope_noise_density','gyroscope_random_walk'):
                if not math.isfinite(parameters[key]) or parameters[key] <= 0:
                    raise RuntimeError('invalid parameter: '+key)
            if parameters['rostopic'] != args.topic or parameters['update_rate'] != rate:
                raise RuntimeError('output topic/rate mismatch')
            for name in ('acceleration.png','gyro.png'):
                if not (output/name).is_file() or (output/name).stat().st_size == 0:
                    raise RuntimeError('missing image: '+name)
            validation['parameters'] = parameters
            save(work/'validation.json', validation)
            table = '\n'.join('| '+k+' | '+str(v)+' |' for k,v in parameters.items())
            report = f'''# allan_variance_ros 分析报告

- 输入：`{dataset}`，仅八列 CSV；bag 已逐条回读比对。
- 选择区间：相对原始首条消息 [{start_ns/1e9}, {end_ns/1e9 if end_ns is not None else 'end'}) 秒；原时间戳不归零。
- 使用上游提交：`{record['upstream_revision']}`，计算与拟合源码未修改。
- 读取消息：{source['messages']}；计算实际接收：{validation['consumed_messages']}，不降采样。
- 时长：{source['duration_s']:.9f} s；实际平均采样率：{rate:.12f} Hz。
- measure_rate：{measure_rate} Hz（上游只支持整数）；时间尺度按此值计算，非均匀采样为近似。
- Allan 标准差：六轴各 9999 点，tau=0.1–999.9 s；五个尺度的六轴独立复算通过。
- 陀螺仪曲线单位 deg/s，加速度计 m/s²；YAML 陀螺仪参数已转换至弧度制。

![Accelerometer](acceleration.png)

![Gyroscope](gyro.png)

## 上游生成的 Kalibr 参数候选

| 参数 | 值 |
|---|---|
{table}

`imu.yaml` 是本次原版 analysis.py 的输出，不代表噪声模型或标定精度验收。
当前上游白噪声拟合使用 tau<10 s，随机游走拟合使用全部时间尺度，并将斜率约束为 +0.5 到 +0.501。
它没有自动筛选可信 +1/2 区间；静止性和恒温条件也未自动验收。请结合图中拟合线与数据的偏差审阅参数。

## 文件

顶层为 bag、imu.yaml、两张 PNG 和本报告；CSV 中间曲线、配置、日志、源码版本和验证记录在 work/。
详见 work/run.json、work/validation.json、work/computation.log 和 work/analysis.log。
'''
            (output/'allan_variance_ros_report.md').write_text(report, encoding='utf-8')
            record['status'] = 'pass'
        except BaseException as exc:
            record['status'] = 'fail'
            record['error'] = str(exc)
            raise
        finally:
            if master is not None and master.poll() is None:
                os.killpg(master.pid, signal.SIGINT)
                try:
                    master.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(master.pid, signal.SIGKILL)
                    master.wait()
            if master_log is not None:
                master_log.close()
            record['finished_utc'] = datetime.now(timezone.utc).isoformat()
            save(work/'run.json', record)
        print('PASS:', output, flush=True)


if __name__ == '__main__':
    main()
