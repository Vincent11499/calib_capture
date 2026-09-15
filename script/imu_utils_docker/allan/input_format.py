"""Read immutable eight-column IMU CSV and select a relative half-open time range."""
import csv
from decimal import Decimal, ROUND_HALF_UP
import math

COLUMNS = ['timestamp_ns', 'wx', 'wy', 'wz', 'ax', 'ay', 'az', 'temperature']


def add_time_arguments(parser):
    start = parser.add_mutually_exclusive_group()
    start.add_argument('--start-s', help='start seconds relative to first source IMU, inclusive')
    start.add_argument('--start-h', help='start hours relative to first source IMU, inclusive')
    end = parser.add_mutually_exclusive_group()
    end.add_argument('--end-s', help='end seconds relative to first source IMU, exclusive')
    end.add_argument('--end-h', help='end hours relative to first source IMU, exclusive')


def time_bounds(args):
    def convert(seconds, hours):
        value = hours if hours is not None else seconds
        if value is None:
            return None
        number = Decimal(value)
        if not number.is_finite() or number < 0:
            raise ValueError('time bounds must be finite and non-negative')
        nanoseconds = number * (3600 if hours is not None else 1) * 1000000000
        if nanoseconds != nanoseconds.to_integral_value():
            raise ValueError('time bounds must have nanosecond precision or coarser')
        return int(nanoseconds)
    start = convert(args.start_s, args.start_h)
    end = convert(args.end_s, args.end_h)
    start = 0 if start is None else start
    if end is not None and end <= start:
        raise ValueError('end must be later than start')
    return start, end


def time_cli(start_ns, end_ns):
    result = ['--start-s', str(Decimal(start_ns)/1000000000)]
    if end_ns is not None:
        result += ['--end-s', str(Decimal(end_ns)/1000000000)]
    return result


def time_suffix(start_ns, end_ns):
    if start_ns == 0 and end_ns is None:
        return ''
    def hours(value):
        result = (Decimal(value)/3600000000000).quantize(Decimal('0.1'), rounding=ROUND_HALF_UP)
        return format(result, 'f').rstrip('0').rstrip('.')
    return '_' + hours(start_ns) + 'h-' + (hours(end_ns)+'h' if end_ns is not None else 'end')


def rows(dataset):
    with (dataset if dataset.is_file() else dataset / 'imu/imu0.csv').open(encoding='utf-8', newline='') as stream:
        reader = csv.reader(stream)
        if next(reader, None) != COLUMNS:
            raise ValueError('expected eight-column IMU CSV: ' + ','.join(COLUMNS))
        previous = None
        for line, row in enumerate(reader, 2):
            if len(row) != 8:
                raise ValueError('wrong column count at line ' + str(line))
            stamp = int(row[0])
            values = tuple(float(v) for v in row[1:])
            if not 0 <= stamp < (2**32) * 10**9:
                raise ValueError('timestamp outside ROS1 time range')
            if previous is not None and stamp <= previous:
                raise ValueError('non-increasing timestamp at line ' + str(line))
            if not all(math.isfinite(v) for v in values):
                raise ValueError('non-finite value at line ' + str(line))
            previous = stamp
            yield stamp, values


def selected_rows(dataset, start_ns=0, end_ns=None):
    # Call validate_input first: the entire source is checked before any range is exported.
    origin = None
    for stamp, values in rows(dataset):
        if origin is None:
            origin = stamp
        offset = stamp-origin
        if end_ns is not None and offset >= end_ns:
            break
        if offset >= start_ns:
            yield stamp, values


def validate_input(dataset, frame_id='imu_link', start_ns=0, end_ns=None):
    if not frame_id.strip():
        raise ValueError('empty frame_id')
    if start_ns < 0 or (end_ns is not None and end_ns <= start_ns):
        raise ValueError('invalid selection range')
    first = last = origin = source_last = None
    count = total = 0
    min_dt = float('inf')
    max_dt = 0
    for stamp, values in rows(dataset):
        if origin is None:
            origin = stamp
        total += 1
        source_last = stamp
        offset = stamp-origin
        if offset < start_ns or (end_ns is not None and offset >= end_ns):
            continue
        if first is None:
            first = stamp
        if last is not None:
            min_dt = min(min_dt, stamp-last)
            max_dt = max(max_dt, stamp-last)
        last = stamp
        count += 1
    if count < 2:
        raise ValueError('at least two samples required in selected interval')
    return dict(schema_version='1.0.0', kind='imu_utils_input_validation', status='pass',
                source=str(dataset), dataset_id=dataset.parent.parent.name if dataset.is_file() else dataset.name,
                frame_id=frame_id, messages=count, first_timestamp_ns=first, last_timestamp_ns=last,
                duration_s=(last-first)/1e9, effective_rate_hz=(count-1)*1e9/(last-first),
                minimum_interval_ns=min_dt, maximum_interval_ns=max_dt,
                selection=dict(origin_timestamp_ns=origin, start_offset_ns=start_ns, end_offset_ns=end_ns,
                               start_s=start_ns/1e9, end_s=None if end_ns is None else end_ns/1e9,
                               interval='[start,end)', source_messages=total,
                               source_last_timestamp_ns=source_last, source_duration_s=(source_last-origin)/1e9,
                               complete_source_checked=True, timestamps_rebased=False),
                sampling_note='Mean sample period approximation; no resampling or dropped rows within selection.',
                temperature_note='latest-received temperature retained in CSV only; no synthetic Temperature topic')
