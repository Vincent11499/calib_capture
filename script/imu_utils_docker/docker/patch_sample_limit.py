"""Change only ingestion termination; leave upstream Allan/fitting algorithms intact."""
from pathlib import Path

p = Path('/imu_ws/src/imu_utils/src/imu_an.cpp')
s = p.read_text()
changes = {
    'int max_time_min = 10;': 'int max_time_min = 10;\nint max_samples = 0;\nint received_samples = 0;',
    '    double time = imu_msg->header.stamp.toSec( );': '    if ( end ) return;\n    double time = imu_msg->header.stamp.toSec( );',
    '    if ( start )': '    ++received_samples;\n    if ( max_samples > 0 && received_samples >= max_samples ) end = true;\n\n    if ( start )',
    'if ( time_min > max_time_min )': 'if ( max_samples <= 0 && time_min > max_time_min )',
    '    ros::Subscriber sub_imu =': '    n.param<int>( "max_samples", max_samples, 0 );\n\n    ros::Subscriber sub_imu =',
    '    while ( !end )': '    while ( ros::ok( ) && !end )',
    '    gyr_x->calc( );': '    if ( !end ) return 2;\n    gyr_x->calc( );',
}
for before, after in changes.items():
    if s.count(before) != 1:
        raise RuntimeError('upstream source changed: ' + before)
    s = s.replace(before, after)
p.write_text(s)
