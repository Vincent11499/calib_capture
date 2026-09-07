# 目录数据集格式

目录数据集是唯一原始数据。ROS1 bag 是可重复生成的派生文件。

录制命令把 `--output` 作为根目录，默认使用 `./data`；每次会话的数据集目录为 `<output>/YYYY-MM-DD_HH-MM-SS/`。以下结构均位于该时间戳目录内：

```text
dataset.yaml
cameras/cam0/timestamps.csv
cameras/cam0/<timestamp_ns>.bmp
cameras/cam1/timestamps.csv
cameras/cam1/<timestamp_ns>.bmp
imu/imu0.csv
meta/pairs.csv
meta/imu_full.jsonl
meta/session.json
meta/checksums.sha256
validation.json
```

## 相机

- `cam0` 固定表示左目，`cam1` 固定表示右目。
- `dataset.yaml` 记录 `capture_api: AX_VIN_GetYuvFrame`、源格式 NV12，以及每个相机实际使用的 VIN `pipe/chn`；默认左目为 `2/0`、右目为 `0/0`。
- BMP 为无压缩 8 位灰度、256 项灰度调色板、bottom-up 行顺序。
- 文件名和 `timestamps.csv` 使用相机各自的 `u64PTS × 1000` 纳秒值。
- `timestamps.csv` 只有 `timestamp_ns,filename` 两列，便于标定程序直接读取。
- 不用左右平均时间覆盖源时间戳。配对关系、硬件序号和左右时间差存放在 `meta/pairs.csv`。

## IMU

`imu/imu0.csv` 列定义：

```text
timestamp_ns,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
```

角速度单位为 rad/s，加速度单位为 m/s²，时间戳为 ROS 消息原始 `header.stamp`。`meta/imu_full.jsonl` 额外保留 frame ID、orientation 及三组 covariance，以便无损构造 `sensor_msgs/Imu`。

## 元数据与一致性

- `dataset.yaml`：稳定的传感器与文件接口。
- `meta/session.json`：实际命令配置、SDK ABI/头文件哈希、运行库、源/保存/IMU 帧率、长间隔、完整对丢弃、源序号缺口和所有 AX 非零返回统计。
- `meta/checksums.sha256`：所有 BMP、CSV、JSONL 与 `dataset.yaml` 的 SHA-256。
- `validation.json`：完整性、单调时间戳、双目同步、有效频率和相机–IMU共同时间窗结论。

写盘时先使用隐藏 `.tmp` 文件，左右 BMP 都完成后才写入 CSV。持续录制使用容量为 2 对的队列；过载时整对丢弃，所以正式数据集中不会主动制造单侧图像。

## ROS1 bag 映射

- `cam0` → `/stereo/left/image_raw`，`sensor_msgs/Image`，`mono8`。
- `cam1` → `/stereo/right/image_raw`，`sensor_msgs/Image`，`mono8`。
- `imu0` → `/imu/data`，`sensor_msgs/Imu`。
- message Header stamp 和 bag record time 都等于源时间戳。
- 所有传感器消息按时间戳归并后写入，导出完成后重新打开 bag 核对数量和时间范围。
