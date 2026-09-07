# 目录数据集格式

目录数据集是唯一原始数据。ROS1 bag 是可重复生成的派生文件。

运行配置使用 `--config` 指定的 INI 文件，与本页的数据集契约相互独立。输出中的 `dataset.yaml` 和 JSON 元数据保留原格式；`dataset.yaml` 写入与校验不依赖 PyYAML。

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
validation.json
```

## 相机

- `cam0` 固定表示左目，`cam1` 固定表示右目。
- `dataset.yaml` 记录 `capture_api: AX_VIN_GetYuvFrame`、源格式 NV12，以及每个相机实际使用的 VIN `pipe/chn`；默认左目为 `2/0`、右目为 `0/0`。
- BMP 为无压缩 8 位灰度、256 项灰度调色板、bottom-up 行顺序。
- 文件名和 `timestamps.csv` 使用相机各自的 `u64PTS × 1000` 纳秒值。
- `timestamps.csv` 只有 `timestamp_ns,filename` 两列，便于标定程序直接读取。
- 不用左右平均时间覆盖源时间戳。配对关系、硬件序号和左右时间差存放在 `meta/pairs.csv`。
- 连续采集默认按 PTS 保存双目 5 Hz（约 200 ms 一对）；实际频率和丢对统计写入会话元数据。

## IMU

`imu/imu0.csv` 列定义：

```text
timestamp_ns,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
```

角速度单位为 rad/s，加速度单位为 m/s²，时间戳为 ROS 消息原始 `header.stamp`。`meta/imu_full.jsonl` 额外保留 frame ID、orientation 及三组 covariance，以便无损构造 `sensor_msgs/Imu`。

IMU 收到的消息全部保存，不受图像保存频率或分频参数影响。

## 元数据与一致性

- `dataset.yaml`：稳定的传感器与文件接口。
- 当前版本为 `schema_version: 2`，`data_integrity: structural`；仍可读取 schema 1 的旧数据集。
- `meta/session.json`：实际命令配置、SDK ABI/头文件哈希、运行库、源/保存/IMU 帧率、长间隔、完整对丢弃、源序号缺口和所有 AX 非零返回统计。
- 数据文件不计算或验证 SHA-256，也不生成校验和清单。读取旧数据时忽略旧清单。相机 SDK 的启动 ABI/头文件契约仍独立检查。
- `meta/session.json.write_performance`：`pair_count`、`mean_pair_ms`、`max_pair_ms` 和 `total_pair_write_s`，计时包括 BMP 大块写入、重命名和 CSV 更新，不包括取帧、排队等待或 `fsync`。
- `validation.json`：完整性、单调时间戳、双目同步、有效频率和相机–IMU共同时间窗结论。

写盘时先使用隐藏 `.tmp` 文件，左右 BMP 都完成后才写入 CSV。持续录制使用容量为 2 对的队列；过载时整对丢弃，所以正式数据集中不会主动制造单侧图像。

结构校验检查 BMP 文件头、调色板、尺寸、文件长度、文件与 CSV 一一对应关系及时间戳。等长度像素损坏不会被检测；未启用逐对 `fsync`，突然断电时最近的数据可能丢失。

## ROS1 bag 映射

- `cam0` → `/stereo/left/image_raw`，`sensor_msgs/Image`，`mono8`。
- `cam1` → `/stereo/right/image_raw`，`sensor_msgs/Image`，`mono8`。
- `imu0` → `/imu/data`，`sensor_msgs/Imu`。
- message Header stamp 和 bag record time 都等于源时间戳。
- 所有传感器消息按时间戳归并后写入，导出完成后重新打开 bag 核对数量和时间范围。
