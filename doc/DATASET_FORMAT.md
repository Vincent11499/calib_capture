# 目录数据集格式

目录数据集是唯一原始数据。ROS1 bag 是可重复生成的派生文件。

运行配置使用 `--config` 指定的 INI 文件，与本页的数据集契约相互独立。输出中的 `dataset.yaml` 和 JSON 元数据保留原格式；`dataset.yaml` 写入与校验不依赖 PyYAML。

录制命令把 `--output` 作为根目录，默认使用 `./data`；每次会话的数据集目录为 `<output>/YYYY-MM-DD_HH-MM-SS/`。以下结构均位于该时间戳目录内：

```text
dataset.yaml
cameras/cam0/timestamps.csv
cameras/cam0/<timestamp_ns>.bmp 或 .jpg
cameras/cam1/timestamps.csv
cameras/cam1/<timestamp_ns>.bmp 或 .jpg
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
- JPEG 为有损、单通道 8 位灰度 `.jpg`，质量默认 95（1–100）；不改变尺寸或时间戳，同一会话只选择一种格式。使用 OpenCV/NumPy 编解码，不依赖 Pillow。
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
- 当前版本为 `schema_version: 3`，`data_integrity: structural`；仍可读取 schema 1/2 的旧 BMP 数据集。`dataset.yaml` 增加 `image_format`、`jpeg_quality`、`lossy`，格式/质量也写入会话 config；BMP 模式忽略 JPEG 质量。
- `meta/session.json`：实际命令配置、SDK ABI/头文件哈希、运行库、源/保存/IMU 帧率、长间隔、完整对丢弃、源序号缺口和所有 AX 非零返回统计。
- 数据文件不计算或验证 SHA-256，也不生成校验和清单。读取旧数据时忽略旧清单。相机 SDK 的启动 ABI/头文件契约仍独立检查。
- `meta/session.json.write_performance`：保留 `pair_count`、`mean_pair_ms`、`max_pair_ms` 和 `total_pair_write_s`。v1.8 起表示图像任务提交到该对有序提交结束的延迟，包含线程池/顺序等待，不含输入队列等待、取帧和 fsync。多对重叠时累计延迟不是总录制时间。`image_writer_workers` 记录实际线程数（默认 2，1–8）。
- `capture_stages`：取帧同步、映射/缓存刷新、Y 平面复制和解除映射的 count/mean/max/total。`image_stages`：每图 JPEG 编码、文件写入，以及每对提交、0.5 秒元数据刷新的同类统计；不储存逐帧样本。
- `capture_stats.pipeline`：输入队列及同时处理对数的容量/高水位、缓冲池实际分配字节数/高水位/剩余占用、队列等待和结束排空时间。`capture_stats.capture_wall_s`、`capture_stats.capture_peak_rss_bytes` 记录录制墙钟时间和录制阶段 RSS 峰值；`capture_and_drain_cpu_s`／`capture_and_drain_wall_s` 记录录制及排空的进程 CPU 秒数／墙钟秒数，二者比值约为平均使用 CPU 核数。`validation_duration_s`、`peak_rss_bytes` 另记录校验耗时和含校验阶段的峰值。平台不支持 RSS 时为 null。
- `validation.json`：完整性、单调时间戳、双目同步、有效频率和相机–IMU共同时间窗结论。

写盘时先使用隐藏 `.tmp` 文件，左右图像都完成后才有序提交文件并写 CSV。持续录制默认有 2 对等待队列，最多同时处理 `ceil(image_workers/2)` 对；完成但尚未有序提交的对也占用额度。缓冲池容量为这两项之和再加 1 个生产者槽位，按需分配，不随时长增长。过载时在映射前整对丢弃。

CSV 和 IMU JSONL 每 0.5 秒批量刷新，正常停止及可处理异常退出时最终刷新。强杀/断电可能丢失尚未刷新的记录。工作线程失败时先等待已提交的任务结束，再清理临时文件和复用缓冲；重命名或 CSV 提交中途失败仍可能留下半对，整个目录必须保留为 `.failed`，不得交付为成功数据。

结构校验检查 BMP 文件头、调色板、尺寸、文件长度、文件与 CSV 一一对应关系及时间戳。等长度像素损坏不会被检测；未启用逐对 `fsync`，突然断电时最近的数据可能丢失。

JPEG 校验检查 SOI/EOI 标志并完整解码，拒绝非单通道、错误尺寸、缺少结束标志及不可解码图像；不保证发现所有可解码的像素损坏。质量 95 不代表保留 95% 像素或固定压缩率。JPEG 模式的写入计时包含编码耗时。

## ROS1 bag 映射

- `cam0` → `/stereo/left/image_raw`，`sensor_msgs/Image`，`mono8`。
- `cam1` → `/stereo/right/image_raw`，`sensor_msgs/Image`，`mono8`。
- `imu0` → `/imu/data`，`sensor_msgs/Imu`。
- message Header stamp 和 bag record time 都等于源时间戳。
- 所有传感器消息按时间戳归并后写入，导出完成后重新打开 bag 核对数量和时间范围。
- JPEG 输入解码为 `sensor_msgs/Image` 的 `mono8`，不是 `CompressedImage`；像素是 JPEG 解码结果，无法恢复原始灰度值，bag 体积也不等于 JPG 体积。
