# 数据集契约 v1.0.0

目录数据是原始输入，ROS1 bag 是可重复生成的派生文件。软件、INI、目录 manifest 和本项目 JSON/YAML 输出统一声明 `1.0.0`；不同用途以 `type` 或 `kind` 区分。本版不接收旧整数 schema 1/2/3 或旧双 CSV 布局，不迁移、改名或删除历史数据。

## 命名和目录

会话名称为 `<dataset_name>_YYMMDD_hhmm`，例如 `evt31_260909_1517`。日期来自运行设备本地时间，只用于命名，不能当作传感器时间。session 另外记录 UTC 创建与封存时间。

`--output` 是输出根目录，默认 `./data`；`dataset_name` 可由 INI 或 CLI 显式指定，默认双目 `stereo`、Camera–IMU `stereo_imu`、独立 IMU `imu`。前缀范围为 1–64 个 ASCII 字母、数字、下划线、连字符，首字符是字母或数字；空 INI 值表示模式默认。相同前缀一分钟内重复启动报错，正式目录及 `.inprogress`、`.failed` 均不覆盖。

```text
dataset.yaml
cameras/cam0/timestamps.csv
cameras/cam0/<timestamp_ns>.bmp 或 .jpg
cameras/cam1/timestamps.csv
cameras/cam1/<timestamp_ns>.bmp 或 .jpg
imu/imu0.csv                  # 仅 camera-imu
meta/capture_dataset.yaml
meta/pairs.csv
meta/imu_full.jsonl            # 仅 camera-imu
meta/session.json
meta/validation.json
```

纯双目不创建 IMU 文件；Camera–IMU 只保存一份 IMU CSV。采集器只写灰度 BMP 或 JPG；Kalibr reader 也支持 PNG、JPEG/JPG、BMP。所有 manifest 路径相对数据集根目录，图像 CSV 文件名相对该相机 images 目录，不允许越界。

## 公共 manifest

```yaml
schema_version: "1.0.0"
type: kalibr_directory_dataset
dataset_id: evt31_260909_1517
cameras:
  - id: cam0
    timestamps: cameras/cam0/timestamps.csv
    images: cameras/cam0
    frame_id: cam0
    resolution: [1920, 1080]
    clock_source: axera_pts_us
  - id: cam1
    timestamps: cameras/cam1/timestamps.csv
    images: cameras/cam1
    frame_id: cam1
    resolution: [1920, 1080]
    clock_source: axera_pts_us
imus:
  - id: imu0
    data: imu/imu0.csv
    frame_id: imu_link
    clock_source: ros_header_stamp
    angular_velocity_unit: rad/s
    linear_acceleration_unit: m/s^2
```

根必需字段为 `schema_version/type/dataset_id/cameras/imus`，公共协议允许可选 `created_at/producer`。相机必需 `id/timestamps/images`，可选 `topic/frame_id/resolution/clock_source`；IMU 必需 `id/data`，可选 topic、身份、时钟源和上述两个单位字段。采集器始终写出示例中的完整传感器字段，校验时与 session 核对，不接受未知字段。纯双目写 `imus: []`。

目录标定通过 `id` 选择相机和 IMU，任务中不需要 `topic`，噪声配置也不需要
`rostopic`。路径只在根 manifest 定义一次。原始 ROS 话题保存在 meta/session.json
和 meta/capture_dataset.yaml，供订阅来源追溯与导出 bag 使用。根 manifest 如保留
可选 topic，采集校验仍要求其与 session 一致；新采集不再重复写出。

为保持板端零 YAML 依赖，采集器用标准库 JSON 写法输出 `dataset.yaml`，它是 YAML 的合法子集。`dataset_id` 是正式会话目录名，移动整个数据目录不改变 ID。根 IMU `frame_id` 表示会话有效坐标系；原消息的 frame ID 另存于 JSONL，不能据此改写六轴。

## 相机、IMU 和配对

- cam0 恒为左目，cam1 恒为右目；默认 VIN 分别是 pipe 2/chn 0、pipe 0/chn 0。单目标定可以直接选择一个相机，不需要另存图像。
- `timestamps.csv` 严格为 `timestamp_ns,filename`；例如 `63359465332000,63359465332000.bmp`。
- 相机各自保存 `u64PTS × 1000` 纳秒，`clock_source: axera_pts_us` 说明硬件源单位是微秒，CSV 始终是纳秒。IMU 保留 ROS 原始 header stamp；所有时间戳必须为 `[0, 2^63-1]` 范围内整数且各流严格递增。
- `imu/imu0.csv` 严格为 `timestamp_ns,wx,wy,wz,ax,ay,az`；角速度 rad/s，加速度 m/s²。不交换轴、不变号、不缩放、不因图像限频裁剪 IMU。
- `meta/pairs.csv` 为 `pair_index,left_timestamp_ns,right_timestamp_ns,left_sequence,right_sequence,delta_us,left_format,right_format`。保留两侧原始时间戳，不写平均时间。
- 默认双目同步容差 200 µs，含边界；超限重取较早侧，最多 10 次。Kalibr camera task 可显式使用 `synchronization_tolerance_s: 0.0002`，这是标定配对参数，不是目录 manifest 字段或通用 Camera–IMU 参数。
- IMU 非空原始 frame ID 优先，空值才回退配置 `imu_frame_id` 并告警；会话内有效 frame ID 变化失败。JSONL 始终保存原始字符串，包括空值；ROS1 导出只为空值应用有效 frame ID。
- Camera–IMU 默认按实际传感器时间核验前后各 1 秒 IMU，`imu_preroll_s/imu_postroll_s` 允许非负有限秒数。没有共同窗口、回退时间、覆盖不足或停流均为失败；同启动阶段相机与 IMU 参照差超过默认 5 秒也失败。

图像默认无损 mono8 BMP，保留 bottom-up 布局；可选 JPEG quality 95（1–100）保存有损单通道 JPG。同一会话格式一致，不缩放；JPEG 不代表亚像素角点无损。根 manifest 分辨率必须等于真实落盘尺寸。

## 元数据、质量和恢复

| 文件／结构 | kind | 主要内容 |
|---|---|---|
| meta/capture_dataset.yaml | capture_dataset | 硬件映射、源格式、尺寸、图像格式与有损标记 |
| meta/session.json | capture_session | dataset_id、软件版本、完整有效配置、UTC 时间、计数、SDK 契约、性能与时序 |
| meta/imu_full.jsonl 每行 | imu_message | 原 header frame ID、时间、六轴、orientation、covariance、回调到达时刻 |
| meta/validation.json | capture_validation | 结构与采集时序的 pass/warning/fail、错误、警告和统计 |

表内结构的 `schema_version` 均为字符串 `1.0.0`。session 的 `dataset_manifest` 为 `kalibr_directory_dataset`、`capture_layout` 为 `compact`。采集器检查根 manifest、内部 manifest、CSV、JSONL、session 之间的一致性。原始消息内容及其数值保留；版本和 kind 只标识包装结构。

数据完整性为 `structural`，不计算逐帧 SHA-256。BMP 检查格式、长度和尺寸；JPEG 检查首尾标志并完整解码。两者都不保证检测所有等长度或仍可解码的像素改动。SDK 启动契约哈希继续独立核验。

写盘先使用隐藏临时文件，左右两图完成后按提交顺序写文件与 CSV；队列默认两对，编码线程默认两个，容量有界，满时整对丢弃。CSV/JSONL 每 0.5 秒刷新，正常停止最终刷新；强杀或断电可能丢失未刷记录。失败保留 `.failed`，校验完成后才提交正式目录。

采集质量与标定质量分开：

- 单侧缺失、时间回退、同步超限、图像损坏、无 IMU、坐标系变化或时钟窗口错误为失败。
- 队列丢对、低有效频率、源序号缺口、缺少运行时 SDK 头文件可产生警告。
- 源序号缺口按每次成功 VIN Get 统计，包括随后用于重同步的帧；左右缺口单位是帧。同步主动淘汰是帧，限频／分频／写盘丢弃是双目对，不能相互相减。
- `capture_stats.source_sequence_accounting: successful_vin_get_v1` 是固定统计方法标识，不是文件版本。统计窗口在预热和前滚后开始；全程 sync_retries 与录制窗口缺口不是同口径。
- `write_performance` 为提交到有序写完的逐对延迟，包含线程池和顺序等待，不含输入队列等待、取帧或 fsync；重叠延迟累计不等于总墙钟时间。stage timing、队列／缓冲高水位和 RSS 单独记录。

采集校验通过只说明格式与时序满足契约，标定可用性还需要角点、重投影、极线误差和可观性评价。

## 独立 IMU 和 ROS1

Allan 指定时间区间时默认结果目录名追加 `_<起点小时>h-<终点小时>h`，整数不带小数、非整数四舍五入到一位；未给终点用 `end`。命名仅展示，不改变精确秒数选区，重名仍拒绝覆盖；显式 `--output` 不追加后缀。

显式 `--kalibr-noise` 时，分析报告增加 `noise_fit` 对象（默认仍为 `not_performed`）：逐轴拟合、选段准则、三轴最大值聚合、缺失项及 YAML 文件名。拟合完整才生成标准 Kalibr `kalibr_imu.yaml`，不添加本项目 schema 字段；不完整只保留分析报告，退出码 1。该 YAML 是待审查的噪声参数候选，不是完成标定的证明。

Allan 可用 `--start-s/--end-s` 选择相对第一条 IMU 的秒数区间 `[start,end)`。`analysis.json.selection` 保存请求范围、实际首末原始时间戳、源/选中样本数及两端排除数量；时序、温度和六轴统计仅针对选中段。默认全段，源文件保持只读。

离线 `allan` 读取下面的独立 IMU 会话，默认将分析结果写入项目根目录下 `data/allan_variance/allantools_output/<输入目录名>/`，可用 `--output` 覆盖，不修改原始数据。输出 `analysis.json` 使用 `schema_version: "1.0.0"`、`kind: imu_allan_analysis`，记录源路径、计算方法、采样间隔统计、实际使用频率、温度摘要与警告；不拟合噪声参数。六轴 `allan_<轴名>.csv` 表头为 `tau_s,adev,avar,adev_error,n`，另有标准差及方差两张 PNG 曲线图；这些 PNG 是离线图表，不是相机采集格式。非均匀采样仅在显式允许时近似计算并告警，原始零值保留。

独立 IMU CSV 现为八列 `timestamp_ns,wx,wy,wz,ax,ay,az,temperature`，仅此模式新增温度；Camera–IMU 的七列契约不变。默认同时订阅 `/imu/data`（约 100 Hz）和 `/imu/temperature`（sensor_msgs/msg/Temperature，约 1 Hz）。每条 IMU 附带最新收到的摄氏温度，不插值、不按温度 header.stamp 排序或匹配，原 IMU 时间戳保持不变。variance 不保存，NaN variance 不影响记录；温度值本身非有限则失败。

启动先等到温度，再保存后续 IMU 并开始计时；之前的 IMU 为预热跳过，不缓存或倒填未来温度。缺温度/IMU 共用启动超时。温度停止更新时继续沿用旧值，沿用时间超过 2 秒只告警。session.temperature 记录话题、类型、单位、更新数、最大沿用秒数和预热跳过数。校验要求新增八列和温度 metadata，不自动修改旧七列独立采集文件。

`imu-continuous` 用于后续 Allan 分析，不调用相机 SDK。只输出一份 `imu/imu0.csv`、`meta/session.json` 和 `meta/validation.json`，无双目 manifest、图像或 JSONL；session kind 为 `imu_capture_session`，layout 为 `imu_allan_csv`，版本仍为 `1.0.0`。逐行处理，不因时长无限缓存；原始单位不变。至少需要两条递增有限样本，停流／坐标系变化失败。

可选 ROS1 `imu_utils` 交叉分析只读取八列 `imu/imu0.csv`，不要求输入元数据，构造只用于回放的派生 bag，但该
文件不是本项目 `export-ros1` 的正式输出，也不属于 v1.0.0 数据集。原会话必须只读；
bag 和 `imu_utils` 的 TXT/YAML/PNG/日志/报告统一放在独立的 `data/allan_variance/imu_utils_output/<会话名>/`。
派生消息的 Header stamp 与 bag record time 都等于 `timestamp_ns`，六轴数值、单位、轴序
保持不变；frame ID 由 `--frame-id` 指定，默认 `imu_link`，不是从 CSV 恢复的原始帧名；无姿态时明确标记 orientation 未知。八列 CSV 不具备原始温度消息
的准确时间，因此不从沿用温度值伪造温度 topic。派生 bag 必须逐条回读，运行结果必须
证明六轴 `numData` 全部等于 bag 消息数。详细流程见 [imu_utils 容器与分析说明](IMU_UTILS.md)。

ROS1 导出仅用于新双目／Camera–IMU 数据，要求 PC 的 `rosbags==0.11.0`。消息 Header stamp 与 bag record time 都等于源时间；图像类型为 mono8 Image，JPEG 先解码，不能恢复压缩前像素。导出按时间归并，完成后重新打开核对类型、数量、时间范围和 Header。ROS 消息与 bag 格式属于第三方标准，不增加本项目 schema 字段；导出 JSON 报告使用 `kind: ros1_export_result`、`schema_version: "1.0.0"`。
