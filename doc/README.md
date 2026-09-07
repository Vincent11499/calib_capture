# 双目及双目–IMU 标定数据采集工具

本工具通过 `ctypes` 直接调用 `AX_VIN_GetYuvFrame`，从已经运行的 VIN 管线获取全分辨率双目 NV12 帧；IMU 通过 ROS 2 话题订阅。数据先保存为可恢复的目录数据集，ROS1 bag 在录制完成后离线生成，不使用 `ros2 bag record`。

## 文档导航

- [使用操作说明](./使用操作说明.md)：板端部署、探测、采集、校验、bag 导出和故障处理。
- [项目 Agent 维护规范](../AGENTS.md)：后续 Agent 修改代码时必须遵守的硬件、ABI、数据与测试契约。工程只保留这一份顶层 Agent 文档。
- [AXERA VIN/ABI 约束](./AXERA_IVPS.md)：VIN 获取、物理映射、完整帧释放和 SDK ABI。
- [目录数据集格式](./DATASET_FORMAT.md)：文件、CSV、时间戳和 ROS1 bag 映射。

板端部署目标为 `/workspace/minsky/Vincent/calib_capture`；对应宿主机目录是 `/workspace/test/Vincent/calib_capture`。SSH、SCP 和 Docker 进入步骤及授权边界见《使用操作说明》和顶层 `AGENTS.md`。

## 运行边界

- 相机/VIN/IVPS 生产管线必须已经启动。脚本中的 `--resolution` 只校验实际输出，不会修改通道配置或缩放图像。
- 标定（cali）默认映射为 `cam0=左目=VIN pipe 2/chn 0`、`cam1=右目=VIN pipe 0/chn 0`，接口为 `AX_VIN_GetYuvFrame`，源格式必须为 NV12。
- 录制期间不要让其他消费者竞争读取相同的 VIN pipe/chn 输出队列。
- 板端运行库默认为 `/soc/lib`；运行前设置 `BSP_MSP_DIR=/soc/lib` 和 `LD_LIBRARY_PATH=/soc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}`，脚本同时按依赖顺序加载现有 AX 库。随后调用 `AX_SYS_Init`、`AX_VIN_Init`，退出时反向释放当前 Python 消费者上下文。
- 4K、mono8、双目 10 Hz、60 秒约产生 10 GB 图像。本工具不做容量或写速准入检查；写盘队列满时会成对丢弃左右图并写入质量报告。

## 依赖与部署

以下命令默认从项目顶层执行。安装采集依赖：

```bash
python3 -m pip install -r script/requirements-capture.txt
```

只有 `camera-imu` 和 `probe --with-imu` 需要先 source 与 IMU 发布端一致的 ROS 2 环境；纯双目采集、校验和 bag 导出不依赖 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

仅双目采集不导入 `rclpy`。标定路径强制核验 NV12，并直接提取 Y 平面生成 mono8 BMP。

## 已确认的 cali / vis 硬件映射

| 模式 | 输出 | 左目 | 右目 | 接口 | 格式与分辨率 |
|---|---|---|---|---|---|
| cali | VIN | pipe 2 / chn 0 | pipe 0 / chn 0 | `AX_VIN_GetYuvFrame` | NV12；分辨率跟随 config，标定脚本支持核验 3840×2160 或 1920×1080 |
| vis | VIN | pipe 2 / chn 0 | pipe 0 / chn 0 | `AX_VIN_GetYuvFrame` | NV12，3840×2160 |
| vis | IVPS | grp 7 / chn 0 | grp 8 / chn 0 | `AX_IVPS_GetChnFrame` | 灰度，640×384 |

本采集工具的录制命令使用表中 cali VIN 路径。vis 两套输出记录在 `default_config.yaml` 的 `hardware_layout` 中，供联调和避免接错通道；不会被标定录制命令误用。

ROS1 bag 只在 PC 本机离线导出，不在板端安装 `rosbags` 或执行转换：

```bash
python3 -m pip install -r script/requirements-export.txt
```

## 推荐操作流程

先探测硬件。默认要求实际通道已经输出 3840×2160：

```bash
python3 script/calibration_data.py probe --samples 50
python3 script/calibration_data.py probe --samples 50 --with-imu
```

`probe` 会报告实际分辨率/格式、左右源帧率、最大与平均 PTS 差、序号缺口、SDK 头文件契约；加 `--with-imu` 后还会报告 IMU 有效频率、最大间隔、与首个相机 PTS 的偏移及共同时间窗。时钟域差超过默认 5 秒或 PTS 单位明显不合理时返回非零退出码。

如果上游配置为 1080p，必须显式指定：

```bash
python3 script/calibration_data.py probe --resolution 1920x1080
```

两种分辨率的代码支持位置为：`calib_capture/cli.py` 接收参数，`config.py::parse_resolution()` 限定 3840×2160/1920×1080，`axera.py::validate_frame_metadata()` 对每个 VIN 帧强校验实际尺寸，`validator.py` 再核验落盘 BMP。脚本不会切换相机 config 或缩放图像。

如需临时覆盖接线，使用 `--left-pipe/--left-chn/--right-pipe/--right-chn`；例如默认值等价于：

```bash
python3 script/calibration_data.py probe \
  --left-pipe 2 --left-chn 0 --right-pipe 0 --right-chn 0
```

逐姿态采集双目标定数据。每调整一次标定板姿态后按 Enter，默认保存 20 对：

```bash
python3 script/calibration_data.py stereo-triggered --output ./data
```

连续双目采集：

```bash
python3 script/calibration_data.py stereo-continuous \
  --output ./data \
  --duration-s 60 --frame-divisor 3
```

`--frame-divisor N` 按左右 `u64SeqNum` 源序号分频：第一对保存，之后当两侧源序号都至少前进 N 时再保存。源周期为 20 ms 且源帧可取得时，`3` 约为 60 ms/对（16.67 Hz），`5` 约为 100 ms/对（10 Hz）；若期间源帧已不可取得，实际间隔只会更长，并在元数据中记录序号缺口。它与按 PTS 控制目标频率的 `--save-rate-hz` 是两种互斥策略；两者都省略时默认保存 10 Hz。命令行显式策略会覆盖 YAML 中的另一策略，YAML 同一层不能同时设置正数 `frame_divisor` 和 `save_rate_hz`。

双目–IMU采集：

```bash
python3 script/calibration_data.py camera-imu \
  --output ./data \
  --duration-s 60 --frame-divisor 5 --imu-topic /imu/data
```

`--output` 是会话根目录，默认就是 `./data`，因此可以省略。每次录制自动生成 `./data/YYYY-MM-DD_HH-MM-SS/`，不会直接把文件写到 `./data` 根目录。

假设本次生成 `./data/2026-09-07_15-30-00/`，离线校验与 bag 导出为：

```bash
python3 script/calibration_data.py validate ./data/2026-09-07_15-30-00
python3 script/calibration_data.py validate ./data/2026-09-07_15-30-00 --verify-checksums
python3 script/calibration_data.py export-ros1 \
  ./data/2026-09-07_15-30-00 \
  ./data/2026-09-07_15-30-00.bag
```

可选 bag 压缩：`--compression lz4` 或 `--compression bz2`。默认不压缩。
导出前固定执行完整 SHA-256 校验；导出后再打开 bag，逐话题核对消息类型、数量、首尾时间戳、单调性以及 Header stamp 与 bag record time 是否完全相同。实现基于 [rosbags 0.11 ROS1 Writer](https://ternaris.gitlab.io/rosbags/topics/rosbag1.html)。
只有诊断损坏数据时才使用 `--force-dataset`；默认情况下任何硬校验错误都会阻止交付 bag。

## YAML 配置

模板为 `script/default_config.yaml`。`--config` 必须放在子命令前，命令行参数优先于 YAML：

```bash
python3 script/calibration_data.py --config script/default_config.yaml camera-imu \
  --output ./data \
  --resolution 1920x1080
```

## 结果状态

- 正常数据集：`<output>/YYYY-MM-DD_HH-MM-SS/`。
- 捕获异常、错误分辨率或未完成 20 次触发：时间戳目录保留为 `.failed` 后缀。
- 录制中的临时目录：时间戳目录使用 `.inprogress` 后缀。
- `validation.json` 的 `status` 为 `pass`、`warning` 或 `fail`。队列丢帧、低实际频率及源序号缺口为警告；单侧缺失、时间倒退、同步超限、损坏 BMP、无 IMU 或时钟域不一致为失败。
- 正常和失败数据集都会生成 `validation.json`；失败数据不会被静默删除。连续模式按源 PTS 统计真实取帧率和源序号缺口，保存降频造成的序号跳变不会误报为上游丢帧。
- 板端找不到 SDK 头文件时仍会执行固定 ctypes ABI 检查并给出质量警告。要完成严格的哈希验收，请复制对应 SDK `include` 目录并传入 `--sdk-include-dir /path/to/include`。

## 无硬件自检

```bash
cd script
python3 calibration_data.py self-test
python3 -m unittest discover -s tests -v
```

自检覆盖 ctypes ABI、SDK 头文件哈希、BMP、颜色格式转换、PTS 限频、整对帧分频、目录数据集和校验和。安装 rosbags 后，测试还会生成并重新打开一份 ROS1 bag。
