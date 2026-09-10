# calib_capture v1.0.0 快速开始

采集软件和 Kalibr no-ROS 共享 `schema_version: "1.0.0"` 数据契约。采集、求解和结果评价分别执行，采集报告的通过或警告不代表标定精度已经合格。

## 文档

- [操作说明](使用操作说明.md)：设备环境、采集参数、停止和故障处理。
- [数据集格式](DATASET_FORMAT.md)：目录、身份、单位、时间和元数据。
- [AXERA VIN 与 ABI](AXERA_IVPS.md)：接口、帧所有权和映射规则。
- [历史板端验证](BOARD_VALIDATION.md)：历史版本实测记录，不表示本版已重新上板。
- [Agent 维护规范](../AGENTS.md)。

## 录制

正式标定读取 cali VIN，cam0 恒为左目 pipe 2/chn 0，cam1 恒为右目 pipe 0/chn 0。源图像必须是 NV12，实际分辨率必须符合 3840×2160 或 1920×1080；脚本不缩放或重配管线。

```bash
python3 script/calibration_data.py probe --resolution 1920x1080
python3 script/calibration_data.py stereo-triggered --output ./data --dataset-name evt31_stereo
python3 script/calibration_data.py stereo-continuous --output ./data --duration-s 60
python3 script/calibration_data.py --config script/default_config.ini camera-imu \
  --output ./data --dataset-name evt31 --resolution 1920x1080
python3 script/calibration_data.py imu-continuous --output ./data --duration-s 3600
```

`--dataset-name` 允许 1–64 个 ASCII 字母、数字、下划线或连字符，首字符须为字母或数字。留空 INI 值表示按模式自动选择 `stereo`、`stereo_imu`、`imu`。输出例如 `data/evt31_260909_1517/`，末尾固定 `_YYMMDD_hhmm`，使用设备本地时间。同名正式、录制中或失败目录存在时拒绝录制，不追加秒数或覆盖旧数据。

默认 BMP 无损 mono8；可显式配置 JPEG95。有损压缩不会改变图像尺寸、时间戳和轴，但可能影响角点定位。默认双目保存 5 Hz，IMU 全部保存。Camera–IMU 图像窗前后各要求至少 1 秒 IMU，并以原始传感器时间戳核验；`duration_s` 只包含图像窗口。

## 配置

独立 `imu-continuous` 默认还订阅 `/imu/temperature`：同一 CSV 第八列 `temperature` 为最新收到的摄氏温度，约 1 Hz 更新之间沿用上一值。先等温度再保存 IMU，超时失败；不改变 Camera–IMU CSV。可通过 `--imu-temperature-topic` 或 INI `imu_temperature_topic` 覆盖话题。

`script/default_config.ini` 保持简洁，`script/reference_config.ini` 给出完整参数。显式 `--config` 放在子命令前；CLI > INI > 默认。INI 只允许 `[capture]`，必须声明版本；值不加引号，注释独立起行，不做字符串插值。

```ini
[capture]
schema_version = 1.0.0
; 可选；留空按模式自动命名。
dataset_name = evt31
resolution = 1920x1080
image_format = bmp
sync_tolerance_us = 200
```

## 校验与交付

```bash
python3 script/calibration_data.py validate ./data/evt31_260909_1517
python3 script/calibration_data.py export-ros1 \
  ./data/evt31_260909_1517 ./data/evt31_260909_1517.bag
```

新数据无需先导出 bag，Kalibr task 的目录路径指向会话目录即可。单目标定只选 cam0；双目标定选 cam0、cam1；双目–IMU 使用相机结果和按 ID 关联的 IMU 参数。目录任务不需要 topic，IMU 噪声 YAML 可省略 rostopic；bag 任务仍需要话题。独立 `imu-continuous` 只生成 Allan 分析原始 CSV 和元数据，不支持当前 ROS1 导出。

`meta/validation.json` 给出结构与采集质量的 `pass/warning/fail`；异常保留 `.failed`，录制中使用 `.inprogress`。不计算逐帧 SHA-256。v1.0.0 不读取历史 schema 1/2/3，也不改名或迁移已有数据。

## 验证与部署边界

```bash
cd script
python3 calibration_data.py self-test
python3 -m unittest discover -s tests -v
```

测试使用合成小数据；真实 Kalibr reader 集成检查 BMP/JPEG、话题、身份、数量和原始 IMU 值。PC ROS1 测试要求现有 `rosbags==0.11.0`，缺依赖明确跳过。板端不安装新依赖；只有获得单独授权后才执行部署和采集，部署只同步 `script/`。
