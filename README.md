# calib_capture v1.0.0

双目及双目–IMU 标定采集工具。板端通过 AXERA VIN 获取图像，通过 ROS 2 订阅 IMU；新目录数据集直接交给 Kalibr no-ROS v1.0.0。入口为 `script/calibration_data.py`。

软件、CLI、INI 配置和项目结构化输出统一版本 `1.0.0`。本版只接收新契约，不兼容或迁移历史数据；已有采集文件保留原样，仍可由原版本工具读取。

默认 cam0 为左目 VIN pipe 2/chn 0，cam1 为右目 pipe 0/chn 0。正式标定只读取 cali VIN NV12，不重配管线、不缩放；保存原尺寸无损灰度 BMP，可显式选择 JPEG95。支持上游 3840×2160 或 1920×1080 输出。

`--output` 为输出根目录，默认 `./data`。新会话命名为 `<dataset_name>_YYMMDD_hhmm`，日期来自设备本地时间；同名前缀在一分钟内重复录制会报错。前缀可用 `--dataset-name` 或 INI `dataset_name` 设置，默认双目 `stereo`、双目–IMU `stereo_imu`、独立 IMU `imu`。

```bash
python3 script/calibration_data.py --version
python3 script/calibration_data.py --config script/default_config.ini camera-imu \
  --output ./data --dataset-name evt31 --resolution 1920x1080
```

默认双目保存 5 Hz、图像窗口 60 秒、IMU 全部保存；Camera–IMU 默认在图像前后各保留至少 1 秒 IMU。连续采集按完整双目对限频和入队，默认两个图像写盘线程；左右 PTS 差最多 200 µs。

采集 INI 使用 Python 标准库，无需 PyYAML，注释单独起行。优先级为 CLI > 显式 INI > 内置默认；不指定 `--config` 时不自动加载模板。INI 必须包含 `[capture]` 和 `schema_version = 1.0.0`。

- [快速开始](doc/README.md)
- [操作说明](doc/使用操作说明.md)
- [PC 端 IMU Allan 分析](doc/PC_IMU_ALLAN.md)
- [ROS1 imu_utils 容器与分析](doc/docker/IMU_UTILS.md)
- [ROS1 allan_variance_ros](doc/docker/ALLAN_VARIANCE_ROS.md)
- [数据集格式与字段](doc/DATASET_FORMAT.md)
- [AXERA 接口与 ABI](doc/AXERA_IVPS.md)
- [维护规范](AGENTS.md)

无硬件验证从 `script` 执行：

```bash
python3 calibration_data.py self-test
python3 -m unittest discover -s tests -v
```

部署需要另行授权，只同步 `script/`；不在本次升级中执行 SSH、SCP 或板端采集。数据、临时文件和生成物不提交 Git。
