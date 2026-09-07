# calib_capture

双目及双目–IMU 标定数据采集工具。板端使用 Python 调用 AXERA VIN 获取双目图像，通过 ROS 2 订阅 IMU；先保存目录数据集，再在 PC 本机使用 `rosbags` 导出 ROS1 bag。

默认左目为 VIN pipe 2 / chn 0，右目为 VIN pipe 0 / chn 0，输入格式 NV12。支持校验上游输出的 3840×2160 或 1920×1080 分辨率，保存无损灰度 BMP。

连续采集默认保存双目 5 Hz（约 200 ms 一对），IMU 收到的消息全部保存。默认持续 60 秒，4K 双目图像约 5 GB。

配置可通过 `--config script/default_config.ini` 指定，使用 Python 标准库 `configparser`，无需 PyYAML；不指定时使用内置默认值和命令行参数，不自动读取模板。`--config` 须放在子命令前。

图像按大块写入，不计算数据文件 SHA-256。校验保留 BMP 格式/大小、配对、CSV 与时间戳一致性检查；不能检测文件长度未变的像素位错误。

## 文档

- [快速开始](doc/README.md)
- [使用操作说明](doc/使用操作说明.md)
- [数据集格式](doc/DATASET_FORMAT.md)
- [AXERA 接口与 ABI](doc/AXERA_IVPS.md)
- [项目维护规范](AGENTS.md)

入口为 `script/calibration_data.py`，提供 `probe`、`stereo-triggered`、`stereo-continuous`、`camera-imu`、`validate`、`export-ros1` 和 `self-test` 子命令。

## 文件管理

Git 跟踪 `script/`、`doc/` 和顶层项目说明。`data/`、`tmp/`、生成的 bag/BMP、Python 环境及缓存等由 `.gitignore` 排除。采集默认输出到 `./data/YYYY-MM-DD_HH-MM-SS/`。

板端部署仅同步 `script/`。ROS1 bag 转换及其 `rosbags` 依赖在 PC 本机使用。
