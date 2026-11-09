# calib_capture 文档导航

## 首先阅读

- [使用操作说明](使用操作说明.md)：板端环境、探测、采集命令、校验、RAW16 转 PNG
  以及采集参数全集。
- [数据集格式](DATASET_FORMAT.md)：目录、manifest、相机／IMU CSV、元数据和质量统计。
- [AXERA VIN 与 ABI](AXERA_IVPS.md)：VIN/RAW 接口、帧所有权、映射和结构体约束。
- `script/read_imx678_eeprom.py`：按模组地址表只读并解析 IMX678 EEPROM；使用方法见
  [使用操作说明](使用操作说明.md#49-读取-imx678-模组-eeprom)。

## PC 端 IMU 噪声分析

- [PC 端 IMU 分析总览](PC_IMU_ALLAN.md)：时域／温度、高斯性／白噪声诊断，以及
  Python AllanTools、`allan_variance_ros` 和 `imu_utils` 的统一入口、时间选区和结果对比。
- [imu_utils 专项说明](docker/IMU_UTILS.md)：容器构建、一键流程、手动回放、验证和输出。
- [allan_variance_ros 专项说明](docker/ALLAN_VARIANCE_ROS.md)：一键流程、复用 bag、参数和排错。
- [allan_variance_ros 源码解读](ALLAN_VARIANCE_ROS_SOURCE.md)：分箱、拟合与 Kalibr YAML 单位。
- [纯 Python 复现 allan_variance_ros](PYTHON_ALLAN_VARIANCE.md)：无需 ROS、bag、C++ 或 Docker，
  直接从八列 IMU CSV 生成兼容曲线、图片和 `imu.yaml`。

## 快速开始

板端容器：

```bash
cd /workspace/minsky/Vincent/calib_capture
export BSP_MSP_DIR=/soc/lib
export LD_LIBRARY_PATH=/soc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

使用 IMU 时再执行：

```bash
export ROS_DOMAIN_ID=83
source /opt/ros/humble/setup.bash
```

探测与采集：

```bash
python3 script/calibration_data.py probe --resolution 3840x2160 --samples 50
python3 script/calibration_data.py stereo-triggered --poses 20 --output ./data
python3 script/calibration_data.py stereo-baselink-triggered --output ./data --resolution 3840x2160
python3 script/calibration_data.py stereo-continuous --save-rate-hz 5 --output ./data
python3 script/calibration_data.py stereo-imu --save-rate-hz 5 --output ./data
python3 script/calibration_data.py usb-continuous --save-rate-hz 5 --output ./data
python3 script/calibration_data.py usb-baselink-triggered --output ./data
python3 script/calibration_data.py imu-continuous --duration-s 3600 --output ./data
```

实验性 RAW：

```bash
python3 script/calibration_data.py stereo-raw-triggered --poses 3 --output ./data
```

PC 裸 Bayer RAW16 转彩色和灰度 PNG：

```powershell
python script/raw_to_png.py image.raw
```

校验和 ROS1 bag 导出：

```bash
python3 script/calibration_data.py validate ./data/stereo_260918_1200
```

```powershell
python script/calibration_data.py export-ros1 data/stereo_260918_1200 data/stereo_260918_1200.bag
```

## 固定约束

- 双目数据的 cam0 为左目 VIN pipe 2/chn 0，cam1 为右目 pipe 0/chn 0；USB 单相机数据只有 cam0。
- 正式标定源格式为 NV12，支持 3840×2160 或 1920×1080。
- USB 图像固定从 IVPS grp 3/chn 0 获取，固定 1280×960、源 30 Hz；默认保存 rgb8 24-bit BMP，可用 `--pixel-format mono8` 改为灰度。
- `--resolution` 只校验上游输出，不修改相机管线。
- 默认双目保存 5 Hz、mono8 BMP、两个图像线程；USB 默认 5 Hz、RGB BMP；IMU 全量保存。
- 左右 PTS 差默认不超过 200 µs。
- 输出目录为 `<output>/<dataset_name>_YYMMDD_hhmm`。
- 采集先写 `.inprogress`，失败保留 `.failed`。
- 三个连续相机模式省略 `--duration-s` 时持续采集，第一次 Ctrl+C 正常收尾。
- 板端只部署 `script/`，不安装 PC 分析依赖。

维护规则见 [AGENTS.md](../AGENTS.md)。
