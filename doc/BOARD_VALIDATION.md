# 板端验证记录：2026-09-07 / v1.5.1

默认双目 5 Hz 的 60 秒联合采集完成，结构与时序检查无硬错误，写盘队列未丢对。数据集状态为 `warning`：保留源序号缺口及板端未提供 SDK 头文件两项质量提示。

## 环境与命令

只同步 `script/` 的源码、配置和测试；未同步 `doc/`、`AGENTS.md`、`data/`、`tmp/` 或 PC 专用导出依赖清单，未安装或下载包、库。板端无 rsync，使用已有 SCP。没有更改上游相机配置。

在 Docker `ax650_aarch64v1.1_test` 内执行：

```bash
cd /workspace/minsky/Vincent/calib_capture
export BSP_MSP_DIR=/soc/lib
export LD_LIBRARY_PATH=/soc/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export ROS_DOMAIN_ID=83
source /opt/ros/humble/setup.bash
python3 -B script/calibration_data.py camera-imu --duration-s 60
python3 -B script/calibration_data.py probe --with-imu --samples 50
```

采集命令未传 `--save-rate-hz`、`--frame-divisor` 或 `--output`；会话元数据确认默认值分别为 `5.0`、`0`，输出位于：

```text
/workspace/minsky/Vincent/calib_capture/data/2026-09-07_18-17-39
```

## 60 秒实采结果

| 检查项 | 结果 |
|---|---:|
| 输入 | 左 VIN pipe 2/chn 0；右 pipe 0/chn 0；NV12 3840×2160 |
| 左 / 右图像 | 301 / 301 张，无损 mono8 BMP |
| 左 / 右实际保存频率 | 5.000000 / 5.000000 Hz |
| IMU CSV / JSONL | 6000 / 6000 条，内容与时间戳一致 |
| IMU 有效频率 | 99.859929 Hz |
| IMU 最大间隔 / 长间隔数 | 10.055416 ms / 0 |
| 最大双目时间差 | 7 µs |
| 相机–IMU共同时间窗口 | 59.997103 秒 |
| 写盘队列丢对 | 0 |
| AX 非零返回 / 退出返回 | 无 / 0 |
| 每对写入平均 / 最大耗时 | 134.528 / 452.885 ms |
| 每对写入累计耗时 | 40.493 秒 |
| 源序号缺口 | 左 42、右 42；无序号回退 |

首对立即保存，末对 PTS 距首对约 60 秒，因此本次包含 301 对，而不是恰好 300 对。IMU 独立接收和保存，没有随图像降为 5 Hz。

写入计时包括两张 BMP、重命名和 CSV 更新，不含取帧、排队等待或 `fsync`，不能视为持久化到物理介质的耗时。偶发写盘耗时超过 200 ms，本次由队列吸收，未丢弃目标双目对。

收尾后再次只读运行完整结构校验，并完整解码左右首尾共 4 张 BMP：尺寸均为 3840×2160，像素字节数均为 8294400。无单侧缺失、时间倒退、同步超限或截断文件；没有生成数据 SHA-256 清单。

## 自检与范围

- PC：42 项测试，38 通过、4 跳过（2 项 PyYAML、2 项 rosbags）；SDK 头文件契约与 ctypes ABI 通过。
- 板端：42 项测试，40 通过、2 项 rosbags 跳过；固定 ctypes ABI 通过。
- 4K `probe`：左右源频率均为 50 Hz；最大双目差 4 µs；IMU 99.859739 Hz；无 AX 错误。
- 板端源序号缺口只记录观察值，未定位原因；它不等于保存数据缺失，主动限频造成的序号跳跃另行统计。
- 未在板端复制 SDK 头文件，因此未执行板端头文件哈希核验。
- 未切换上游分辨率，1080p 实采未复测；未执行 PC ROS1 bag 回读或 Kalibr 加载验收。本次未安装这些可选依赖。

本记录及其他文档仅保留在 PC/Git；测试数据保留在板端工作区，不纳入 Git。

## INI 配置迁移补充：2026-09-07 / v1.6.0

运行配置改为 `script/default_config.ini`，使用 Python 标准库 `configparser`，不再直接依赖 PyYAML。数据集格式及采集、写盘逻辑不变。

- PC 自检：53 项，51 通过、2 项 rosbags 跳过；SDK 头文件契约通过。
- 板端使用 `python3 -S -B script/calibration_data.py self-test` 禁用第三方 site-packages 后，53 项中 51 通过、2 项 rosbags 跳过。
- 默认 INI 构建的配置与不传 `--config` 的内置配置完全一致。INI/BOM、注释、字面值、CLI 优先级、Hz/分频互斥及非法配置拒绝均有测试覆盖。
- 仅同步相关脚本、INI 模板、测试及采集依赖清单；没有安装任何包、库，没有同步文档或数据。板端旧 YAML/JSON 模板改名为 `default_config.yaml.legacy`、`default_config.json.legacy` 留作迁移参考，不再读取。

在前述板端环境下执行：

```bash
python3 -B script/calibration_data.py --config script/default_config.ini probe --with-imu --samples 50
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --duration-s 5
```

INI `probe` 通过：NV12 3840×2160，左右源频率均为 50 Hz，最大双目差 4 µs，IMU 99.859750 Hz，无 AX 错误。

短时实采输出为 `/workspace/minsky/Vincent/calib_capture/data/2026-09-07_18-40-21`：

- 左右各 26 张，实际保存频率约 5 Hz；IMU 共 507 条，99.810108 Hz，无长间隔。
- 最大双目时间差 5 µs，写盘队列丢对 0，AX 返回错误 0。
- 平均每对写入 140.979 ms，最大 251.018 ms。
- 会话配置确认 `duration_s = 5` 覆盖 INI 中的 60 秒；`save_rate_hz = 5`、`frame_divisor = 0` 保持不变。
- 结构与时序检查无硬错误；状态为 `warning`，原因仍为板端无 SDK 头文件及源序号缺口（左右各 10）。

同时只读重验上面的 v1.5.1 一分钟数据，仍无硬错误。本次只改配置入口，执行了 5 秒端到端回归，未重复一分钟录制；1080p 实采、ROS1 bag 回读和 Kalibr 加载未新增测试。
