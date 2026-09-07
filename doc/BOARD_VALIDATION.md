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

## OpenCV JPEG95 补充：2026-09-07 / v1.7.0

JPEG 直接使用板端现有 OpenCV 4.5.4、NumPy；OpenCV 的 JPEG 后端为系统 libjpeg（ver 80），没有安装包或库，也没有使用 Pillow。BMP 默认值不变，INI 可配置 `image_format = jpeg`、`jpeg_quality = 95`。

最初左右串行编码的 10 秒实采得到 37 对、3.673 Hz，队列丢弃 14 对；平均每对写入 282.743 ms。使用该数据解码后的同一对灰度图做 5 次内存编码比较：串行平均 250.456 ms，并行平均 127.758 ms，文件大小相同。因此 JPEG 改为两个复用线程编码左右图；两侧全部完成后才提交文件和 CSV，失败时也等待两侧结束后清理临时文件。

最终实采命令：

```bash
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format jpeg --jpeg-quality 95 --duration-s 10
```

输出 `/workspace/minsky/Vincent/calib_capture/data/2026-09-07_20-09-48`：

| 项目 | 结果 |
|---|---:|
| 实际分辨率 / 格式 | 3840×2160，灰度 JPEG95 |
| 双目数量 / 频率 | 51 对 / 5.000 Hz |
| IMU 数量 / 频率 | 1011 条 / 99.859392 Hz |
| 写盘队列丢对 | 0 |
| 最大双目差 / IMU 长间隔 | 0 µs / 0 次 |
| 编码加写入平均 / 最大耗时 | 161.036 / 195.165 ms 每对 |
| 102 张 JPG 总大小 | 154,457,627 bytes |
| 每张 JPG 平均大小 | 1,514,290 bytes（约 1.51 MB） |
| 同尺寸 BMP 等效大小 | 846,138,756 bytes |
| 本画面存储减少 | 81.75% |

结构与完整解码校验无硬错误。状态为 `warning`：JPEG 有损、板端无 SDK 头文件、源序号缺口（左右各 3）。以上压缩率取决于画面，不能保证其他场景相同；写入耗时不含取帧、排队等待或 `fsync`。

PC 测试 63 项：56 通过、7 跳过（缺 OpenCV/NumPy 及 rosbags）。板端测试 63 项：60 通过、3 项 rosbags 回读跳过；JPEG 编解码、质量、损坏图像、配对失败清理、两线程重叠、旧 schema 1/2 BMP 兼容均通过。bag 图像构造的 JPEG→mono8 路径已单测，完整 ROS1 bag 回读与 Kalibr 加载仍未执行。未重录一分钟，也未切换 1080p。

## 有界流水线优化：2026-09-07 / v1.8.0

默认继续为 4K、BMP、双目 5 Hz、图像线程 2；JPEG95 可选，IMU 全量保存。只同步 `script/`，未安装任何包或库，未修改系统/相机环境，未部署文档、AGENTS.md 或数据，也未提交/推送 Git。目录数据集及 PC 离线 bag 导出保留，不增加板端直接录 bag。

### 实现与自检

- `image_workers`／`--image-workers` 范围 1–8。每图一个任务，最多同时处理 `ceil(workers/2)` 对；一个协调线程按采集顺序提交。默认等待队列 2 对，完整对满载丢弃。
- Y 平面直接复制到可复用缓冲；BMP 在复制时形成 bottom-up 行序，JPEG 保持 top-down。缓冲池按需分配，上限为等待队列 + 同时处理对数 + 1。所有正常和失败任务结束后才可复用缓冲。
- CSV/IMU JSONL 每 0.5 秒刷新，退出最终刷新，不增加逐帧 fsync。性能计数器固定容量；JPEG 解码校验不再额外复制整张像素 bytes。
- PC：76 项，69 通过、7 跳过（OpenCV/NumPy 及 rosbags）；SDK 头文件契约及 ctypes ABI 通过。
- 板端现有环境：76 项，73 通过、3 项 rosbags 跳过。覆盖 stride、无 NumPy 回退、BMP 字节一致性、跨对并行/顺序提交、缓冲上限/归还、队列饱和、工作线程及部分提交失败、后续任务取消、后台刷新失败及最终刷新。
- 板端 `python3 -S -B ... self-test`：76 项，69 通过、7 项可选依赖测试跳过，确认没有第三方 site-packages 时 BMP 路径仍可用。
- 4K `probe --with-imu --samples 50`：NV12 3840×2160，左右源 PTS 频率 50 Hz，最大双目差 4 µs，IMU 99.859703 Hz，无 AX 错误。未切换上游分辨率，1080p 实采未复测。

### 默认 5 Hz，一分钟联合采集

命令均在板端项目顶层、既有 ROS 2/AX 环境中执行：

```bash
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format bmp --duration-s 60
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format jpeg --jpeg-quality 95 --duration-s 60
```

| 项目 | BMP | JPEG95 |
|---|---:|---:|
| 数据目录（板端 data/ 下） | 2026-09-07_21-19-28 | 2026-09-07_21-21-32 |
| 左右图像数量 | 各 301 张 | 各 301 张 |
| 双目实际频率 | 约 5.000 Hz | 约 5.000 Hz |
| IMU 数量 / 频率 | 6000 / 99.847157 Hz | 5999 / 99.846881 Hz |
| IMU 长间隔 | 0 | 0 |
| 最大双目差 | 7 µs | 6 µs |
| 写盘队列丢对 / AX 错误 | 0 / 0 | 0 / 0 |
| 源序号缺口 | 左 2、右 2 | 左 2、右 2 |
| 每对任务到提交平均 / 最大延迟 | 58.261 / 164.406 ms | 155.929 / 189.182 ms |
| 录制阶段峰值 RSS | 79.08 MiB | 191.61 MiB |
| 实际缓冲分配 | 1 对，15.82 MiB | 2 对，31.64 MiB |
| 平均进程 CPU 核数（含排空） | 1.09 | 1.99 |
| 结束排空 | 0.0025 s | 0.0278 s |
| 收尾校验时间 | 2.52 s | 56.47 s |
| 图像总字节数 | 4,993,877,756 | 948,268,787 |

两份数据均无结构/时序硬错误，左右时间戳严格递增、有相机–IMU共同窗口，缓冲槽位最终全部归还。警告为源序号缺口、板端缺 SDK 头文件，以及 JPEG 有损。IMU 保存所有收到的回调消息；计数不代表能够证明发布端没有丢包。

另取 BMP 一分钟数据左右首尾共 4 张，以 OpenCV 和项目 BMP 读取器交叉完整解码：尺寸均为 3840×2160，灰度像素字节完全一致。所有测试结束后工作区所在盘剩余约 14.68 GB，没有删除旧数据。

历史 BMP 一分钟每对平均延迟约 134.528 ms，本次 58.261 ms；这是不同时段的对照，不是严格的相同负载 A/B。JPEG 每图编码平均 129.737 ms、写文件平均 13.015 ms，编码仍是主要开销。JPEG 完整解码校验保留，因此一分钟录制结束后还需要约 56 秒校验，不应将这部分视为采集丢帧或写盘队列积压。

### 显式 20 Hz 探索，每次 10 秒

```bash
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format bmp --save-rate-hz 20 --image-workers 4 --duration-s 10
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format jpeg --jpeg-quality 95 --save-rate-hz 20 --image-workers 6 --duration-s 10
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format jpeg --jpeg-quality 95 --save-rate-hz 20 --image-workers 8 --duration-s 10
```

| 项目 | BMP / 4 线程 | JPEG95 / 6 线程 | JPEG95 / 8 线程 |
|---|---:|---:|---:|
| 数据目录 | 2026-09-07_21-24-54 | 2026-09-07_21-25-08 | 2026-09-07_21-25-59 |
| 保存对数 / 实际 Hz | 171 / 17.000 | 145 / 14.458 | 151 / 15.030 |
| 队列丢对 | 9 | 43 | 4 |
| 观察到的源序号缺口（两侧总数） | 578 | 340 | 626 |
| 录制阶段峰值 RSS | 126.50 MiB | 254.13 MiB | 277.09 MiB |
| 缓冲实际分配 / 容量（对） | 4 / 5 | 5 / 6 | 6 / 7 |
| 同时处理对数峰值 | 2 | 3 | 4 |
| 平均进程 CPU 核数 | 2.25 | 4.47 | 4.62 |
| 结束排空 | 0.080 s | 0.171 s | 0.114 s |

三个探索数据集均无结构硬错误、AX 错误或 IMU 长间隔，但明显未达到稳定 20 Hz；观察到的源取帧频率降低及序号缺口包含消费者处理不及时的影响，不能据此断言相机生产端降频。各队列、并发数和缓冲池均有界，结束占用归零。

以上保存频率根据传感器 PTS 计算，生产采集没有逐帧 fsync；10 秒结果不等于存储介质长期持久化带宽。维持默认 5 Hz，不因短测结果提高默认频率。完整 ROS1 bag 回读与 Kalibr 实际加载未执行，未为这些测试安装依赖。全部采集数据保留在板端，本记录仅在 PC/Git 工作区。

## 10 Hz 一分钟测试补充：2026-09-07 / v1.8.0

用户要求测试 10 Hz。保持 3840×2160、原 VIN 映射、IMU 全量保存和现有运行环境不变，分别录制 60 秒。只用 CLI 临时覆盖，默认 INI 和代码仍为 5 Hz、2 个图像线程；没有安装依赖、修改代码、删除旧数据或提交/推送 Git。

```bash
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format bmp --save-rate-hz 10 --image-workers 2 --duration-s 60
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format jpeg --jpeg-quality 95 --save-rate-hz 10 --image-workers 2 --duration-s 60
python3 -B script/calibration_data.py --config script/default_config.ini camera-imu --image-format jpeg --jpeg-quality 95 --save-rate-hz 10 --image-workers 4 --duration-s 60
```

| 项目 | BMP / 2 线程 | JPEG95 / 2 线程 | JPEG95 / 4 线程 |
|---|---:|---:|---:|
| 数据目录（板端 data/ 下） | 2026-09-07_21-35-58 | 2026-09-07_21-37-39 | 2026-09-07_21-40-35 |
| 左右图像数量 | 各 601 张 | 各 397 张 | 各 601 张 |
| 实际保存频率 | 10.000 Hz | 6.611 Hz | 9.997 Hz |
| 写盘队列丢对 | **0** | **204** | **0** |
| IMU 数量 / 频率 | 6003 / 99.855638 Hz | 6000 / 99.851353 Hz | 6001 / 99.843376 Hz |
| 最大双目差 | 8 µs | 6 µs | 6 µs |
| 源序号缺口（左右分别） | 8 / 8 | 3 / 3 | 3 / 3 |
| 每对任务到提交平均延迟 | 64.145 ms | 151.361 ms | 170.201 ms |
| 录制峰值 RSS | 110.77 MiB | 207.30 MiB | 230.54 MiB |
| 平均进程 CPU 核数 | 1.57 | 2.39 | 3.38 |
| 实际缓冲分配 / 容量（对） | 3 / 4 | 3 / 4 | 4 / 5 |
| 结束排空 | 0.0013 s | 0.1856 s | 0.0533 s |
| 收尾校验 | 4.14 s | 71.92 s | 109.74 s |

三份数据均通过结构与时序校验，无 AX 错误、无 IMU 长间隔，缓冲池占用最终归零。JPEG 全部逐张解码校验，校验时间不计入录制频率。四线程允许两对图像同时处理，因此每对延迟约 170 ms 仍可实现约 100 ms 的输出间隔；不能用单对延迟的倒数计算并发流水线吞吐。

结论：当前负载下，**BMP 两线程、JPEG95 四线程均通过一分钟 10 Hz 零写盘队列丢对测试**；JPEG95 两线程不满足 10 Hz。该结论不代表无限时长保证，也不代表整个链路绝对零丢帧：仍有少量源序号缺口、板端缺 SDK 头文件，以及 JPEG 有损警告。默认继续保持 5 Hz。测试后盘上剩余约 12.91 GB，全部本轮采集数据保留在板端。
