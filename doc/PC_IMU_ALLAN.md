# PC 端 IMU 诊断与 Allan 分析

本文统一说明 IMU 时域／白噪声诊断和三套 Allan 分析流程：

1. 项目原生时域与白噪声诊断；
2. 项目原生 `allan_variance` 分析（命令 `allan`，Python AllanTools）；
3. ROS1 `ALLAN_VARIANCE_ROS`（程序名 `allan_variance_ros`）；
4. ROS1 `IMU_UTILS`（程序名 `imu_utils`）。

这些流程都只读板端 `imu-continuous` 生成的 CSV，不修改原始数据。建议先运行 Python
AllanTools 快速检查，再让两个 ROS 工具使用同一时间区间、同一个派生 bag 做交叉分析。

## 1. 输入准备

板端采集：

```bash
python3 script/calibration_data.py imu-continuous \
  --duration-s 86400 \
  --imu-topic /imu/data \
  --imu-temperature-topic /imu/temperature \
  --output ./data
```

把完整会话复制到 PC：

```text
data/allan_variance/data/<会话名>/
  imu/imu0.csv
  meta/session.json
  meta/validation.json
```

三套工具的共同数据列为：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az,temperature
```

没有温度发布时，采集命令加 `--ignore-temperature`，CSV 为七列
`timestamp_ns,wx,wy,wz,ax,ay,az`。项目内置 `allan` 分析支持这两种格式；无温度输入的报告会标记
`temperature.available: false`，不会补造温度值。

- 时间戳：整数纳秒，严格递增；
- 角速度：rad/s；
- 加速度：m/s²；
- 温度：°C，仅用于摘要，不进入 Allan 六轴计算。

ROS 两套工具只要求八列 CSV；Python AllanTools 还会读取会话元数据进行完整性检查。

## 2. 时间区间规则

所有入口都支持相对第一条原始 IMU 时间戳选择 `[start,end)`：

| 参数 | 含义 |
| --- | --- |
| `--start-s / --end-s` | 秒 |
| `--start-h / --end-h` | 小时，仅 ROS 容器入口 |
| 不传起点 | 从第一条样本开始 |
| 不传终点 | 到文件末尾 |

例如第 1–6 小时：

```text
--start-s 3600 --end-s 21600
```

或在 ROS 容器入口中使用：

```text
--start-h 1 --end-h 6
```

时间戳不会归零。区间目录名显示为 `_1h-6h`；非整数小时保留一位小数，但实际选区仍按
精确纳秒计算。已有输出目录默认拒绝覆盖。

## 3. 时域与白噪声诊断

两个脚本接受完整独立 IMU 会话目录，或直接接受 `<会话>/imu/imu0.csv`。它们严格检查
八列表头、有限数值和递增时间戳；不插值、不重采样，也不修改源文件。PC 环境需要
NumPy、Matplotlib 和 SciPy，依赖列在 `script/requirements-allan.txt`。

### 3.1 时间戳、六轴和温度

默认分析全段：

```powershell
python script/analyze_imu_timeseries.py Q:\File\machine_data\kalibr_no_ros\imu_260917_2234
```

指定相对第一条 IMU 时间戳的 `[start,end)`：

```powershell
python script/analyze_imu_timeseries.py Q:\File\machine_data\kalibr_no_ros\imu_260917_2234 `
  --start-s 3600 --end-s 7200
```

默认输出到源会话同级的 `analysis/<会话名>/timeseries_all/` 或带时间区间的子目录。输出
`timestamp_intervals.png`、`imu_axes_temperature.png`、`summary.json` 和
`statistics.csv`。曲线默认最多绘制 20,000 个 min/max 包络点，减少绘图内存并保留孤立
尖峰；`--max-plot-points` 可调整。时间戳直方图默认沿用时序检查工具的右闭区间：
`<=7.5 ms`、按 1 ms 递增、`(19.5,20] ms`、`>20 ms`，图中标注每档数量及两端比例；
`summary.json` 和 `statistics.csv` 同时保存精确数量与比例。分箱可用 `--hist-min-ms`、
`--hist-max-ms`、`--hist-bin-width-ms` 修改。

min/max 包络会保留每个桶内的最小值和最大值，因此单个孤立异常、正负尖峰和跳变两侧
通常都能显示。它不会保留同一桶中的每一个异常：多个同向尖峰只显示最极端者，跳变的
精确时刻最多有一个桶宽的不确定性。完整 280 万点压到 20,000 点时，一个桶约 280 个
原始样本；定位细节时应缩小 `--start-s/--end-s`，或提高 `--max-plot-points`。

### 3.2 高斯性和白噪声诊断

默认选择完整数据中间的 1,800 秒：

```powershell
python script/analyze_imu_noise.py Q:\File\machine_data\kalibr_no_ros\imu_260917_2234
```

也可以选择精确区间：

```powershell
python script/analyze_imu_noise.py Q:\File\machine_data\kalibr_no_ros\imu_260917_2234 `
  --start-s 3600 --end-s 5400
```

仅指定 `--start-s` 时向后取 `--duration-s`，仅指定 `--end-s` 时向前取；同时指定起止时
不能再显式指定时长。默认结果写入 `analysis/<会话名>/noise_mid_30min/`，包含：

- `noise_distribution.png`：六轴直方图和拟合高斯 PDF；
- `noise_qq.png`：六轴正态 Q-Q 图；
- `noise_acf.png`：六轴 ACF、95% 置信线及工程阈值；
- `noise_psd.png`：六轴 Welch PSD；
- `noise_report.json`、`axis_verdicts.csv`：完整统计量和逐轴结论。

每轴只减去选区均值，不做线性去趋势或滤波。高斯工程条件为
`|skew| <= 0.1`、`|excess kurtosis| <= 0.2`、`Q-Q r >= 0.995`；白噪声工程条件为
前 10 秒 `max |ACF| <= 0.05`。工程条件和显著性检验均通过为 `pass`，只有工程条件通过
为 `approximate`，工程条件不通过为 `fail`。Welch PSD 和谱平坦度只作辅助判断。

这套结论用于检查所选静止段是否近似高斯白噪声；它不拟合 Kalibr 参数，也不等同于
IMU 噪声模型或标定验收。已有输出目录不会被覆盖，失败现场保留为 `.failed`。

## 4. Python AllanTools

### 4.1 环境

在 Windows PowerShell 中进入已有 `calib` 环境：

```powershell
conda activate calib
cd Q:\project\kalibr_cedarobo
```

依赖列在 `script/requirements-allan.txt`。程序不会自动安装依赖。

### 4.2 全段分析

```powershell
python script/calibration_data.py allan data/allan_variance/data/imu_260911_1502
```

也可以使用独立入口：

```powershell
python script/analyze_allan.py data/allan_variance/data/imu_260911_1502
```

默认结果：

```text
data/allan_variance/allantools_output/imu_260911_1502/
```

### 4.3 指定区间并拟合 Kalibr 候选参数

```powershell
python script/calibration_data.py allan data/allan_variance/data/imu_260911_1502 `
  --start-s 3600 --end-s 21600 `
  --taus octave --allow-irregular --kalibr-noise
```

`--allow-irregular` 允许按实测平均频率近似分析不完全等间隔的数据，会在报告中告警；它不
重采样、不插值，也不删除区间内样本。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `dataset` | 必填 | 独立 IMU 会话目录 |
| `--output DIR` | 自动目录 | 完整结果目录，必须不存在 |
| `--rate-hz HZ` | 从时间戳推算 | 显式计算频率 |
| `--start-s / --end-s` | 全段 | 相对第一条样本的秒数区间 |
| `--taus decade|octave` | `decade` | Allan 平均时间点分布 |
| `--allow-irregular` | 关闭 | 允许非均匀采样近似 |
| `--kalibr-noise` | 关闭 | 拟合白噪声和偏置随机游走 |
| `--kalibr-imu-topic TOPIC` | `/imu/data` | 候选 YAML 的 rostopic |

输出包括：

- `allan_wx.csv` 至 `allan_az.csv`；
- Allan deviation 和 variance 曲线；
- `analysis.json`；
- 拟合完整时的候选 `kalibr_imu.yaml`。

如果任一轴缺少可信的 −1/2 或 +1/2 区间，命令不会伪造缺失参数，也不会生成
`kalibr_imu.yaml`；原因保存在 `analysis.json.noise_fit`。

## 5. ROS 容器准备

`allan_variance_ros` 和 `imu_utils` 共用一个长期 ROS1 Noetic 容器。Docker Desktop
运行后，在 WSL 中执行：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose up -d
docker compose ps
```

目录映射：

| Windows | 容器 | 权限 |
| --- | --- | --- |
| `script/imu_utils_docker/allan` | `/scripts` | 只读 |
| `data/allan_variance/data` | `/data` | 只读 |
| `data/allan_variance/imu_utils_output` | `/output` | 读写 |

两个分析任务共享运行锁，不要在同一容器中同时执行。

## 6. imu_utils

### 6.1 一键分析

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose exec -T imu-utils bash /scripts/run.sh \
  /data/imu_260911_1502/imu/imu0.csv \
  --start-h 1 --end-h 6
```

默认输出：

```text
data/allan_variance/imu_utils_output/imu_260911_1502_1h-6h/
```

关键参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| 输入 CSV／会话目录 | 必填 | `/data` 下的八列 CSV |
| `--output DIR` | 自动目录 | `/output` 下的新目录 |
| `--start-h/s、--end-h/s` | 全段 | 时间选区 |
| `--topic TOPIC` | `/imu/data` | 派生 ROS1 bag 话题 |
| `--frame-id ID` | `imu_link` | CSV 没有 frame ID，因此由用户指定 |
| `--playback-rate N` | `100` | rosbag 回放倍速 |
| `--max-cluster N` | `100` | imu_utils 聚类参数 |
| `--analysis-timeout SEC` | `1800` | 回放结束后等待计算的超时 |
| `--convert-only` | 关闭 | 只生成并逐条验证 bag |

流程会：

1. 把 CSV 转为派生 ROS1 bag；
2. 逐条回读核对时间戳、六轴、topic 和 frame ID；
3. 回放给 `imu_utils`；
4. 要求六轴 `numData` 全部等于 bag 消息数；
5. 生成 YAML、三张 PNG 和 Markdown 报告。

`gyr_w/acc_w` 是上游 bias instability 最低值，不能直接当作 Kalibr bias random walk。

完整的容器内手动操作、验证脚本和产物解释见
[imu_utils 专项说明](docker/IMU_UTILS.md)。

## 7. allan_variance_ros

### 7.1 独立一键分析

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv \
  --start-h 1 --end-h 6
```

默认输出：

```text
data/allan_variance/imu_utils_output/imu_260911_1502_1h-6h_allan_variance_ros/
```

### 7.2 与 imu_utils 复用同一个 bag

先运行上一节的 `imu_utils`，再执行：

```bash
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv \
  --start-h 1 --end-h 6 \
  --bag /output/imu_260911_1502_1h-6h/imu_260911_1502_1h-6h.bag \
  --output /output/imu_260911_1502_1h-6h_allan_variance_ros
```

`--bag` 必须与 CSV、选区、topic 和 frame ID 一致。入口仍会逐条回读验证；结果目录建立
相对符号链接，不复制 bag。

关键参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| 输入 CSV／会话目录 | 必填 | `/data` 下的八列 CSV |
| `--output DIR` | 自动目录 | `/output` 下的新目录 |
| `--bag FILE` | 自动转换 | 复用匹配的已有 bag |
| `--start-h/s、--end-h/s` | 全段 | 时间选区 |
| `--topic TOPIC` | `/imu/data` | bag 话题 |
| `--frame-id ID` | `imu_link` | 派生消息 frame ID |
| `--threads N` | `2` | C++ 计算线程数 |
| `--timeout SEC` | `7200` | 每个受控子命令的超时 |

主要结果是 `imu.yaml`、`acceleration.png`、`gyro.png`、中文报告及
`work/allan_variance.csv`。YAML 为 Kalibr 连续时间噪声参数候选，不要再次乘除
√采样率。

完整参数、手动执行和故障处理见
[allan_variance_ros 专项说明](docker/ALLAN_VARIANCE_ROS.md)，算法和拟合细节见
[源码解读](ALLAN_VARIANCE_ROS_SOURCE.md)。

## 8. 三套结果如何使用

| 工具 | 优点 | 主要限制 |
| --- | --- | --- |
| Python AllanTools | 启动快；直接读取会话；适合检查多个时间区间 | 非均匀采样需显式近似；自动拟合可能因缺少可信上升段而不输出 YAML |
| imu_utils | 五项模型拟合；有完整 bag 回读和消息无丢失检查 | ROS 回放耗时；`gyr_w/acc_w` 不是 Kalibr random walk |
| allan_variance_ros | 直接输出 Kalibr 格式候选；可与 imu_utils 共用 bag | 固定分箱和拟合策略仍需人工审查 |

推荐顺序：

1. 用 Python AllanTools 检查全段和多个子区间；
2. 选定静止、温度较稳定且时长足够的区间；
3. 用 `imu_utils` 生成并验证派生 bag；
4. 将同一个 bag 交给 `allan_variance_ros`；
5. 对比三套曲线和参数，人工检查 −1/2、最低点及 +1/2 区域；
6. 将审查后的参数用于 Kalibr，并通过实际标定结果继续验证。

`PASS` 只表示输入、计算和产物检查通过，不代表 IMU 噪声模型或最终标定质量已经验收。
