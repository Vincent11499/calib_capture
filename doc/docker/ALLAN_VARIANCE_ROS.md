# allan_variance_ros 容器与使用说明

更新日期：2026-09-15。适用于本项目现有 ROS1 Noetic 容器与当前脚本。

`allan_variance_ros` 直接离线读取 bag，计算六轴 Allan 标准差，再由上游 `analysis.py` 拟合并生成图像和 Kalibr 参数候选。本项目入口另外负责 CSV 时间选区、bag 转换或复用、回读、输出检查及中文报告。

算法、拟合区间与单位细节见 [源码解读](../ALLAN_VARIANCE_ROS_SOURCE.md)；另一个工具见 [imu_utils 使用说明](IMU_UTILS.md)。

## 1. 环境与目录

| 项目 | 当前配置 |
|---|---|
| 项目根目录 | `Q:\project\kalibr_cedarobo` |
| 长期容器 | `kalibr-imu-utils-noetic` |
| Compose 服务 | `imu-utils` |
| 镜像 | `kalibr-imu-utils-noetic:local` |
| 上游工作空间 | `/allan_ws` |
| 上游固定提交 | `1d54b602ee7f2ba0427865d63afe4945d913ed24` |
| 容器行为 | `sleep infinity`，`restart: unless-stopped` |

当前上游 C++ 和 Python 算法没有修改；`patch_sample_limit.py` 仅用于 imu_utils，不用于 allan_variance_ros。

| Windows 路径（相对项目根） | 容器路径 | 用途 |
|---|---|---|
| `script/imu_utils_docker/docker/` | 不挂载 | Compose、Dockerfile、安装脚本 |
| `script/imu_utils_docker/allan/` | `/scripts`，只读 | CSV、分析和报告脚本 |
| `data/allan_variance/data/` | `/data`，只读 | 原始输入 |
| `data/allan_variance/imu_utils_output/` | `/output`，读写 | 两种工具的独立结果目录 |

当前 Compose 使用上述三个 bind mount，没有挂载 catkin 命名卷。`/allan_ws` 在镜像／容器内部；安装脚本和 Dockerfile 可在重建时恢复固定版本。退出交互终端不会删除容器。

## 2. 启动与检查

Docker Desktop 运行后，从 PowerShell 进入 WSL：

```powershell
wsl.exe -d Ubuntu-22.04
```

以下命令在 **WSL 宿主机**执行：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose up -d
docker compose ps
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh --help
```

需要重建镜像时，在没有分析任务运行的情况下执行：

```bash
docker compose build
docker compose up -d
```

仅修改 bind mount 中的 `/scripts` 脚本或项目文档，不需要重建镜像。

## 3. 输入与时间范围

输入可为数据集目录，也可直接指定 `<数据集>/imu/imu0.csv`。表头严格为：

```text
timestamp_ns,wx,wy,wz,ax,ay,az,temperature
```

时间单位为整数纳秒，角速度 rad/s，加速度 m/s²，温度为摄氏度。入口只读取 CSV，不要求 meta 文件；完整检查列数、有限值、时间戳严格递增后选择区间。温度不进入 bag 和 Allan 计算。默认 frame ID 是 `imu_link`，可用 `--frame-id` 指定。

| 参数 | 含义 |
|---|---|
| `--start-h` / `--end-h` | 相对源 CSV 首条 IMU 时间戳的小时数 |
| `--start-s` / `--end-s` | 同一相对时间，单位秒 |
| 不传起点 | 从 0 开始 |
| 不传终点 | 到源文件末尾 |

同一边界不能同时指定秒和小时；支持小数，选择精度为整数纳秒。区间包含起点、不包含终点。例如 `--start-h 1 --end-h 23` 等价于 `--start-s 3600 --end-s 82800`，共 22 小时。

保留原始消息时间戳和六轴值，时间戳不归零；没有插值、重采样、温补或删去区间内异常样本。已有格式／时间错误会使任务失败，而不是自动修复源文件。

## 4. 一键分析

### 4.1 分析第 1–23 小时

**WSL 宿主机**执行：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 23
```

此命令表示首次运行：转换一次选段 bag，再校验、计算、拟合与生成报告。默认结果目录：

```text
容器：/output/imu_260911_1502_1h-23h_allan_variance_ros
Windows：Q:\project\kalibr_cedarobo\data\allan_variance\imu_utils_output\imu_260911_1502_1h-23h_allan_variance_ros
```

**本次结果已经存在，直接重执行会拒绝覆盖。** 要重跑分析并复用现有 bag，使用下一节命令。

### 4.2 使用已有 bag 重跑，不重复转换

```bash
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 23 \
  --bag /output/imu_260911_1502_1h-23h_allan_variance_ros/imu_260911_1502_1h-23h.bag \
  --output /output/imu_260911_1502_1h-23h_allan_variance_ros_rerun
```

`--output` 必须是尚不存在的完整结果目录，重复运行本例时需另选新名字。`--bag` 必须与 CSV、时间选区、topic 和 frame ID 匹配；入口仍会逐条验证。新目录顶层建立指向原 bag 的相对符号链接，不复制文件。保留被引用的原 bag；删除或移动它会使链接失效。

### 4.3 与 imu_utils 使用同一个 bag

已有第 1–6 小时的 imu_utils bag，可直接供 allan_variance_ros 使用：

```bash
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 6 \
  --bag /output/imu_260911_1502_1h-6h/imu_260911_1502_1h-6h.bag \
  --output /output/imu_260911_1502_1h-6h_allan_variance_ros_rerun
```

用于其他新数据时，先运行 imu_utils 生成目标区间 bag，再将该路径传入 `--bag`。两个引擎比较必须使用同一选区；不能把 1–6 小时的 bag 用于 1–23 小时计算。

### 4.4 全段分析

```bash
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv
```

不传时间参数即分析完整源数据；默认输出 `<数据集名>_allan_variance_ros`。指定时间则加小时后缀，如 `_1h-23h`；非整数小时显示至一位小数，仅影响命名，不降低选区精度。显示舍入造成同名时仍拒绝覆盖。

## 5. 进入容器操作

**WSL 宿主机**：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose exec imu-utils bash
```

以下在 **容器内**执行，可通过同一入口完成所有步骤：

```bash
bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 23 \
  --bag /output/imu_260911_1502_1h-23h_allan_variance_ros/imu_260911_1502_1h-23h.bag \
  --output /output/imu_260911_1502_1h-23h_allan_variance_ros_container
```

包装入口自行加载 Noetic 和 `/allan_ws`、启动和清理自己的 roscore。它与 imu_utils 共享锁，不要在同一容器同时启动两个分析任务。

### 5.1 容器内手动执行上游各阶段

以下是低层操作示例，复用已有 1–23 小时 bag；没有一键任务运行时再执行，手动操作不自动获取包装入口的运行锁。

```bash
source /opt/ros/noetic/setup.bash
source /allan_ws/devel/setup.bash
rospack find allan_variance_ros

OUT=/output/imu_260911_1502_1h-23h_allan_variance_ros_manual
# 若目录已经存在，mkdir 会失败；此时停止本例并换一个新 OUT。
mkdir "$OUT"
mkdir -p "$OUT/work/bags"
ln -s ../imu_260911_1502_1h-23h_allan_variance_ros/imu_260911_1502_1h-23h.bag \
  "$OUT/imu_260911_1502_1h-23h.bag"
ln -s "$OUT/imu_260911_1502_1h-23h.bag" \
  "$OUT/work/bags/imu_260911_1502_1h-23h.bag"

python3 /scripts/validate_rosbag.py /data/imu_260911_1502/imu/imu0.csv \
  "$OUT/imu_260911_1502_1h-23h.bag" --start-h 1 --end-h 23
```

上一步必须成功，再创建配置。以下值**仅对应当前这份 1–23 小时数据**；其他输入请使用一键入口自动统计配置。

```bash
cat > "$OUT/work/config.yaml" <<'YAML'
imu_topic: /imu/data
imu_rate: 100.53508534939816
measure_rate: 100
sequence_time: 79201
YAML

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
export ROS_LOG_DIR="$OUT/work/ros_logs"
roscore > "$OUT/work/roscore.log" 2>&1 &
ALLAN_MASTER_PID=$!
# 等待 rosparam list 成功后继续；失败时先查看 roscore.log。
rosparam list

OMP_NUM_THREADS=2 rosrun allan_variance_ros allan_variance \
  "$OUT/work/bags" "$OUT/work/config.yaml" > "$OUT/work/computation.log" 2>&1
```

确认命令成功及读取数量一致后，再拟合与绘图：

```bash
mv "$OUT/work/bags/allan_variance.csv" "$OUT/work/allan_variance.csv"
cd "$OUT"
MPLBACKEND=Agg python3 /allan_ws/src/allan_variance_ros/scripts/analysis.py \
  --data "$OUT/work/allan_variance.csv" --config "$OUT/work/config.yaml" \
  --output "$OUT/imu.yaml" > "$OUT/work/analysis.log" 2>&1

# 完成或中断手动步骤后，清理本例自行启动的 roscore。
kill -INT "$ALLAN_MASTER_PID"
wait "$ALLAN_MASTER_PID"
```

手动上游命令产生 CSV、YAML 和 PNG，**不自动生成本项目中文报告、run.json 和五尺度独立复算记录**。需要完整产物时使用一键入口。上游只处理目录中排序后的第一个 bag，所以 `work/bags/` 只放一个输入链接。

### 5.2 只生成区间 bag

若尚无匹配的 bag，可在容器内独立转换；目标 bag 必须不存在：

```bash
source /opt/ros/noetic/setup.bash
python3 /scripts/csv_to_rosbag.py /data/imu_260911_1502/imu/imu0.csv \
  /output/imu_260911_1502_bag_1h-23h/imu_260911_1502_1h-23h.bag \
  --start-h 1 --end-h 23
```

随后给分析入口传该 `--bag`，不再转换。这是独立导出的可选操作；已存在匹配 bag 时直接复用。

## 6. 参数与计算配置

| 入口参数 | 默认值／说明 |
|---|---|
| dataset | 必填：`/data` 下的数据集目录或其 `imu/imu0.csv` |
| --start-h / --end-h | 小时选区；也可用秒参数 |
| --output | 新的完整结果目录，必须在 `/output` 下 |
| --bag | 引用已有、与所选 CSV 匹配的 bag；不传则转换一次 |
| --topic | `/imu/data` |
| --frame-id | `imu_link` |
| --threads | 2，通过 OMP_NUM_THREADS 控制 C++ 计算 |
| --timeout | 7200 秒，适用于每个受控子命令，不是整个流程总时限 |

自动配置 `imu_rate=(N−1)/(t_last−t_first)`，`measure_rate=floor(imu_rate)`，并要求 `int(imu_rate/measure_rate)=1`，确保不会因上游抽取步骤丢弃消息。`sequence_time=ceil(N/imu_rate)+1` 覆盖选段。

上游固定计算 0.1–999.9 秒、步长 0.1 秒，共 9,999 点，实际为非重叠分箱。入口要求整数 measure_rate 至少 10 Hz，且消息数足够支持最大尺度的两个箱；不足时拒绝，不自动修改上游范围。

分箱按样本数及整数 measure_rate，未按真实 dt 加权。1–23 小时数据平均频率约 100.535 Hz，但 measure_rate 为 100 Hz；图上 tau=999.9 秒的箱按平均频率估计约为 994.58 秒。均匀周期近似及局部抖动的解释见源码文档。

## 7. 输出文件与查看方式

```text
imu_260911_1502_1h-23h_allan_variance_ros/
  imu_260911_1502_1h-23h.bag       # 新转换文件；复用模式则为链接
  imu.yaml                        # 上游 Kalibr 参数候选
  acceleration.png
  gyro.png
  allan_variance_ros_report.md
  work/
    allan_variance.csv            # 空格分隔、无表头，实际是标准差
    config.yaml
    run.json                      # status: running / pass / fail
    validation.json
    imu_260911_1502_1h-23h.validation.json
    bag_validation.log
    computation.log
    analysis.log
    upstream_revision.txt
    roscore.log
    ros_logs/
    bags/                         # 指向顶层 bag 的链接
```

新转换时另有 `conversion.log` 和 `*.conversion.json`。`workflow.log` 是本次测试另外归档的入口日志，不是每次入口自动生成；共享文件检查 `shared_bag_check.json` 也是相应历史验证额外保存的记录。

Windows 可直接打开输出目录中的 Markdown、PNG 和 YAML。无需打开图形桌面；上游绘图采用 Agg 后台输出。入口运行时终端会等待，进度详见 `work/`。从另一个容器终端检查：

```bash
tail -n 20 /output/imu_260911_1502_1h-23h_allan_variance_ros/work/computation.log
cat /output/imu_260911_1502_1h-23h_allan_variance_ros/work/run.json
```

### 图形标记与 YAML

| 标记／线 | 意义 |
|---|---|
| 红／绿／蓝 | X／Y／Z 轴 |
| 紫线、圆点 | 约 −1/2 斜率拟合，取 tau=1 秒值作为白噪声系数 |
| 黄线、五角星 | 约 +1/2 斜率拟合，取 tau=3 秒值作为偏置随机游走系数 |
| 三角形 | 实测曲线最低点，上游称 Bias Instability；不直接写入 Kalibr random_walk |
| 黑线 | 三轴平均参数合成的模型，不是各轴分别最佳拟合 |

YAML 的四个噪声参数是连续时间系数，按三轴最大值聚合；陀螺仪图用 deg/s，写 YAML 时转换为弧度制。不要再次将 YAML 噪声值乘除 √采样率再交给 Kalibr。

## 8. 验证结论与常见问题

一键入口依次检查：完整 CSV → bag 与选段逐条一致 → 上游读入数一致 → 9,999×7 曲线形状、正值及有限值 → 五个尺度的六轴独立复算 → YAML 正值、topic/rate 和图片存在。

`PASS` 表示这些流程检查通过。当前上游白噪声只拟合 tau<10 秒，随机游走却用全部 tau、强约束约 +1/2 斜率；未自动判断可信拟合段、静止性和恒温条件，也没有执行实际 Kalibr 标定。参数应作为候选审阅。

| 情况 | 处理 |
|---|---|
| 输出目录已存在 | 用新的 --output；不要为重跑覆盖原结果 |
| bag 校验失败 | 核对 CSV、选区、topic、frame ID 是否与 bag 一致 |
| 运行锁被占用 | 等待现有 imu_utils／allan_variance_ros 任务完成 |
| 长时间没有新终端输出 | 查看 work 日志；CSV 检查和逐条回读会持续处理，未必逐条打印 |
| roscore 启动失败 | 查看 work/roscore.log，检查是否有手动启动的残留 master |
| 输入不足以支持固定 tau | 改用足够长的原始选段；入口不会自行扩展数据或缩短网格 |
| 只看到手动运行的 PNG/YAML | 上游手动命令没有中文报告功能；完整流程用包装入口 |
| 黄线存在但实测曲线没有明显上升段 | 强制拟合仍会出数值；这不能证明随机游走参数可信 |

计算错误保留 work 中的日志和中间结果，入口写 status=fail；不要把失败或未结束目录当作最终结果。

## 9. 已完成的真实测试

| 数据集／区间 | 工具 | 消息数 | 结果 |
|---|---|---:|---|
| imu_260910_2209 全段 | allan_variance_ros | 1,447,587 | 读取、曲线、五尺度复算及产物检查通过 |
| imu_260911_1502 [1h,6h) | imu_utils 与 allan_variance_ros | 1,809,415 | 两工具使用同一物理 bag；各自检查通过 |
| imu_260911_1502 [1h,23h) | allan_variance_ros | 7,962,378 | 读取、曲线、五尺度复算及产物检查通过 |

1–23 小时测试于 2026-09-15 完成，平均采样率 100.53508534939816 Hz，首末时长 79,199.982496933 秒；之后在源码解读中独立复现了四个 YAML 噪声参数。

- [1–23 小时报告](../../data/allan_variance/imu_utils_output/imu_260911_1502_1h-23h_allan_variance_ros/allan_variance_ros_report.md)
- [1–23 小时 imu.yaml](../../data/allan_variance/imu_utils_output/imu_260911_1502_1h-23h_allan_variance_ros/imu.yaml)
- [1–6 小时 allan_variance_ros 报告](../../data/allan_variance/imu_utils_output/imu_260911_1502_1h-6h_allan_variance_ros/allan_variance_ros_report.md)
- [1–6 小时 imu_utils 报告](../../data/allan_variance/imu_utils_output/imu_260911_1502_1h-6h/imu_utils_report.md)
- [上游固定版本](https://github.com/ori-drs/allan_variance_ros/tree/1d54b602ee7f2ba0427865d63afe4945d913ed24)
