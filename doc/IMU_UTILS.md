# ROS1 imu_utils 容器配置与使用说明

## 目录与容器

项目根为 `Q:\project\kalibr_cedarobo`，Docker Desktop 必须运行。

| 宿主机目录（相对项目根） | 容器路径 | 权限 |
|---|---|---|
| `script/imu_utils_docker/allan` | `/scripts` | 只读分析脚本 |
| `data/allan_variance/data` | `/data` | 只读原始 CSV |
| `data/allan_variance/imu_utils_output` | `/output` | 读写结果 |

这是三个 bind mount，不使用命名卷。源码和编译结果 `/imu_ws` 在镜像内。
基础镜像 `kalibr-ros1-noetic:latest`，派生镜像 `kalibr-imu-utils-noetic:local`，
长期容器 `kalibr-imu-utils-noetic`，配置为 `init: true`、`restart: unless-stopped`。
旧命名卷 `kalibr-imu-utils-catkin` 保留但不再挂载。
固定上游提交为 `f8a770cc81f8c86e93ee3c94518d2456dc722ad3`，安装 Ceres、NumPy、Matplotlib、PyYAML，
仅编译 `imu_utils`；不需要构建 `code_utils` 示例。此目录只用于 PC，不部署到板端。

WSL 中创建／启动容器：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose up -d
docker compose ps
```

需要重新构建镜像时用 `docker compose build`；重新下载、完整编译用
`docker compose build --no-cache`，然后 `docker compose up -d`。
改挂载后用 `docker compose up -d --force-recreate`。Python 脚本修改立即通过挂载生效，无需重编译。


目录按职责分开：

```text
script/imu_utils_docker/
  docker/
    compose.yaml
    Dockerfile
    .dockerignore
    patch_sample_limit.py
  allan/
    run.sh
    run_analysis.py
    input_format.py
    csv_to_rosbag.py
    validate_rosbag.py
    validate_imu_utils_output.py
    generate_report.py
  README.md
  REORGANIZATION_VALIDATION.md
```

`docker/` 只负责容器配置、镜像构建与源码补丁；`allan/` 负责 CSV 转换、分析调度、结果检查和绘图。
仅 `allan/` 挂载为 `/scripts`，所以容器内命令不变。宿主机的 Compose 命令改在 `docker/` 中执行。
Compose 的输入、输出相对路径均已随层级调整，仍指向原来的数据目录。

## 输入：只读取 CSV

只需要 `/data/<输入名>/imu/imu0.csv`，不需要 `meta/session.json` 或 `meta/validation.json`，
也不读取或校验元数据。表头严格为：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az,temperature
```

单位约定为整数纳秒、角速度 rad/s、加速度 m/s²、温度摄氏度；程序不从元数据确认单位，
使用者须保证 CSV 符合此约定。保留必要的 CSV 读取检查：八列、数值有限、时间严格递增，至少两条。
样本数、时长和平均频率直接从 CSV 得到，不改变时间戳或六轴数据。
`temperature` 不参与 Allan 计算，也不生成独立温度消息。

CSV 没有 frame ID，使用 `--frame-id` 指定，默认 `imu_link`；它是派生消息的标签，
不是从 CSV 恢复的原始坐标系。不旋转或变换六轴。无姿态时 `orientation_covariance[0] = -1`。

已移除 `test_workflow.py`，使用流程不要求执行格式错误、篡改等自动测试。
运行结果的两个验证脚本仍保留，作用见下文。

## 一键运行（WSL 宿主机）

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose exec -T imu-utils bash /scripts/run.sh /data/imu_260910_2209
```

也可以直接传入 CSV 路径。原输出目录已存在时拒绝覆盖，重跑选择新目录：

```bash
docker compose exec -T imu-utils bash /scripts/run.sh \
  /data/imu_260910_2209/imu/imu0.csv \
  --output /output/imu_260910_2209_rerun --frame-id imu_link
```

`--topic` 默认 `/imu/data`，`--playback-rate` 默认 100，`--max-cluster` 默认 100，
`--analysis-timeout` 默认 1800 秒（回放结束后等待拟合），`--convert-only` 只转换与回读 bag。
一键流程自动启动和清理 ROS 进程，同一容器内不要与手动分析同时运行。

## 进入 Docker 后运行

在 WSL 宿主机执行：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose up -d
docker compose exec imu-utils bash
```

或在 Windows PowerShell 中直接进入运行中的容器：

```powershell
docker exec -it kalibr-imu-utils-noetic bash
```

此后的命令都在容器内执行，使用 `/data`、`/output`、`/scripts`，不使用 Q 盘或 `/mnt/q` 路径。

在容器内运行完整流程：

```bash
bash /scripts/run.sh /data/imu_260910_2209/imu/imu0.csv \
  --output /output/imu_260910_2209_container
```

这与宿主机的一键运行相同。如果希望分别执行转换、回放和绘图，按下面步骤操作。

## 容器内逐步操作

### 1. 加载 ROS 环境并准备新输出目录

以下步骤在同一个容器终端依次执行。输出名 `_manual` 已存在时，换一个名字。

```bash
source /opt/ros/noetic/setup.bash
source /imu_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=localhost
NAME=imu_260910_2209
CSV=/data/$NAME/imu/imu0.csv
OUT=/output/${NAME}_manual
mkdir "$OUT"
mkdir "$OUT/work"
```

### 2. CSV 转为 bag，再检查转换结果

```bash
python3 /scripts/csv_to_rosbag.py "$CSV" "$OUT/$NAME.bag" \
  --topic /imu/data --frame-id imu_link
python3 /scripts/validate_rosbag.py "$CSV" "$OUT/$NAME.bag" \
  --topic /imu/data --frame-id imu_link
rosbag info "$OUT/$NAME.bag"
```

从刚生成的转换摘要取样本数（这是输出统计文件，不是输入元数据）：

```bash
N=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["messages"])' "$OUT/work/$NAME.conversion.json")
echo "$N"
cp /imu_ws/upstream_revision.txt /imu_ws/local.patch "$OUT/work/"
```

### 3. 启动 ROS master

```bash
export ROS_LOG_DIR="$OUT/work/ros_logs"
roscore > "$OUT/work/roscore.log" 2>&1 &
MASTER_PID=$!
```

确认下面命令成功后继续；启动过程中失败可稍后重试：

```bash
rosparam list
```

### 4. 启动分析节点，然后回放 bag

```bash
rosrun imu_utils imu_an __name:=imu_an \
  _imu_topic:=/imu/data _imu_name:="$NAME" \
  _data_save_path:="$OUT/work/" \
  _max_samples:="$N" _max_time_min:=240 _max_cluster:=100 \
  > "$OUT/work/imu_utils.log" 2>&1 &
ANALYZER_PID=$!

rosbag play --quiet --wait-for-subscribers --delay=2 \
  --queue=1000000 --rate=100 "$OUT/$NAME.bag" \
  > "$OUT/work/rosbag_play.log" 2>&1

wait "$ANALYZER_PID"
```

播放器等待分析节点订阅后才发送。回放完成后，`wait` 等待 Allan 计算和拟合结束。
`max_samples` 启用时，`max_time_min` 不参与截止判断；240 只是上游必填参数的占位值。
若长时间没有结束，可另开容器终端，用 `tail -f /output/imu_260910_2209_manual/work/imu_utils.log`
查看日志。手动流程发生错误时先排查，不要继续生成报告；一键流程会自动阻止后续步骤。

### 5. 检查分析输出，生成图和报告

```bash
python3 /scripts/validate_imu_utils_output.py "$OUT/work" \
  --name "$NAME" --expected-messages "$N"
python3 /scripts/generate_report.py "$OUT/work" --name "$NAME"
ls -lh "$OUT"
```

手动流程不生成一键调度器的 `run.json`；参数以实际执行命令和日志为准。
停止自己启动的 ROS master 并退出终端：

```bash
kill -INT "$MASTER_PID"
wait "$MASTER_PID"
exit
```

退出终端不会删除或停止长期容器。在宿主机执行 `docker compose stop` 才停止容器，
`docker compose start` 再启动。`docker compose down` 删除容器和网络，不删除宿主机输入与输出。

## 两个验证脚本分别做什么

| 脚本 | 检查阶段 | 作用 |
|---|---|---|
| `validate_rosbag.py` | CSV → bag 之后 | 逐条确认话题、消息类型、数量、时间戳、frame ID、六轴值和未知姿态标记一致，发现转换错误或 bag 数据不完整 |
| `validate_imu_utils_output.py` | bag 回放 → imu_utils 之后 | 确认六轴 `numData` 全部等于 CSV/bag 消息数；检查 tau、实测／拟合曲线及 YAML 参数有效，发现回放丢消息或无效计算结果 |

第一个证明“bag 写对了”，第二个证明“分析实际收齐了消息，输出数值有效”。
即使 bag 正确，ROS 回放仍可能丢消息，所以两者作用不同。它们不是元数据校验，也不是
`test_workflow.py` 的测试用例。当前一键流程仍自动执行这两个结果检查，报告依赖通过标记。

## patch_sample_limit.py 的作用

这是构建镜像时执行的源码补丁，不需要每次分析时手动运行。
上游按传感器时间跨度超过 `max_time_min` 结束接收；不足阈值的短 bag 播完后可能一直等待，
阈值小于数据时长又可能提前结束。
补丁增加 `max_samples`：收到 CSV 实际样本数 N 后开始计算，后续消息不再加入。
它不改变 Allan 或 Ceres 拟合公式，不插值、不补消息。
若传输丢失导致收不到 N 条，一键流程最终超时报错，不能把少量数据当成完整结果。

## 输出与参数解释

输出根只放 bag、最终参数 YAML、三张 PNG 和 Markdown 报告；转换摘要、验证记录、运行状态、ROS 日志和源码补丁统一放在 `work/`。
`work/` 包含 16 个 TXT 及所有中间记录。上游先将 YAML 写入 work，验证通过后自动移至顶层；绘图脚本读取 work 数据并将图和报告写入顶层。
三张图分别为组合图、陀螺仪图和加速度计图；空心点为实测 Allan 标准差，实线为五项模型拟合。
陀螺仪曲线为上游内部单位 deg/h，加速度计为 m/s²。
`gyr_w/acc_w` 为 bias instability，不能直接作为 Kalibr 的 bias random walk。
平均采样周期近似及温漂可能影响物理解释，验证通过不表示标定或噪声模型验收通过。

失败保留日志；一键分析失败详情保留在 `work/run.json` 与 `work/` 日志中。重试使用新输出目录。
历史数据和报告不重写；旧脚本在 `legacy_20260911/`，只作归档。
[迁移验证记录](../script/imu_utils_docker/REORGANIZATION_VALIDATION.md)描述迁移时版本；
本次简化已删除当时使用的 test_workflow.py，输入也不再依赖元数据。

当前每次运行的输出结构：

```text
imu_260910_2209/
  imu_260910_2209.bag
  imu_260910_2209_imu_param.yaml
  imu_utils_accel_allan.png
  imu_utils_allan_deviation.png
  imu_utils_gyro_allan.png
  imu_utils_report.md
  work/
    data_imu_260910_2209_*.txt
    imu_260910_2209.conversion.json
    imu_260910_2209.validation.json
    validation.json
    run.json
    imu_utils.log
    roscore.log
    rosbag_play.log
    ros_logs/
    upstream_revision.txt
    local.patch
```

验证和绘图的结果目录参数指向 `work/`：

```bash
python3 /scripts/validate_imu_utils_output.py /output/imu_260910_2209/work --name imu_260910_2209 --expected-messages 1447587
python3 /scripts/generate_report.py /output/imu_260910_2209/work --name imu_260910_2209
```

仅转换模式顶层只有 bag。转换中的 `.bag.inprogress` 位于 work，完成后移至顶层。
手动模式同样使用此布局；JSON 文件需保留，报告会读取转换摘要与验证结果。

同一容器已加入 allan_variance_ros，直接离线读取 bag。安装、输入与输出说明见 [allan_variance_ros 使用说明](ALLAN_VARIANCE_ROS.md)。

## 指定分析／bag 导出时间范围

CSV 转 bag、bag 回读验证和两个分析入口均支持：

| 参数 | 含义 |
|---|---|
| `--start-h` / `--end-h` | 相对原始首条 IMU 的小时数 |
| `--start-s` / `--end-s` | 相对原始首条 IMU 的秒数 |

起点包含、终点不包含，例如 `--start-h 1 --end-h 6` 选择 `[3600,21600)` 秒，共 5 小时。
这是相对源 CSV 第一条传感器时间戳的偏移，不是墙钟时间，也不是文件名中的录制时间。
同一边界不能同时指定秒和小时；不传起点默认 0，不传终点一直到文件末尾。
支持小数，范围判定使用整数纳秒，精度不因目录名显示而降低。

先完整检查源 CSV，再只导出和计算所选区间。原始 CSV 不修改，bag 的消息时间戳不归零，
六轴和区间内样本保持不变，bag seq 从 0 编排。消息数、采样率与时长统计针对所选区间；
work 中的 JSON 同时记录区间、原始首时间戳与源文件总数。

两个分析入口的默认目录会追加小时区间标记（显示保留至一位小数）；同名已有目录仍拒绝覆盖。
不指定时间保持原有全段行为。`--output` 可以指定完整新目录，但 bag／参数名称仍保留区间标记。
独立 CSV 转换脚本的 bag 名称由传入的完整路径决定。

### 本次 imu_260911_1502 第 1–6 小时

WSL 宿主机执行：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker

docker compose exec -T imu-utils bash /scripts/run.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 6

docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 6 \
  --bag /output/imu_260911_1502_1h-6h/imu_260911_1502_1h-6h.bag
```

结果分别在 `/output/imu_260911_1502_1h-6h` 和
`/output/imu_260911_1502_1h-6h_allan_variance_ros`。
复用 bag 时必须与所选区间一致，否则逐条验证会失败；不传 `--bag` 会自动转换相同区间。
上述结果已存在时，重跑请为两个命令分别指定新的 `--output`。

只转换、检查区间 bag（容器内）：

```bash
source /opt/ros/noetic/setup.bash
python3 /scripts/csv_to_rosbag.py /data/imu_260911_1502/imu/imu0.csv \
  /output/imu_260911_1502_bag_1h-6h/imu_260911_1502_1h-6h.bag --start-h 1 --end-h 6
python3 /scripts/validate_rosbag.py /data/imu_260911_1502/imu/imu0.csv \
  /output/imu_260911_1502_bag_1h-6h/imu_260911_1502_1h-6h.bag --start-h 1 --end-h 6
```

`--start-s 3600 --end-s 21600` 与本例的小时参数完全等价。

双工具对照只转换一次区间 bag，allan_variance_ros 的 --bag 直接引用 imu_utils 已生成的同一个文件，不复制、不重复转换；其顶层 bag 为相对符号链接，原文件需保留。
