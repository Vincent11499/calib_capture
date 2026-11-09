# 纯 Python 复现 allan_variance_ros

## 1. 目的

`script/imu/allan/analyze_allan_variance.py` 直接读取 `imu-continuous` 生成的八列 CSV，取代原版
`allan_variance_ros` 的 ROS1 bag 读取和 C++ Allan 标准差计算，同时复刻其 Python 拟合、图片和
Kalibr `imu.yaml`。兼容目标固定为：

```text
ori-drs/allan_variance_ros@1d54b602ee7f2ba0427865d63afe4945d913ed24
```

新流程不需要 ROS、bag、roscore、C++、catkin、Docker、PyYAML 或 AllanTools。只需要 PC
环境中的 NumPy、SciPy 和 Matplotlib；不会安装或同步到板端。

## 2. 输入与运行

输入必须是独立 IMU 会话目录，或其中的 `imu/imu0.csv`。表头严格为：

```csv
timestamp_ns,wx,wy,wz,ax,ay,az,temperature
```

在项目根目录运行全段：

```powershell
conda activate calib
cd Q:\project\kalibr_cedarobo
python script/imu/allan/analyze_allan_variance.py `
  data/allan_variance/data/imu_260911_1502
```

分析第 1–23 小时：

```powershell
python script/imu/allan/analyze_allan_variance.py `
  data/allan_variance/data/imu_260911_1502 `
  --start-h 1 --end-h 23
```

秒级选区同样可用：

```powershell
python script/imu/allan/analyze_allan_variance.py DATASET `
  --start-s 3600 --end-s 82800
```

选区相对源 CSV 第一条时间戳，采用 `[start,end)`。完整源文件都会先校验；选区外存在错误也会
失败。已有输出、`.inprogress` 或 `.failed` 目录都不会覆盖。

## 3. 默认输出

默认根目录固定为：

```text
Q:\project\kalibr_cedarobo\data\allan_variance\allan_variance_output\
```

全段结果示例：

```text
allan_variance_output/imu_260911_1502_allan_variance/
  imu.yaml
  acceleration.png
  gyro.png
  allan_variance_report.md
  work/
    allan_variance.csv
    config.json
    fit.json
    run.json
    validation.json
```

时间选区会加入 `_1h-23h` 等后缀。可用 `--output DIR` 指定完整结果目录。

## 4. 与原版保持一致的行为

- 实际采样率为 `(N-1)/(t_last-t_first)`，`measure_rate=floor(实际采样率)`；不重采样。
- tau 固定为 0.1–999.9 s，共 9,999 点；`m=int(tau*measure_rate)`。
- 非重叠分箱，箱数固定为 `floor((N-1)/m)`；当样本数恰好整除 `m` 时仍舍弃最后完整箱。
- 加速度保持 m/s²，陀螺仪先由 rad/s 转为 deg/s，再计算相邻箱均值差的 Allan deviation。
- `work/allan_variance.csv` 保留原 C++ 的 19/7 位有效数字及列顺序。
- 白噪声使用 tau 0.1–9.9 s、斜率约束 `[-0.5,-0.499]`，在 1 s 取值。
- 随机游走使用全部 9,999 点、斜率约束 `[0.5,0.501]`，在 3 s 取值。
- 每类参数取三轴最大值；陀螺仪写 YAML 前转回弧度制。

Python 前缀和与 C++ 逐项累加的浮点求和顺序不同，所以不能要求曲线文本逐字节相同。历史
4 小时数据回归中，9,999×6 点全部通过原项目 `rtol=2e-5, atol=1e-10` 判据；最大相对差约
`8.51e-7`，四项 YAML 参数仅有浮点末位差异。

## 5. 结果边界

`PASS` 表示完整输入检查、固定曲线结构、五个尺度独立复算和产物检查通过。原算法会强制在
全部尺度上拟合 `+1/2` 斜率，并不自动判断数据是否真的存在可信上升段。因此 `imu.yaml`
是“原版算法兼容候选”，不是 IMU 模型或 Kalibr 标定质量已经通过。

旧 `script/imu_utils_docker/allan` 和 `data/allan_variance/imu_utils_output` 暂时保留，只用于
历史回归与 `imu_utils` 交叉分析；新纯 Python 流程不会创建 bag。
