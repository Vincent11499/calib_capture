# 纯 Python allan_variance_ros 兼容实现

该目录直接读取 `imu-continuous` 的八列 `imu/imu0.csv`，用 NumPy 取代原 C++ 的 bag
读取与 Allan 标准差计算，再用 SciPy/Matplotlib 复刻固定提交
`ori-drs/allan_variance_ros@1d54b602ee7f2ba0427865d63afe4945d913ed24` 的拟合和图片。

不需要 ROS、rosbag、C++、catkin、Docker、PyYAML 或 AllanTools。输入只读，默认输出到：

```text
data/allan_variance/allan_variance_output/<会话名>_allan_variance/
```

在项目根目录运行：

```powershell
conda activate calib
python script/imu/allan/analyze_allan_variance.py `
  data/allan_variance/data/imu_260911_1502
```

选择第 1–23 小时：

```powershell
python script/imu/allan/analyze_allan_variance.py `
  data/allan_variance/data/imu_260911_1502 `
  --start-h 1 --end-h 23
```

需要 NumPy、SciPy 和 Matplotlib。输出 `imu.yaml`、`acceleration.png`、`gyro.png`、分析报告，
以及 `work/allan_variance.csv` 和 JSON 验证记录。已有输出不会覆盖。

兼容是指数值算法、边界、单位和拟合策略兼容；Python/NumPy 的浮点求和顺序与 C++ 逐项
累加不完全相同，因此曲线以原 C++ 七位有效数字输出精度为回归判据。
