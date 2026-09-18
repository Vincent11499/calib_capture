# imu_utils Docker 与 Allan 分析

- `docker/`：Compose、Dockerfile、构建上下文规则和 `patch_sample_limit.py`。
- `allan/`：CSV 转换、Allan 分析调度、结果验证和图像报告脚本。
- 输入：项目 `data/allan_variance/data/`，只读挂载 `/data`。
- 输出：项目 `data/allan_variance/imu_utils_output/`，读写挂载 `/output`。
- `allan/` 只读挂载到 `/scripts`；Docker 配置不挂载。

WSL 宿主机启动与运行：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose up -d
docker compose exec -T imu-utils bash /scripts/run.sh /data/imu_260910_2209/imu/imu0.csv --output /output/imu_260910_2209_rerun
```

进入容器后运行：

```bash
docker compose exec imu-utils bash
bash /scripts/run.sh /data/imu_260910_2209/imu/imu0.csv --output /output/imu_260910_2209_container
```

输出目录必须尚不存在。只读取八列 CSV，不需要元数据文件。
完整的镜像构建、容器内逐步操作与各脚本解释见[使用说明](../../doc/docker/IMU_UTILS.md)。


imu_utils 每次运行输出顶层只放 bag、最终参数 YAML、三张 PNG 和 Markdown 报告；中间曲线 TXT、转换摘要、日志、验证、运行记录和源码补丁全部放 work/。验证与绘图命令的结果目录参数指向 work/。

同一容器已加入 allan_variance_ros，直接离线读取 bag。安装、输入与输出说明见 [allan_variance_ros 使用说明](../../doc/docker/ALLAN_VARIANCE_ROS.md)。

两个分析入口及 CSV 转 bag 支持 --start-h 1 --end-h 6（或 --start-s 3600 --end-s 21600），按原始首条消息起算选择 [1h,6h)，保留原时间戳。默认结果目录追加 _1h-6h；详见使用文档的时间范围章节。

## allan_variance_ros：复用 1–23 小时 bag

以下在 WSL 宿主机执行，结果目录必须尚不存在：

```bash
cd /mnt/q/project/kalibr_cedarobo/script/imu_utils_docker/docker
docker compose exec -T imu-utils bash /scripts/run_allan_variance_ros.sh \
  /data/imu_260911_1502/imu/imu0.csv --start-h 1 --end-h 23 \
  --bag /output/imu_260911_1502_1h-23h_allan_variance_ros/imu_260911_1502_1h-23h.bag \
  --output /output/imu_260911_1502_1h-23h_allan_variance_ros_rerun
```

只引用原 bag，不复制或再次转换。操作步骤、手动容器命令、结果查看与排错见 [使用说明](../../doc/docker/ALLAN_VARIANCE_ROS.md)；分箱、拟合及 Kalibr YAML 单位见 [源码解读](../../doc/ALLAN_VARIANCE_ROS_SOURCE.md)。
