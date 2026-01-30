### 1) 启动 AprilTag + TF 发布（已有 launch）

在已经能发布 `fixed_frame` 与 `moving_frame` 的 TF 前提下，直接运行：

```bash
# 在工作区根目录执行（默认会同时打开 RViz）
roslaunch d435i_apriltag_csv fixed_and_moving_tag_relative_csv.launch
```

如果不需要 RViz，可以关闭：

```bash
roslaunch d435i_apriltag_csv fixed_and_moving_tag_relative_csv.launch use_rviz:=false
```

### 2) 输出 CSV 文件

- 默认输出路径：`<workspace>/output/tag_relative.csv`（与 `src/` 同级，启动时会覆盖写入）
- 表头字段：`stamp_sec, fixed_frame, moving_frame, x_m, y_m, z_m, range_m`

### 3) 常用参数（可覆盖）

该脚本读取以下 ROS 参数（可在 launch 中覆盖）：

- `fixed_frame`：固定坐标系（默认 `tag_fixed`）
- `moving_frame`：被测坐标系（默认 `tag_obj`）
- `csv_path` / `output_csv`：CSV 输出路径（默认 `<workspace>/output/tag_relative.csv`）
- `rate_hz`：写入频率（默认 30Hz）
- `use_rviz`：是否启动 RViz（默认 true）
- `rviz_config`：RViz 配置文件路径（默认 `d435i_apriltag_csv/rviz/apriltag_view.rviz`）
- `rviz_wait_topic`：等待的图像话题（默认 `/camera/color/image_rect_color`）
- `rviz_wait_timeout`：等待话题超时秒数（默认 10.0，超时仍会启动 RViz）
- `rviz_initial_delay`：启动前额外延迟（秒，默认 0.0）
- `enable_static_ref`：启动后标定一次并发布固定参考帧（默认 true）
- `fixed_frame_ref`：标定后的参考帧名称（默认 `tag_0_ref`）
- `camera_frame`：相机光学坐标系（默认 `camera_color_optical_frame`）
- `calib_duration`：标定采样时长（秒，默认 2.0）
- `calib_samples`：标定最大采样帧数（默认 60）
- `calib_timeout`：单次 TF 等待超时（秒，默认 0.1）
- `median_window`：中值滤波窗口大小（默认 7，设为 1 关闭）
- `alpha`：指数平滑系数（默认 0.1，越小越稳但越慢）
- `write_raw`：是否在 CSV 里追加原始未滤波数据（默认 false）

示例（覆盖参数）：

```bash
roslaunch d435i_apriltag_csv fixed_and_moving_tag_relative_csv.launch \
	csv_path:=$(pwd)/output/tag_relative.csv \
	fixed_frame:=tag_1 \
	moving_frame:=tag_2 \
	rate_hz:=15
```

### 3)如果只想开相机：
```bash
roslaunch realsense2_camera rs_camera.launch   enable_color:=true enable_depth:=true   align_depth:=true   color_width:=1280 color_height:=720 color_fps:=30
```