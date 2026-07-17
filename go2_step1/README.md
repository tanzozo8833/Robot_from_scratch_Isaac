# STEP 1 — Build: Go2 chạy được trong Isaac Sim + ROS 2 + SLAM + Nav2

Bộ file triển khai cho `step1-go2-ros2-nav2-pipeline.md`.

| File | Vai trò |
|------|---------|
| `go2_ros2.py` | Script chính: nạp scene, chạy policy, gắn LiDAR+camera, publish ROS 2 |
| `config/slam_go2.yaml` | Cấu hình slam_toolbox cho Go2 |
| `config/nav2_go2.yaml` | Cấu hình Nav2 cho Go2 (holonomic, trần tốc độ khớp policy) |

---

## ⚠️ ĐỌC TRƯỚC: trạng thái của code này

Tôi **không chạy được Isaac Sim từ máy này**, nên code được viết dựa trên tài liệu NVIDIA 6.0 chứ **chưa được test trên VM của bạn**. Các phần **nhiều khả năng phải chỉnh nhỏ** khi chạy lần đầu:

| Chỗ | Rủi ro | Cách xử lý |
|-----|--------|-----------|
| Import `PolicyController` | Đường dẫn module có thể khác ở 6.0.1 | Chạy lệnh dò ở mục 1 dưới |
| Tên thuộc tính `_decimation`, `_action_scale`, `_previous_action` | Có thể tên khác | `print(dir(go2))` để xem |
| Tên node OmniGraph (`isaacsim.ros2.bridge.*`) | Có thể lệch tên attribute | Xem log lỗi, đối chiếu Isaac docs |
| `config="Example_Rotary"` của RTX LiDAR | Tên profile có thể khác | Dò bằng lệnh mục 1 |
| TF `base_link -> sensors` | Node `PublishTF` cần chỉnh targetPrims | Kiểm bằng `view_frames` |

→ **Đây là bộ khung đúng kiến trúc.** Chạy → gặp lỗi → gửi log cho tôi, tôi sửa từng cái.

---

## 1. Dò thông tin trên VM (chạy trước, gửi output cho tôi)

```bash
# a) Class Go2 / PolicyController có sẵn không
find ~/isaacsim -path "*robot.policy*" -name "*.py" 2>/dev/null | head -20
grep -rl "class PolicyController" ~/isaacsim/exts* 2>/dev/null | head

# b) Policy pre-trained của NVIDIA (.pt + env.yaml — phải cùng cặp)
find ~/isaacsim -iname "*.pt"     -path "*olicy*" 2>/dev/null | head
find ~/isaacsim -iname "env.yaml" -path "*olicy*" 2>/dev/null | head

# c) Profile RTX LiDAR có sẵn
find ~/isaacsim -iname "*.json" -path "*lidar*" 2>/dev/null | head -20

# d) Scene + prim path Go2
find ~ -name "build_from_scratch.usd" 2>/dev/null
```

---

## 2. Copy sang VM

Từ máy Windows này:
```powershell
scp -r go2_step1/* azureuser@20.55.80.91:~/go2_deploy/
```

---

## 3. Điền CONFIG trong `go2_ros2.py`

Sửa block `1. CONFIG` ở đầu file:

```python
USD_SCENE_PATH   = "..."   # từ lệnh 1d
GO2_PRIM_PATH    = None    # để None -> tự dò ArticulationRoot
POLICY_PT_PATH   = "..."   # từ lệnh 1b
POLICY_ENV_YAML  = "..."   # từ lệnh 1b — PHẢI cùng cặp với .pt
HEADLESS         = False   # True nếu không có DCV
```

> **Nhắc lại:** `.pt` và `env.yaml` phải từ **cùng một lần train**. `env.yaml` chứa observation scale / action_scale / decimation / default joint pos — `PolicyController` đọc từ đây nên bạn không hard-code gì.

---

## 4. Chạy — thứ tự bring-up (mỗi lệnh 1 terminal)

```bash
# T1 — Isaac Sim + policy + ROS2 bridge
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh ~/go2_deploy/go2_ros2.py
# Bấm Play trong UI nếu chạy có display

# T2 — Kiểm tra topic đã ra chưa (xem mục 5)
ros2 topic list

# T3 — Lái tay TRƯỚC, xác nhận Go2 bước được rồi mới bật SLAM/Nav2
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# T4 — SLAM
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  slam_params_file:=/home/azureuser/go2_deploy/config/slam_go2.yaml

# T5 — Nav2
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/home/azureuser/go2_deploy/config/nav2_go2.yaml

# T6 — RViz
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=True
```

---

## 5. Nghiệm thu từng bước (làm ĐÚNG THỨ TỰ này)

Đừng bật hết một lúc. Mỗi bước xanh mới sang bước sau:

### Bước 1 — Policy sống (quan trọng nhất)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```
✅ Go2 **bước tới, không ngã**.
❌ Nếu ngã ngay → xem `step1a-train-go2-policy.md` mục 5. Kiểm tra log in ra `joint_names` và `default_pos` khi khởi động.

### Bước 2 — Topic đầy đủ
```bash
ros2 topic list
```
✅ Phải thấy: `/clock` `/cmd_vel` `/odom` `/tf` `/tf_static` `/scan` `/points` `/go2/front_cam/rgb` `/go2/front_cam/depth` `/go2/front_cam/camera_info` `/joint_states`

### Bước 3 — LiDAR có dữ liệu
```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```
✅ `ranges` có số thực (không phải toàn `inf`).

### Bước 4 — TF liền mạch
```bash
ros2 run tf2_tools view_frames
```
✅ Chuỗi `odom → base_link → lidar_link / front_cam` (chưa có `map` vì SLAM chưa bật).

### Bước 5 — SLAM dựng map
Bật T4, lái Go2 đi vòng bằng T3.
✅ RViz thấy `/map` lớn dần. `view_frames` giờ có `map → odom`.

### Bước 6 — Nav2 tự lái
Bật T5, dùng nút **"Nav2 Goal"** trong RViz.
✅ Go2 tự đi tới đích, không loạng choạng.
❌ Nếu loạng choạng → giảm `vx_max` trong `nav2_go2.yaml` xuống `0.5`.

---

## 6. Các lỗi hay gặp

| Lỗi | Nguyên nhân | Sửa |
|-----|-------------|-----|
| `ModuleNotFoundError: rclpy` | Chưa source ROS 2 | `source /opt/ros/jazzy/setup.bash` trước khi chạy |
| Go2 ngã ngay | Ghép nối obs/khớp | `step1a-train-go2-policy.md` mục 5; check log `joint_names` |
| `/scan` toàn `inf` | LiDAR profile sai / gắn sai chỗ | Đổi `config=` trong `create_lidar()` |
| Nav2 báo thiếu TF | `map→odom` chưa có | Bật slam_toolbox trước Nav2 |
| Topic không ra | ROS 2 bridge chưa bật | Kiểm log `enable_extension("isaacsim.ros2.bridge")` |
| Go2 rung/giật khi Nav2 lái | Tốc độ vượt dải train | Giảm `vx_max`, `wz_max` trong `nav2_go2.yaml` |
| Không thấy cửa sổ Isaac | Headless / không có display | Bật DCV, hoặc `HEADLESS=True` + xem qua RViz |

---

## 7. Ghi chú thiết kế

- **`/cmd_vel` được clamp** trong `CmdVelListener` (`MAX_VX=0.8`) — chốt chặn an toàn để lệnh lạ không làm Go2 ngã. `velocity_smoother` của Nav2 là lớp chặn thứ hai.
- **rclpy chạy trong tiến trình Isaac** — được vì Isaac Sim 6.0.1 và ROS 2 Jazzy cùng Python 3.12.
- **Chia việc:** OmniGraph lo publish cảm biến (cần render pipeline); rclpy chỉ lo subscribe `/cmd_vel` (đơn giản, dễ debug).
- **Camera RGB-D đã gắn sẵn** dù Step 1 chưa dùng — để Step 2 (nhận diện người/hộp) cắm vào là chạy.

---

## 8. Tham chiếu

- `step1-go2-ros2-nav2-pipeline.md` — kiến trúc Step 1
- `step1a-train-go2-policy.md` — train & debug policy (đọc khi Go2 ngã)
- `step2-go2-perception-tasks.md` — Step 2
