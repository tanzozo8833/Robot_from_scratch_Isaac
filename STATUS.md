# STATUS — Nhật ký tiến độ Go2 / Isaac Sim / ROS 2

> **Quy tắc của file này:**
> - Chỉ ghi **sự thật đã kiểm chứng** bằng output lệnh thật hoặc code đã đọc.
> - **Không đoán, không bịa.** Thứ chưa kiểm chứng phải nằm ở mục "❓ CHƯA RÕ — CẦN KIỂM TRA".
> - Mỗi lần cập nhật: ghi **ngày giờ**, đã làm gì, kiểm tra gì, **output ra sao**, từ đó **dùng được gì**.

---

## 📌 TỔNG QUAN TRẠNG THÁI HIỆN TẠI

| Hạng mục | Trạng thái | Ghi chú |
|----------|-----------|---------|
| Isaac Sim 6.0.1 + ROS 2 Jazzy + Isaac Lab | ✅ Chạy được | Đã xác minh từ trước |
| Train policy Go2 (flat, 300 vòng) | ✅ Xong | `go2_flat_policy.pt` |
| **Go2 bước đi được trong Isaac Sim** | ✅ **ĐÃ CHẠY** | `go2_teleop.py` — WASD, không ngã |
| Policy NVIDIA (`USE_MY_POLICY=False`) | ✅ Chạy, không ngã | Xác nhận bởi user |
| Policy tự train (`USE_MY_POLICY=True`) | ✅ Chạy, không ngã | Xác nhận bởi user |
| Gắn LiDAR + camera | ❌ Chưa làm | |
| Publish ROS 2 topics | ❌ Chưa làm | |
| SLAM (slam_toolbox) | ❌ Chưa chạy | Config đã viết, chưa test |
| Nav2 | ❌ Chưa chạy | Config đã viết, chưa test |

---

## 🗓️ 2026-07-17 09:28 — Cập nhật #1: Xác minh nền tảng, phát hiện 2 sai lầm của tôi

### A. Đã kiểm tra gì và output ra sao

#### A1. Tìm extension policy examples
**Lệnh:**
```bash
find ~/isaacsim/exts/isaacsim.robot.policy.examples -name "*.py" | sort
```
**Output (đã lược):**
```
.../examples/controllers/config_loader.py
.../examples/controllers/policy_controller.py
.../examples/robots/go2.py          <-- QUAN TRỌNG
.../examples/robots/anymal.py, franka.py, h1.py, spot.py
.../examples/interactive/go2/go2_example.py
.../examples/tests/test_go2.py
```
**→ Dùng được gì:** Xác nhận **có sẵn class Go2 của NVIDIA** (`robots/go2.py`) và class nền `policy_controller.py`. Không phải tự viết `_compute_observation`.

#### A2. Tìm file policy `.pt` trên đĩa
**Lệnh:**
```bash
find ~/isaacsim -iname "*.pt" -path "*olicy*" 2>/dev/null | head
```
**Output:**
```
/home/azureuser/isaacsim/exts/isaacsim.pip.newton/pip_prebundle/newton[sim]/
  newton-1.2.1-py3-none-any/newton/examples/assets/anymal_walking_policy.pt
```
**→ Kết luận (quan trọng):** **KHÔNG có file policy Go2 nào trên đĩa.**

#### A3. Tìm `env.yaml`
**Lệnh:**
```bash
find ~/isaacsim -iname "env.yaml" -path "*olicy*" 2>/dev/null | head
```
**Output:** *(rỗng — không có kết quả)*

**→ Kết luận:** Policy Go2 của NVIDIA **không nằm local**, mà được **tải từ Nucleus asset server lúc runtime** qua `get_assets_root_path()`. Bằng chứng bổ sung: trong `go2_teleop.py`, gọi `Go2FlatTerrainPolicy(prim_path=..., position=...)` **không truyền policy_path** mà vẫn chạy được → class tự resolve asset.

#### A4. Tìm scene
**Lệnh:** `find ~ -name "build_from_scratch.usd" 2>/dev/null`
**Output:** `/home/azureuser/tan/build_from_scratch.usd`

### B. Sự thật xác lập từ `go2_teleop.py` (script ĐÃ CHẠY ĐƯỢC của user)

Đây là code thật, đã chạy không ngã → mọi API dưới đây là **đã kiểm chứng**, không phải suy đoán.

| Sự thật | Bằng chứng (dòng trong `go2_teleop.py`) |
|---------|------------------------------------------|
| Class Go2 tồn tại | `from isaacsim.robot.policy.examples.robots import Go2FlatTerrainPolicy` |
| Dùng policy NVIDIA | `Go2FlatTerrainPolicy(prim_path="/World/Go2", position=SPAWN_POS)` — **không truyền policy** |
| Dùng policy tự train | thêm `usd_path=`, `policy_path=`, `env_config_path=` |
| Khởi tạo | `go2.initialize()` (gọi ở physics step đầu) |
| Chạy policy | `go2.forward(step_size, base_command)` |
| Command là torch tensor 3 phần tử | `base_command = torch.zeros(3, device=...)` → `[vx, vy, wz]` |
| Ghi khớp trực tiếp | `go2.robot.set_dof_position_targets(positions=wp.from_torch(go2.default_pos))` — dùng **warp** |
| `default_pos` là torch tensor | `go2.default_pos.cpu().numpy()` |
| Tên khớp | `self.robot.dof_names` |
| Layout obs xác nhận | `obs[0:3]` lin_vel, `[3:6]` ang_vel, `[6:9]` gravity, `[9:12]` command, `[12:24]` jointpos−default |
| **Gravity KHÔNG cần đảo dấu** | `FLIP_GRAVITY = False` mà vẫn chạy tốt |
| API Isaac Sim 6.0 (mới) | `SimulationManager`, `RenderingManager`, `isaacsim.core.experimental.utils.stage.define_prim` |
| Vòng lặp | `SimulationManager.register_callback(on_physics_step, IsaacEvents.POST_PHYSICS_STEP)` |
| Physics 200 Hz | `SimulationManager.set_physics_dt(1.0 / 200.0)` |
| Rendering | `RenderingManager.set_dt(8.0 / 200.0)` |
| Device mặc định | `--device` default = **cpu** |
| Scene teleop dùng | `ENV_USD = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"` **tham chiếu vào `/World/Ground`** — KHÔNG mở `build_from_scratch.usd` |
| Cần settle | 100 bước giữ `default_pos` trước khi cho policy chạy |

### C. ⚠️ HAI SAI LẦM CỦA TÔI — đã phát hiện, chưa sửa

#### Sai lầm #1: Kết luận sai về nguyên nhân Go2 ngã
- **Tôi đã nói:** policy tự train ngã ngay ⇒ nghi policy dở (300 vòng quá ít) hoặc ghép nối sai.
- **Sự thật (user xác nhận):** với `go2_teleop.py`, **CẢ HAI** `USE_MY_POLICY=False` và `=True` đều chạy **không ngã**.
- **→ Kết luận đúng:** **Policy 300 vòng của user KHÔNG dở.** Nó chạy tốt khi được nạp qua `Go2FlatTerrainPolicy`. Lần ngã trước đó là do **code import tự viết**, không phải do policy.
- **Hệ quả:** phần "policy dở → train lại 1500 vòng" trong `step1a-train-go2-policy.md` **không áp dụng cho trường hợp này**. Cần sửa lại file đó.

#### Sai lầm #2: `go2_step1/go2_ros2.py` dùng SAI API
Tôi viết script dựa trên tài liệu, dùng API **cũ hơn** so với script đang chạy của user:

| Thứ | `go2_ros2.py` của tôi (SAI) | `go2_teleop.py` (ĐÚNG, đã chạy) |
|-----|------------------------------|----------------------------------|
| Quản lý world | `from isaacsim.core.api import World` | `SimulationManager` + `RenderingManager` |
| Stage | `open_stage(...)` | `define_prim` + `AddReference` |
| Class policy | Tự viết `Go2FlatPolicy(PolicyController)` + tự viết `_compute_observation` | Dùng thẳng `Go2FlatTerrainPolicy` có sẵn |
| Ghi khớp | `set_joint_position_targets(numpy)` | `set_dof_position_targets(wp.from_torch(...))` |
| Vòng lặp | `world.step(render=True)` | callback `POST_PHYSICS_STEP` |
| Kiểu dữ liệu | numpy | torch + warp |

**→ Quyết định:** **Vứt bỏ phần policy trong `go2_ros2.py`**, viết lại trên nền `go2_teleop.py` (đã được chứng minh chạy). Chỉ giữ lại phần OmniGraph ROS 2 (sau khi kiểm chứng tên node).

---

## 📁 TRẠNG THÁI CÁC FILE

| File | Trạng thái | Ý nghĩa / Tại sao |
|------|-----------|-------------------|
| `go2_teleop.py` | ✅ **CHẠY ĐƯỢC — dùng làm nền** | Script gốc của user. Chứng minh policy + Go2 hoạt động. Là **cơ sở đáng tin duy nhất** hiện có |
| `go2_step1/go2_ros2.py` | ⚠️ **SAI API — phải viết lại** | Dựa trên tài liệu, chưa test. Phần policy trùng lặp `Go2FlatTerrainPolicy`. Phần OmniGraph chưa kiểm chứng tên node |
| `go2_step1/config/nav2_go2.yaml` | 🟡 Viết xong, **chưa test** | YAML parse OK. Giá trị (MPPI, Omni, vx_max 0.8, footprint 0.6×0.35) dựa trên tài liệu Nav2 — cần test thật |
| `go2_step1/config/slam_go2.yaml` | 🟡 Viết xong, **chưa test** | YAML parse OK. Cần `/scan` thật mới verify được |
| `go2_step1/README.md` | 🟡 Cần cập nhật | Phần CONFIG `POLICY_PT_PATH` sai hướng (policy NVIDIA không ở local) |
| `step1-go2-ros2-nav2-pipeline.md` | 🟡 Cần sửa nhỏ | Mục 4.5 nói "find policy .pt trên máy" — thực tế không có, policy tải từ asset server |
| `step1a-train-go2-policy.md` | 🟡 Cần sửa | Kết luận "policy có thể dở" đã bị bác bỏ (xem Sai lầm #1) |
| `step2-go2-perception-tasks.md` | ⬜ Chưa đụng | Step 2, chưa tới |

---

## ❓ CHƯA RÕ — CẦN KIỂM TRA NGAY (không được đoán)

| # | Câu hỏi | Cách kiểm tra | Vì sao cần |
|---|---------|---------------|-----------|
| 1 | `Go2FlatTerrainPolicy` nhận tham số gì? Policy NVIDIA tải từ URL nào? Có `decimation`/`action_scale` không? | `cat .../robots/go2.py` | Để ghép ROS 2 đúng cách, biết có cần tự tính obs không |
| 2 | `PolicyController` (class nền) có API gì? | `cat .../controllers/policy_controller.py` | Biết `forward()`, `initialize()` làm gì bên trong |
| 3 | `config_loader.py` đọc env.yaml ra sao? | `cat .../controllers/config_loader.py` | Hiểu obs scale được áp thế nào |
| 4 | **Tên node OmniGraph ROS 2 trong 6.0.1** | `find ~/isaacsim -path "*ros2.bridge*" -name "*.ogn"` | Tôi đang **đoán** tên node — phải xác minh |
| 5 | `rclpy` import được trong `~/isaacsim/python.sh` không? | `~/isaacsim/python.sh -c "import rclpy; print(rclpy.__file__)"` | Quyết định cách nhận `/cmd_vel` |
| 6 | API tạo RTX LiDAR trong 6.0.1 + tên config profile | `find ~/isaacsim -iname "*.json" -path "*lidar*"` | Đang đoán `"Example_Rotary"` |
| 7 | API tạo Camera trong 6.0.1 | `find ~/isaacsim -path "*sensors.camera*" -name "*.py"` | Đang đoán `isaacsim.sensors.camera.Camera` |
| 8 | Prim path Go2 trong `build_from_scratch.usd` | script pxr traverse | **Hoặc bỏ qua**: teleop tự spawn `/World/Go2` từ asset, có thể không cần scene này |
| 9 | Dùng `build_from_scratch.usd` hay tự spawn warehouse như teleop? | Quyết định cùng user | Ảnh hưởng cấu trúc script |

---

## ▶️ BƯỚC TIẾP THEO

1. **Đọc 3 file** (câu hỏi 1–3) → biết chính xác API policy.
2. **Kiểm chứng tên node OmniGraph** (câu hỏi 4) → hết đoán.
3. **Kiểm tra rclpy** (câu hỏi 5) → chốt cách nhận `/cmd_vel`.
4. Viết lại `go2_ros2.py` = `go2_teleop.py` (nền đã chạy) **+** ROS 2 bridge (đã kiểm chứng).
5. Test theo thứ tự: `/cmd_vel` → topic list → `/scan` → TF → SLAM → Nav2.

---

## 🗓️ 2026-07-17 09:39 — Cập nhật #2: Đọc source `go2.py`, xác minh 4 điểm, phát hiện rủi ro lớn cho hướng B

### A. Đã đọc source thật: `robots/go2.py` — SỰ THẬT XÁC LẬP

**Chữ ký constructor (đọc từ code, không đoán):**
```python
Go2FlatTerrainPolicy(
    prim_path: str,
    root_path: str | None = None,      # <-- "path to the articulation root"
    usd_path: str | None = None,
    position: list[float] | None = None,
    orientation: list[float] | None = None,
    policy_path: str | None = None,
    env_config_path: str | None = None,
)
```

**USD mặc định (dòng trong `go2.py`):**
```python
if usd_path is None:
    usd_path = assets_root_path + "/Isaac/Samples/Mujoco_Menagerie/unitree_go2/go2/go2.usda"
```
→ **Policy NVIDIA được gắn với asset Go2 của Mujoco Menagerie**, KHÔNG phải `/Isaac/Robots/Unitree/Go2/go2.usd`.

**Policy mặc định (tải từ asset server, chọn theo physics engine đang bật):**
```python
policy_dir = assets_root_path + "/Isaac/Samples/Policies/go2"
active_engine = SimulationManager.get_active_physics_engine()
is_newton = active_engine == "newton"
policy_path     = f"{policy_dir}/newton_policy.pt"  if is_newton else f"{policy_dir}/physx_policy.pt"
env_config_path = f"{policy_dir}/newton_env.yaml"   if is_newton else f"{policy_dir}/physx_env.yaml"
```
→ **Xác nhận giả thuyết ở Cập nhật #1:** policy KHÔNG ở local, tải từ Nucleus. Có **2 cặp** policy: PhysX và Newton, tự chọn theo engine.

**Các chi tiết khác đọc được:**
| Sự thật | Code |
|---------|------|
| action_scale lấy từ env.yaml | `self._action_scale = self.policy_env_params.get("action_scale", 0.25)` |
| Obs layout 48 | Đúng y như đã ghi ở `step1a` mục 2 (docstring liệt kê rõ `[0:3]`…`[36:48]`) |
| Vận tốc lấy body frame đúng cách | `R_BI = R_IB.squeeze().t()` rồi `lin_vel_b = R_BI @ ...` |
| Gravity | `gravity_b = R_BI @ torch.tensor([0.0, 0.0, -1.0])` |
| joint_vel dùng **hiệu với default_vel** | `obs[24:36] = current_joint_vel - self.default_vel` |
| Action đầu tiên = zeros | `if self._previous_action is None: ... torch.zeros(12)` |
| **`set_dof_position_targets` chỉ gọi TRONG block decimation** | Nằm trong `if self._policy_counter % self._decimation == 0:` |
| API robot | `get_velocities()`, `get_world_poses()`, `get_dof_positions()`, `get_dof_velocities()`, `set_dof_position_targets()` |

**→ Dùng được gì:** Không cần viết lại `_compute_observation`. Chỉ cần **truyền `command` (torch tensor 3 phần tử) vào `go2.forward(dt, command)`** — đó là toàn bộ điểm cắm của `/cmd_vel`.

### B. Kết quả 4 lệnh kiểm tra

#### B1. Tên node OmniGraph ROS 2
**Lệnh:** `find ~/isaacsim -path "*ros2.bridge*" -name "*.ogn" | sed 's|.*/||' | sort`
**Output:** *(RỖNG — không có file .ogn nào)*
**→ Kết luận:** Không enumerate được tên node bằng cách này. **Tên node OmniGraph tôi viết trong `go2_ros2.py` VẪN LÀ ĐOÁN — chưa được xác minh.** Cần cách khác (xem mục ❓ bên dưới).

#### B2. rclpy trong python của Isaac
**Lệnh:**
```bash
source /opt/ros/jazzy/setup.bash
~/isaacsim/python.sh -c "import rclpy; print('rclpy OK:', rclpy.__file__)"
```
**Output:**
```
rclpy OK: /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py
```
**→ ✅ XÁC NHẬN:** rclpy import được trong tiến trình Isaac. **Dùng được:** có thể chạy node ROS 2 ngay trong script → nhận `/cmd_vel`, publish `/odom` `/tf` `/joint_states` bằng rclpy thuần, **không bắt buộc phải dùng OmniGraph** cho các topic không cần render.

#### B3. RTX LiDAR configs
**Lệnh:** `find ~/isaacsim -iname "*.json" -path "*lidar*" | head -20`
**Output (trích):**
```
/home/azureuser/isaacsim/extsDeprecated/isaacsim.sensors.rtx/data/lidar_configs/
    NVIDIA/Debug_Rotary.json
    NVIDIA/Simple_Example_Solid_State.json
    Velodyne/Velodyne_VLS128.json
    SICK/SICK_multiScan165.json, SICK_tim781.json, SICK_picoScan150.json, ...
    Ouster/OS1/OS1_REV6_128ch10hz2048res.json, OS1_REV7_128ch20hz1024res.json, ...
    HESAI/Hesai_XT32_SD10.json
    ZVISION/ZVISION_ML30S.json, ZVISION_MLXS.json
```
**→ 2 kết luận quan trọng:**
1. **`"Example_Rotary"` mà tôi viết trong `go2_ros2.py` KHÔNG TỒN TẠI** → sai. Tên thật có: `Debug_Rotary`, `Simple_Example_Solid_State`, `Velodyne_VLS128`, `OS1_*`, `SICK_*`…
2. **Đường dẫn nằm trong `extsDeprecated/`** → extension `isaacsim.sensors.rtx` **đã bị deprecate ở 6.0.1**. Nghĩa là API tạo LiDAR tôi viết (`IsaacSensorCreateRtxLidar`) **có thể đã đổi**. Phải tìm API mới.

### C. ⚠️ RỦI RO LỚN CHO HƯỚNG B (user đã chọn B)

User chọn **hướng B**: mở `build_from_scratch.usd` và gắn policy vào con Go2 có sẵn.

**Vấn đề phát hiện từ source `go2.py`:**
- Policy NVIDIA được train trên asset **`/Isaac/Samples/Mujoco_Menagerie/unitree_go2/go2/go2.usda`**.
- Con Go2 trong `build_from_scratch.usd` của user được kéo thả từ **`/Isaac/Robots/Unitree/Go2/`** (theo mô tả ban đầu của user).
- **Đây là 2 asset KHÁC NHAU.** Nếu **thứ tự / tên 12 khớp khác nhau** → policy nhận obs sai thứ tự → **Go2 ngã ngay** (đúng cơ chế đã mô tả ở `step1a` mục 5).

**→ CHƯA KIỂM TRA ĐƯỢC. Không được đoán. Phải so sánh joint order của 2 asset trước khi đi tiếp hướng B.**

**Điểm sáng:** constructor có sẵn tham số **`root_path`** ("path to the articulation root") → về mặt API, hướng B **có vẻ khả thi**. Nhưng phải đọc `policy_controller.py` mới biết `prim_path` / `root_path` / `usd_path` tương tác thế nào (có spawn đè không, hay attach vào prim có sẵn).

### D. Cập nhật trạng thái file

| File | Trạng thái mới | Lý do |
|------|---------------|-------|
| `go2_step1/go2_ros2.py` | ❌ **XÁC NHẬN SAI ở 3 điểm** | (1) API cũ (`World`) — sai; (2) tự viết `_compute_observation` — thừa, `Go2FlatTerrainPolicy` đã có; (3) `config="Example_Rotary"` — **không tồn tại** |
| `go2_step1/README.md` | ❌ Mục CONFIG sai | `POLICY_PT_PATH` trỏ file local — thực tế policy tải từ asset server, để `None` là đủ |
| `step1-go2-ros2-nav2-pipeline.md` mục 4.5 | ❌ Sai | Bảo "find policy .pt trên máy" — không có. Đúng ra: để `policy_path=None`, class tự tải |

---

## ❓ CHƯA RÕ — CẦN KIỂM TRA NGAY (cập nhật sau #2)

| # | Câu hỏi | Cách kiểm tra | Vì sao cần |
|---|---------|---------------|-----------|
| 1 | **Joint order của Go2 trong `build_from_scratch.usd` có khớp Mujoco Menagerie Go2 không?** | Traverse USD, in tên joint của cả 2 asset rồi so | **CHẶN hướng B.** Lệch = ngã ngay |
| 2 | `prim_path` / `root_path` / `usd_path` tương tác ra sao? Attach được vào prim có sẵn không? | `cat controllers/policy_controller.py` | **CHẶN hướng B.** Quyết định cách gắn policy |
| 3 | Tên node OmniGraph ROS 2 thật ở 6.0.1 | `find ~/isaacsim -path "*ros2*" -name "Ogn*.py"` hoặc list qua `og` | Hết đoán tên node |
| 4 | ROS 2 bridge nằm ở `exts/` hay `extsDeprecated/`? | `ls ~/isaacsim/exts \| grep -i ros2` | Biết extension nào còn dùng được |
| 5 | API tạo RTX LiDAR mới ở 6.0.1 (vì `isaacsim.sensors.rtx` đã deprecated) | `ls ~/isaacsim/exts \| grep -i sensor` | `IsaacSensorCreateRtxLidar` có thể đã đổi |
| 6 | Physics engine đang dùng là PhysX hay Newton? | `SimulationManager.get_active_physics_engine()` | Quyết định policy nào được tải (`physx_policy.pt` vs `newton_policy.pt`) |

---

## 🗓️ 2026-07-17 10:15 — Cập nhật #3: Xác minh joint order, đọc `policy_controller.py`, PHÁT HIỆN SCENE RỖNG

### A. ✅ Thứ tự 12 khớp mà policy NVIDIA mong đợi — ĐÃ XÁC MINH BẰNG OUTPUT THẬT

**Lệnh:** chạy `go2_teleop.py` (USE_MY_POLICY=False), đọc dòng debug.
**Output thật:**
```
>>> Thu tu 12 khop: ['FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
                     'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
                     'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint']
>>> default_pos: ['+0.10','-0.10','+0.10','-0.10','+0.80','+0.80','+1.00','+1.00','-1.50','-1.50','-1.50','-1.50']
>>> GRAVITY THO buoc 1: [+0.24 -0.21 -0.95]
```

**Phân tích (đối chiếu với `UNITREE_GO2_CFG` của Isaac Lab đã đọc ở Cập nhật trước):**

| Kiểm chứng | Kết quả |
|-----------|---------|
| Thứ tự gom **theo LOẠI khớp** (4 hip → 4 thigh → 4 calf), mỗi loại theo FL/FR/RL/RR | ✅ Đúng như `step1a` mục 5.2 đã ghi |
| `L_hip = +0.1`, `R_hip = -0.1` | ✅ Khớp `UNITREE_GO2_CFG` |
| `F*_thigh = 0.8` (FL,FR), `R*_thigh = 1.0` (RL,RR) | ✅ Khớp `UNITREE_GO2_CFG` |
| `*_calf = -1.5` (cả 4) | ✅ Khớp `UNITREE_GO2_CFG` |
| Gravity Z ≈ **-0.95** (≈ -1) | ✅ Convention ĐÚNG → `FLIP_GRAVITY = False` là chuẩn. XY = (0.24, -0.21) do robot đang nghiêng nhẹ lúc settle — bình thường |

**→ Kết luận quan trọng:** Asset **Mujoco Menagerie Go2** (policy NVIDIA dùng) có **joint order + default_pos GIỐNG HỆT** `UNITREE_GO2_CFG` của Isaac Lab. Nghĩa là **policy bạn tự train (trên asset Isaac Lab) và policy NVIDIA dùng chung một convention khớp** → giải thích vì sao `USE_MY_POLICY=True` cũng chạy được. Rủi ro "2 asset khác nhau" mà tôi lo ở Cập nhật #2 **KHÔNG xảy ra giữa 2 asset này**.

### B. ✅ Đọc `policy_controller.py` — HƯỚNG B ĐƯỢC API HỖ TRỢ

**Code quyết định (đọc thật, không đoán):**
```python
prim = get_prim_at_path(prim_path)
if not prim.IsValid():                      # <-- CHỈ spawn khi prim CHƯA tồn tại
    prim = define_prim(prim_path, "Xform")
    if usd_path:
        prim.GetReferences().AddReference(usd_path)
    else:
        carb.log_error("unable to add robot usd, usd_path not provided")
self._prim_path = prim_path
self._set_physics_variant(prim_path)
self.robot = Articulation(
    paths=prim_path if root_path is None else root_path, ...)
```

**→ Ý nghĩa cho hướng B:**
- Nếu prim tại `prim_path` **ĐÃ TỒN TẠI** (`prim.IsValid() == True`) → **KHÔNG spawn, KHÔNG add reference**. Nó chỉ bọc prim có sẵn vào `Articulation`.
- **→ Hướng B khả thi:** mở `build_from_scratch.usd`, gọi `Go2FlatTerrainPolicy(prim_path="<prim Go2 có sẵn>")` → gắn policy vào con Go2 trong scene, **không bị 2 con Go2**.
- `root_path` dùng khi articulation root nằm ở prim con khác `prim_path`.

**Phát hiện thêm — cơ chế chống lệch tên khớp:**
```python
max_effort, max_vel, stiffness, damping, armature, default_pos, default_vel = \
    get_robot_joint_properties(self.policy_env_params, self.robot.dof_names)
```
→ `initialize()` tra cứu thuộc tính khớp **THEO TÊN** (`self.robot.dof_names`), không theo index cứng. Nên gains/default_pos **tự khớp đúng khớp** miễn là **tên khớp giống nhau**.
> ⚠️ Nhưng **mạng policy vẫn ăn obs theo THỨ TỰ dof của articulation**. Nếu scene có Go2 với **thứ tự dof khác** → obs vẫn bị xáo trộn. Tên giống ≠ thứ tự giống. Vẫn phải kiểm.

**Chi tiết khác đọc được:**
| Sự thật | Code |
|---------|------|
| Policy nạp bằng TorchScript qua `omni.client` (đọc được cả URL Nucleus) | `torch.jit.load(io.BytesIO(omni.client.read_file(path)[2]))` |
| `_decimation`, `_dt`, `render_interval` lấy từ env.yaml | `get_physics_properties(self.policy_env_params)` |
| Asset có **variant set "Physics"** (physx ↔ mujoco), tự chọn theo engine | `_set_physics_variant()`, `_ENGINE_TO_VARIANT = {"physx":"physx", "newton":"mujoco"}` |
| Nếu asset không có variant "Physics" → bỏ qua, không lỗi | `if "Physics" not in variant_sets.GetNames(): return` |
| `initialize()` set control mode = position, set gains/limits từ env.yaml | `switch_dof_control_mode("position")`, `set_dof_gains(...)` |

### C. ❌ CHẶN HƯỚNG B: `build_from_scratch.usd` KHÔNG CÓ ARTICULATION NÀO

**Lệnh:**
```bash
~/isaacsim/python.sh -c "
from pxr import Usd, UsdPhysics
s = Usd.Stage.Open('/home/azureuser/tan/build_from_scratch.usd')
for p in s.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        print('ARTICULATION ROOT:', p.GetPath())
joints = [p.GetName() for p in s.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
print('So joint:', len(joints))
for j in joints: print('  -', j)
"
```
**Output thật:**
```
So joint: 0
```
(và **không có dòng `ARTICULATION ROOT:` nào**)

**→ Sự thật:** Trong `build_from_scratch.usd`, traverse **không tìm thấy prim nào có `ArticulationRootAPI`, và 0 khớp revolute**.

**→ CHƯA KẾT LUẬN ĐƯỢC NGUYÊN NHÂN. Không đoán.** Các khả năng phải kiểm tra:
1. Go2 trong scene là **reference tới USD ngoài** (Nucleus/asset server) — khi `Usd.Stage.Open` standalone không resolve được reference → prim rỗng, không thấy joint.
2. Scene thật sự **chưa có Go2** (chỉ có warehouse).
3. Khớp được định nghĩa bằng schema khác nên `IsA(UsdPhysics.RevoluteJoint)` không bắt được.

**→ Phải dump toàn bộ prim của scene mới biết. Đây là việc kiểm tra tiếp theo.**

### D. ✅ Extension nào còn sống, cái nào đã deprecated

**Lệnh:** `ls ~/isaacsim/exts | grep -iE "ros2|sensor"` và `ls ~/isaacsim/extsDeprecated | grep -iE "ros2|sensor"`

**Output → bảng kết luận:**

| Extension | Vị trí | Trạng thái | Ảnh hưởng |
|-----------|--------|-----------|-----------|
| `isaacsim.ros2.bridge` | `exts/` | ✅ **CÒN DÙNG** | ROS 2 bridge OK |
| `isaacsim.ros2.core`, `.nodes`, `.tf_viewer`, `.urdf`, `.sim_control` | `exts/` | ✅ Còn dùng | `isaacsim.ros2.nodes` = nơi chứa node OmniGraph |
| `isaacsim.sensors.experimental.rtx` | `exts/` | ✅ **API MỚI** | Thay cho `isaacsim.sensors.rtx` |
| `isaacsim.sensors.rtx.nodes`, `.ui` | `exts/` | ✅ Còn dùng | Node OmniGraph cho RTX sensor |
| `isaacsim.sensors.rtx` | **`extsDeprecated/`** | ❌ **DEPRECATED** | API tạo LiDAR cũ của tôi có thể hỏng |
| `isaacsim.sensors.camera` | **`extsDeprecated/`** | ❌ **DEPRECATED** | `from isaacsim.sensors.camera import Camera` trong `go2_ros2.py` — **SAI** |

**→ Xác nhận thêm 2 lỗi trong `go2_step1/go2_ros2.py`:** import camera từ extension đã deprecated, và dùng API RTX LiDAR cũ.

### E. Tổng kết trạng thái sau Cập nhật #3

| Câu hỏi từ #2 | Trả lời |
|---------------|---------|
| Joint order Mujoco Menagerie vs Isaac Lab? | ✅ **GIỐNG HỆT** — không có rủi ro như tôi lo |
| `prim_path`/`root_path` attach được prim có sẵn? | ✅ **ĐƯỢC** — hướng B hợp lệ về API |
| rclpy dùng được? | ✅ Được |
| ROS2 bridge còn sống? | ✅ `exts/isaacsim.ros2.bridge` |
| LiDAR/Camera API? | ❌ Cũ đã deprecated → phải dùng `isaacsim.sensors.experimental.rtx` |
| Scene có Go2 không? | ❌ **0 articulation, 0 joint — CHƯA RÕ VÌ SAO** |

---

## ❓ CHƯA RÕ — CẦN KIỂM TRA NGAY (cập nhật sau #3)

| # | Câu hỏi | Cách kiểm tra | Vì sao cần |
|---|---------|---------------|-----------|
| 1 | **`build_from_scratch.usd` thực sự chứa gì?** Có Go2 không? Reference có resolve không? | Dump toàn bộ prim + references | **CHẶN hướng B** |
| 2 | Nếu có Go2 → prim path + dof order của nó? | Mở bằng SimulationApp (resolve được reference) rồi in `dof_names` | **CHẶN hướng B** |
| 3 | Tên node OmniGraph ROS 2 thật | `find ~/isaacsim/exts/isaacsim.ros2.* -name "Ogn*.py"` | Hết đoán |
| 4 | API tạo RTX LiDAR ở `isaacsim.sensors.experimental.rtx` | `find ~/isaacsim/exts/isaacsim.sensors.experimental.rtx -name "*.py"` | API cũ đã deprecated |
| 5 | Camera API thay thế là gì? | `ls ~/isaacsim/exts/isaacsim.sensors.camera.ui`; tìm trong `experimental` | `isaacsim.sensors.camera` deprecated |
| 6 | Physics engine đang active = physx hay newton? | in `SimulationManager.get_active_physics_engine()` | Quyết định policy nào được tải |

---

## 🗓️ 2026-07-21 07:36 — Cập nhật #4: GỠ CHỖ TẮC — scene CÓ Go2, hướng B sống

### A. ✅ `build_from_scratch.usd` chứa gì — ĐÃ RÕ

**Lệnh:** `Usd.Stage.Open(...)` + `strings ... | grep -i go2`
**Output thật (warning + strings):**
```
Warning: In </World/IsaacWarehouse>: Could not open asset
  @https:/.../Assets/ArchVis/Industrial/Stages/IsaacWarehouse.usd@
  for payload introduced by .../build_from_scratch.usd</World/IsaacWarehouse>
Warning: In </World/go2>: Could not open asset
  @https:/.../Assets/Isaac/6.0/Isaac/Robots/Unitree/Go2/go2.usd@
  for payload introduced by .../build_from_scratch.usd</World/go2>
So joint: 0

strings output:
  ?go2C
  /Robots/Unitree/Go2/go2
```

**→ Kết luận (chắc chắn, không đoán):**
1. Scene **CÓ 2 prim**: `/World/IsaacWarehouse` và **`/World/go2`**.
2. `/World/go2` là **payload** trỏ tới `.../Isaac/6.0/Isaac/Robots/Unitree/Go2/go2.usd` (asset **Unitree**, không phải Mujoco Menagerie).
3. Lý do "So joint: 0" ở Cập nhật #3 = **payload không resolve khi mở offline** (`Usd.Stage.Open` standalone không tải được từ `https://`). Prim rỗng vì asset chưa nạp. **Không phải scene thiếu Go2.** Khi mở bằng SimulationApp (có kết nối asset server) → joint sẽ hiện.

**→ prim path con Go2 trong scene = `/World/go2`** (chữ thường). Đây là mảnh cần cho hướng B.

### B. Hệ quả cho hướng B

| Yếu tố | Trạng thái |
|--------|-----------|
| Scene có Go2 để attach | ✅ `/World/go2` |
| API cho phép attach prim có sẵn | ✅ (Cập nhật #3: `if not prim.IsValid()` mới spawn) |
| Asset Go2 trong scene | Unitree `/Isaac/Robots/Unitree/Go2/go2.usd` |
| Joint order Unitree asset == order policy mong đợi? | ❓ **CHƯA XÁC MINH** — teleop chỉ mới in order của Mujoco asset. Phải attach vào `/World/go2` rồi in `dof_names` để so |
| Articulation root nằm ở `/World/go2` hay prim con? | ❓ Chưa rõ — có thể cần `root_path` |
| Scene có `PhysicsScene` chưa? | ❓ Chưa rõ — nếu thiếu, physics không chạy, phải tự thêm |

### C. ✅ Extension RTX experimental tồn tại

**Lệnh:** `find ~/isaacsim/exts/isaacsim.sensors.experimental.rtx -name "*.py"`
**Output:** có module `isaacsim/sensors/experimental/rtx/` + thư mục `tests/` (`test_lidar_sensor.py`, `test_rtx_lidar_configs.py`, `test_camera_sensor.py`…). `head` chỉ hiện phần tests, **chưa thấy file API chính** (bị cắt).
**→ Cần xem tiếp:** `__init__.py` export class gì (tên class LiDAR/Camera thật).

### D. Quyết định kiến trúc (dựa trên sự thật đã xác minh)

Vì rclpy chạy trong Isaac (Cập nhật #2) và ta có đủ API robot đã kiểm chứng, **chia Step 1 làm 3 stage, test từng stage**:

| Stage | Nội dung | Phụ thuộc | Rủi ro |
|-------|----------|-----------|--------|
| **B0** | Attach policy vào `/World/go2` trong scene → WASD đi được | Chỉ API đã proven ở `go2_teleop.py` | Thấp — chỉ đổi cách nạp scene + prim_path |
| **B1** | Publish `/clock` `/odom` `/tf` `/joint_states` bằng **rclpy thuần** + subscribe `/cmd_vel` | rclpy (đã xác nhận) + Articulation API | Trung bình — cần đúng API đọc pose/vel |
| **B2** | Gắn RTX LiDAR → `/scan`, camera → ảnh, qua OmniGraph `isaacsim.ros2.bridge` | Cần tên node/API experimental.rtx | Cao — chưa xác minh tên node |

→ Làm **B0 trước** (thấp rủi ro nhất, xác nhận hướng B), rồi B1, rồi B2. SLAM/Nav2 chỉ vào cuộc sau B2.

### E. File mới tạo lần này

| File | Tác dụng |
|------|----------|
| `go2_step1/go2_check_sceneB.py` | **Stage B0**: mở `build_from_scratch.usd`, tự dò ArticulationRoot, attach `Go2FlatTerrainPolicy` vào Go2 có sẵn, in `dof_names`, settle, cho WASD. **Mục đích: xác nhận hướng B đi được + in ra joint order thật của asset Unitree.** Chỉ dùng API đã proven từ `go2_teleop.py` |

---

## ❓ CHƯA RÕ — CẦN KIỂM TRA (cập nhật sau #4)

| # | Câu hỏi | Cách kiểm tra | Chặn cái gì |
|---|---------|---------------|-------------|
| 1 | Chạy `go2_check_sceneB.py`: Go2 trong scene có đi được không? `dof_names` ra sao? | Chạy script, đọc log + quan sát | **Xác nhận Stage B0 / hướng B** |
| 2 | Scene có sẵn `PhysicsScene` không? | Script B0 tự in cảnh báo nếu thiếu | Physics chạy được không |
| 3 | Class LiDAR/Camera trong `experimental.rtx` tên gì? | `cat .../experimental/rtx/__init__.py` | Chặn Stage B2 |
| 4 | Tên node OmniGraph ROS 2 (cho `/scan`, camera) | `find ~/isaacsim/exts -path "*ros2*" -name "Ogn*.py"` | Chặn Stage B2 |
| 5 | Physics engine active = physx/newton? | Script B0 in ra | Chọn policy physx vs newton |
