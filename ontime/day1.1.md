# Kiến trúc điều khiển Robot tổng quát (Generic Robot Control Architecture)

> Kiến trúc này **áp dụng cho mọi loại robot**: AMR (mobile), cánh tay công nghiệp
> (industrial arm), robot custom, máy CNC, và mobile manipulator (arm gắn trên base
> di động). Nguyên tắc: **bộ khung tầng (layer) là bất biến; mỗi loại robot là một
> "profile" cắm component cụ thể vào từng tầng.** Tầng nào không cần thì bỏ trống.

---

## 0. Nguyên tắc thiết kế

Hệ thống tách tầng theo **mức độ gần phần cứng** và **tần số/độ chặt real-time
(control rate & criticality)** — đây là ranh giới bất biến, không đổi theo loại robot:

| Tầng | Vai trò | Tính chất thời gian |
|---|---|---|
| **L1 — HMI & Simulation** | Người vận hành cấu hình, giám sát, ra mission | Event-driven (chậm) |
| **(Fleet) — Server tier** | Quản lý đội robot, lập lịch mission | Chậm, có thể trên cloud |
| **L2 — Offline Planning & Code Gen** *(tùy chọn)* | Tính toán nặng *design-time*, sinh dữ liệu tĩnh + mã | Offline (một lần) |
| **Orchestration — Behavior Tree** | Điều phối logic runtime, recovery | Soft real-time (~1–10 Hz) |
| **Bridge — ROS 2** | Perception + motion planning/execution real-time | Soft real-time (10–100 Hz) |
| **L3 — Firmware / Controller** | Điều khiển motor, đọc sensor | Hard real-time (1–20 kHz) |

**Hai quy tắc bất biến (đúng cho mọi robot):**
1. **Tách offline vs online.** Tính toán nặng, tĩnh (quỹ đạo tham chiếu, sinh mã) làm
   *offline* (L2). Kiểm tra va chạm với môi trường thật, re-planning làm *online* (Bridge/ROS).
   → Không bao giờ đặt MATLAB Engine trong vòng lặp real-time.
2. **Một bộ não điều phối duy nhất.** Toàn bộ logic runtime đi qua **một cây Behavior
   Tree gốc** (Mission Executor). Các stack con (Nav2, MoveIt) tự dùng BT bên trong,
   nhưng chỉ có **một** BT cấp cao ra quyết định — tránh hai lớp logic chồng nhau.

---

## 1. Bảng Profile — loại robot nào cắm gì vào tầng nào

Đây là phần cốt lõi của tính tổng quát. Đọc theo cột để biết một loại robot dùng gì:

| Tầng | **AMR (mobile)** | **Cánh tay công nghiệp** | **Robot custom** | **CNC** | **Mobile manipulator** |
|---|---|---|---|---|---|
| L2 Offline planning | *(thường bỏ)* | MATLAB traj + code gen | MATLAB (tùy) | MATLAB → G-code | MATLAB cho arm |
| L2 Code Gen | – | TP / LS / Karel | – | **G-code** | TP/Karel |
| Bridge — Motion | **Nav2** (planner+controller) | **MoveIt + OMPL** | custom / MoveIt | G-code interpreter | **Nav2 + MoveIt** |
| Bridge — Localization | **SLAM + AMCL** | *(cố định, không cần)* | encoder/IMU | *(tọa độ máy)* | SLAM + arm FK |
| Bridge — Perception | Camera/Lidar, obj detect | camera (bin-picking) | tùy sensor | probe/touch-off | cả hai |
| L3 — Controller | wheel motor driver (STM32) | **Fanuc/ABB/KUKA controller** | **STM32 + FreeRTOS** | **CNC controller** (LinuxCNC/GRBL) | STM32 base + vendor arm |
| Fieldbus L3 | CAN-FD / micro-ROS | vendor (Ethernet/IP, PROFINET) | EtherCAT / CAN-FD | Step/Dir, EtherCAT | mix |

> Cách đọc: một **AMR thuần** = cột 1 → bỏ hẳn L2, Bridge dùng Nav2+SLAM, L3 là driver
> bánh xe. Một **cell cánh tay Fanuc** = cột 2 → L2 sinh Karel, Bridge dùng MoveIt,
> L3 là controller Fanuc (không có STM32). Một **mobile manipulator** = ghép cột 1 + 2.

---

## 2. Layer 1 — HMI & Simulation (bất biến)

Bốn công cụ hiển thị phục vụ **mục đích khác nhau**, không đồng nhất:

| Công cụ | Vai trò thực tế | Dành cho |
|---|---|---|
| **React / Node.js** | Dashboard: monitoring, config, user control | Operator / end-user |
| **Unreal Engine** | Real-time 3D rendering, **Digital Twin** | Operator / demo |
| **Gazebo** | **Physics simulation** (test thuật toán trước khi chạy thật) | Kỹ sư / CI |
| **RViz** | Debug visualization của ROS (TF, pointcloud, planning scene) | **Developer** (không phải UI người dùng) |

- **CAD Integration:** user tùy biến robot (limbs, sensors) → xuất **URDF/SDF** dùng
  chung cho Gazebo, RViz, MoveIt.
- **Digital Twin loop:** trạng thái robot thật (joint states, pose, cmd_vel) chảy ngược
  từ ROS → Unreal để twin phản ánh đúng thực tế.
- **Giao thức L1 ↔ Backend:** WebSocket (telemetry real-time) + REST (config/command), payload JSON.

### 1b. Fleet / Server tier (tùy chọn — cần khi có nhiều robot)

Dành cho đội robot hoặc triển khai cloud. Với **1 robot đơn lẻ**, gộp tier này vào
Backend trên IPC (không cần server riêng).

- **Mission Builder:** công cụ soạn mission (thuộc L1 về bản chất).
- **Scheduler / Task Management:** phân phối & lập lịch mission cho từng robot; nhận
  phản hồi trạng thái. Giao tiếp Server ↔ Robot qua WiFi/mạng (WebSocket/gRPC/MQTT).

---

## 3. Layer 2 — Offline / High-level Planning & Code Generation *(tùy chọn)*

Chạy **offline / design-time** trên IPC (hoặc máy dev). **Chỉ xuất hiện khi tác vụ có
tính lặp lại, môi trường tĩnh biết trước** (cánh tay, CNC). **AMR trong môi trường
động thường bỏ tầng này** vì planning phải online.

- Motion planning cấp cao: path planning, **trajectory generation, smoothing, time scheduling**.
- **Kinematic solvers** (IK/FK) cho cấu hình cụ thể.
- Collision check **với world tĩnh a-priori** → quỹ đạo tham chiếu collision-free (static data).
- **Code Generation Engine:** sinh mã cho phần cứng đích — **TP/LS/Karel** (robot công
  nghiệp), **G-code** (CNC).

> **Khuyến nghị:** không gọi *MATLAB Engine* trong runtime. Dùng **MATLAB Coder** biên
> dịch thuật toán planning thành **node C/C++** nhúng vào ROS → không cần MATLAB runtime
> khi deploy, không dính license, latency thấp.

---

## 4. Orchestration — Behavior Tree / Mission Executor (bất biến)

Thay **FSM** và logic if-else kiểu PLC bằng **Behavior Tree** để logic có cấu trúc,
tái sử dụng, dễ mở rộng. Đây là **bộ não điều phối duy nhất**, giống nhau cho mọi robot.

- Mỗi task = một **node** độc lập, tái sử dụng giữa các cây.
- Thư viện node viết bằng **C++/Python**; runtime dùng **BehaviorTree.CPP**; visualize/debug bằng **Groot2**.
- **Lưu ý quan trọng:** `Mission Executor` **chính là** cây BT gốc. Nav2 và nhiều pipeline
  MoveIt tự dùng BT.CPP bên trong → chúng là **BT con** được BT gốc gọi xuống. Đừng tạo
  hai lớp điều phối tách rời (Mission Executor riêng + BT riêng) vì logic sẽ đá nhau.

### Behavior Tree gốc — tổng quát cho mọi profile

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Fallback name="RootFallback">
      <Sequence name="ExecuteMission">
        <Condition ID="IsMissionAvailable"/>    <!-- Có task/mission để chạy không? -->
        <Action    ID="PlanMotion"/>            <!-- Nav2 | MoveIt | traj từ L2 — tùy profile -->
        <Condition ID="CheckCollisionROS"/>     <!-- Va chạm REAL-TIME với data lidar/camera -->
        <Action    ID="ExecuteMotion"/>         <!-- cmd_vel | joint trajectory xuống L3 -->
      </Sequence>
      <Action ID="SafeRecovery"/>               <!-- Fail bất kỳ bước nào -> về trạng thái an toàn -->
    </Fallback>
  </BehaviorTree>
</root>
```

Cùng một cây này chạy cho mọi robot; chỉ **phần hiện thực node** khác nhau:
- `PlanMotion` → gọi Nav2 (AMR) / MoveIt (arm) / nạp quỹ đạo L2 (CNC).
- `ExecuteMotion` → gửi `cmd_vel` (AMR) hoặc `joint trajectory` (arm) xuống L3.
- `CheckCollisionROS` → luôn query **ROS/costmap/planning-scene (sensor real-time)**, không query MATLAB.
- **`Rule Engine`** (nếu có ở perception) chỉ chứa *rule mức nhận thức* (phân loại
  người/vật). Quyết định hành động ("thấy người → dừng") vẫn nằm ở BT này, không nằm ở Rule Engine.

---

## 5. Bridge — ROS 2 (bất biến về vai trò, pluggable về component)

Cầu nối Orchestration ↔ L3, trao đổi qua **topics / services / actions**. Middleware
chung cho mọi robot; chỉ **stack motion/localization thay đổi theo profile** (xem Bảng 1).

- **Motion planning & execution real-time:**
  - **Mobile base** → **Nav2** (planner + controller) + **SLAM** (Map Creation) + **AMCL** (Localization).
  - **Manipulator** → **MoveIt + OMPL** (planning trong config space, collision online).
  - **Mobile manipulator** → Nav2 + MoveIt phối hợp (di chuyển đến vị trí → thao tác).
- **Perception:** sensor driver (camera, lidar), **CV, SLAM**, object/person detection.
- **Kết hợp dữ liệu:** static data từ L2 (nếu có) **+** dữ liệu real-time (lidar/camera)
  → **local re-planning** và **exception handling** khi có vật cản động.
- **Giao tiếp UI ↔ ROS:** `rosbridge_suite` chuyển JSON/WebSocket ↔ ROS message. (Khác với
  shared memory — chỉ dùng cho trao đổi tốc độ cao trong cùng máy.)

---

## 6. Layer 3 — Robot SDK / Firmware / Controller (pluggable)

Nhận setpoint/`cmd_vel` từ Bridge → điều khiển motor hard-real-time. **Hiện thực khác
hẳn nhau theo loại robot:**

- **Custom robot** → **C/C++ + FreeRTOS trên STM32**: PID motor control, sensor
  integration. Nhận setpoint từ IPC → chạy vòng PID (current loop 10–20 kHz).
- **Industrial arm** → **vendor controller (Fanuc / ABB / KUKA)** chạy chương trình
  **TP/Karel** (do L2 sinh). IPC/ROS chỉ gửi lệnh cấp cao; controller lo motor nội bộ.
  Tích hợp qua **ROS-Industrial** (socket TCP + Karel server) hoặc Ethernet/IP, PROFINET.
- **CNC** → **CNC controller** (LinuxCNC / GRBL / vendor) thực thi **G-code**.
- **Mobile base (AMR)** → **motor/wheel driver** (STM32 hoặc driver thương mại) nhận
  `cmd_vel` → điều khiển bánh. *(Đây là tầng bị trừu tượng hóa ẩn đi trong sơ đồ AMR
  chỉ vẽ tới Nav2 Controller — Nav2 xuất `cmd_vel`, còn cần tầng này biến `cmd_vel`
  thành lệnh motor.)*

Tất cả giao tiếp với robot vật lý qua **Robot Controller** tương ứng.

---

## 7. Hardware & Communication (bất biến về pattern)

### IPC / PLC
- **IPC (Industrial PC):** chạy ROS 2, Backend, (node C++ do MATLAB Coder sinh), Behavior Tree. Là master điều phối.
- **PLC (tùy chọn):** giữ cho **safety I/O phần cứng** (E-stop, light curtain, interlock)
  — mạch an toàn phải **độc lập với phần mềm**. Logic nghiệp vụ chuyển lên BT; PLC chỉ lo an toàn cứng.

### Bảng giao thức tổng hợp

| Kết nối | Giao thức | Ghi chú |
|---|---|---|
| UI ↔ Backend | WebSocket / REST | JSON |
| Server ↔ Robot (fleet) | WebSocket / gRPC / MQTT | qua WiFi/mạng |
| Backend ↔ ROS 2 | rosbridge (WS→ROS) | |
| BT ↔ L2 | Node C++ (MATLAB Coder) / MATLAB Engine | ưu tiên compiled |
| BT ↔ ROS nodes | ROS 2 topic/service/**action** | in-process |
| IPC ↔ Custom L3 (STM32) | **EtherCAT** / CAN-FD / micro-ROS | real-time cyclic |
| IPC ↔ Vendor arm | ROS-Industrial (TCP+Karel) / Ethernet-IP / PROFINET | |
| IPC ↔ CNC | Step/Dir / EtherCAT | |

> **Lưu ý EtherCAT + STM32:** STM32 **không có EtherCAT native** → cần chip **ESC**
> (LAN9252 / ET1100) giao tiếp qua SPI. Nếu ít trục, **CAN-FD** đơn giản hơn; hoặc
> **micro-ROS** để STM32 thành ROS 2 node trực tiếp (hợp prototype).

---

## 8. Luồng dữ liệu: Design-time vs Runtime (bất biến)

**Design-time (offline — chỉ profile có L2):**
CAD/URDF → path planning → smoothing → time schedule → collision check (world tĩnh)
→ **quỹ đạo tham chiếu + mã TP/Karel/G-code**.

**Runtime (online — mọi robot):**
Mission → BT gốc → `PlanMotion` (Nav2/MoveIt/traj) → **`CheckCollisionROS` với sensor thật**
→ nếu vật cản động thì **local re-planning** → `ExecuteMotion` (cmd_vel/joint) xuống L3
→ feedback → cập nhật Digital Twin → lặp lại.

### Sơ đồ luồng runtime tổng quát

```
[ L1: HMI ]  React · Unreal(Twin) · Gazebo(sim) · RViz(debug)
   |  ^                                    [ Fleet server (tùy chọn) ]
   |  |  WebSocket/REST                     Mission Builder · Scheduler
   v  |  (telemetry ↑ về Digital Twin)              |  ^  (WiFi)
=====|==|==================================================|==|========
| IPC (INDUSTRIAL PC)                                      v  |        |
|  [ Backend ]  <--- UI + (nhận mission từ server)         |          |
|      | rosbridge                                                     |
|  [ Behavior Tree = Mission Executor ]  <--- bộ não điều phối duy nhất|
|      /            \                                                  |
|     v              v                                                 |
| [L2: Planner]   [Bridge: ROS 2]                                      |
|  (tùy chọn)      - Motion: Nav2 (mobile) | MoveIt (arm)              |
|  MATLAB/Coder    - Localization: SLAM/AMCL (nếu mobile)              |
|  TP/Karel/Gcode  - Perception: camera/lidar, detection              |
|      \_____________/  static data + sensor -> local re-planning      |
=========================================|============================
                                         | EtherCAT/CAN/micro-ROS/vendor
                                         v
[ L3: Controller — theo profile ]
   STM32+FreeRTOS (custom) | Fanuc/ABB/KUKA (arm) | CNC ctrl | wheel driver (AMR)
                                         |
                                         v
                          [ Robot vật lý + Sensors ]
                                         |  (sensor ↑ quay lại ROS: SLAM/collision)
```

---

## 9. Ba ví dụ instantiation (điền profile vào khung)

**A. AMR (robot tự hành):** bỏ L2. BT gốc = "Mission Executor". Bridge = Nav2 + SLAM +
AMCL + object detection. L3 = wheel motor driver nhận `cmd_vel`. Safety = E-stop cứng.
*(Đây chính là sơ đồ Server/Robot bạn đưa — Mission Builder/Scheduler = Fleet tier;
Mission Executor = BT gốc; Navigation Stack + Perception = Bridge; Robot safety = L3 safety.)*

**B. Cell cánh tay công nghiệp:** L2 = MATLAB sinh quỹ đạo + Karel. Bridge = MoveIt+OMPL
(không SLAM/AMCL). L3 = controller Fanuc chạy Karel, tích hợp qua ROS-Industrial. Safety
= PLC + light curtain.

**C. Robot custom:** L2 tùy chọn. Bridge = MoveIt hoặc planner custom. L3 = STM32+FreeRTOS,
PID motor, giao tiếp EtherCAT/CAN-FD với IPC.

---

## Phụ lục — Các điểm đã sửa/làm rõ so với bản gốc

1. **Tổng quát hóa:** chuyển từ kiến trúc riêng cánh tay/CNC sang **framework tầng bất
   biến + profile theo loại robot** (Bảng 1). Tầng nào không cần thì bỏ (AMR bỏ L2/SLAM
   không cần; arm bỏ SLAM/AMCL).
2. **MATLAB không trong vòng real-time:** L2 = offline/static; collision real-time do
   ROS. Gợi ý MATLAB Coder để deploy không cần MATLAB runtime.
3. **XML BT hợp lệ:** bỏ text `-- ... --` trong cây (không hợp lệ) → dùng comment `<!-- -->`;
   trừu tượng hóa node (`PlanMotion`/`ExecuteMotion`) để dùng chung mọi profile.
4. **Mission Executor = Behavior Tree:** làm rõ chỉ có **một** BT gốc; Nav2/MoveIt là BT con.
5. **Rule Engine không ra quyết định hành động** — chỉ rule mức perception; quyết định ở BT.
6. **RViz ≠ UI người dùng;** tách vai Unreal (twin) / Gazebo (sim) / RViz (debug).
7. **Nav2 vs MoveIt** theo profile (mobile vs arm); mobile manipulator dùng cả hai.
8. **Thống nhất fieldbus** + lưu ý STM32 cần chip ESC cho EtherCAT.
9. **Bổ sung:** Fleet/Server tier (fleet), Digital Twin feedback, L3 safety cứng, và
   tầng L3 driver `cmd_vel` còn thiếu trong sơ đồ AMR.