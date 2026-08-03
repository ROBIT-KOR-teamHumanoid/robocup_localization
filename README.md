# robocup_localization - 2026 

Monte Carlo Localization (MCL) package for RoboCup humanoid soccer robots.

The package estimates the robot pose `(x, y, yaw)` by combining odometry, IMU measurements, and field feature detections from the vision system. The estimated pose is then used to transform ball, obstacle, and teammate information into the global field coordinate system.

---

## Localization Pipeline

### 1. Motion Update

Walking odometry from the IK controller is used to propagate particles across the field.

* Particle motion noise is configurable
* Updated whenever new odometry is received

### 2. Observation Update

Field landmarks detected by the vision system are matched against the field model.

Supported landmarks:

* L-Corner
* T-Junction
* X-Cross

Particle weights are updated according to observation likelihood, followed by resampling.

### 3. Yaw Correction

Goalpost observations are used to compensate for accumulated IMU yaw drift.

This improves long-term heading stability during matches.

### 4. Global Coordinate Generation

The estimated robot pose is used to convert:

* Ball position
* Opponent positions
* Teammate positions

into the global field coordinate frame.

---

## Topic Interface

### Subscribed Topics

| Topic              | Description                                     |
| ------------------ | ----------------------------------------------- |
| `/vision_feature`  | Field feature detections used for MCL updates   |
| `/vision`          | Ball and opponent position information          |
| `/ikcoordinate`    | Walking odometry                                |
| `/Imu`             | Robot orientation (yaw)                         |
| `/gamecontroldata` | Game state and penalty information              |
| `/udp`             | Teammate localization data received through UDP |

### Published Topics

| Topic                  | Description                                              |
| ---------------------- | -------------------------------------------------------- |
| `/localization`        | Estimated robot pose and transformed world-frame objects |

---

## Data Flow

```text
Vision Features ──┐
                  │
Walking Odometry ─┼──► Particle Filter (MCL) ───► Robot Pose
                  │
IMU Yaw ──────────┘
                         │
                         ▼
                 Global Coordinate Transform
                         │
                         ├── Ball Position
                         ├── Opponent Positions
                         └── Teammate Positions
```

---

## Build

```bash
colcon build --packages-select robocup_localization
```

## Launch

```bash
source install/setup.bash
ros2 launch robocup_localization robocup_localization.launch.py
```
