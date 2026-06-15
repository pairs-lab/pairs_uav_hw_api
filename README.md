# pairs_uav_hw_api

Hardware abstraction layer of the PAIRS UAV stack. It defines the pluginlib
interface that sits between the autonomy stack (control, estimation) and the
low-level flight controller, so the rest of the stack can command any vehicle —
PX4, an SDK drone, a simulator — through one uniform API. Concrete platforms
(for example the Tello driver) implement this interface as plugins; this package
provides the base class, the manager that loads a chosen plugin at runtime, and
a dummy implementation for testing.

## Contents

- `pairs_uav_hw_api::PairsUavHwApi` — the abstract plugin base class that every
  hardware driver implements (status/capabilities reporting plus actuator,
  attitude, acceleration, velocity, position, and tracker command callbacks).
- `HwApiManager` — the component that loads the configured HW API plugin and
  wires it into the stack.
- `pairs_uav_hw_api::DummyApi` — a no-op plugin implementation used for tests and
  bring-up.
- `launch/hw_api.launch.py` and `config/hw_api.yaml`.

## Branches

- `ros1` — ROS 1 Noetic (catkin).
- `ros2` — ROS 2 Jazzy (ament_cmake).

## Install (ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-pairs-uav-hw-api
```

## Usage

```bash
ros2 launch pairs_uav_hw_api hw_api.launch.py
```

## License
BSD 3-Clause (© 2020 PAIRS group, CTU Prague; PAIRS adaptations © 2024 Thanh Nguyen Canh). See [LICENSE](LICENSE).
