# piper_gripper_hardware

ROS 2 `ros2_control` hardware plugin for Piper gripper over raw SocketCAN, without `piper_sdk`.

## Protocol basis

- **Command CAN ID**: `0x159` (`ARM_GRIPPER_CTRL`)
- **Feedback CAN ID**: `0x2A8` (`ARM_GRIPPER_FEEDBACK`)
- Frame layout follows AgileX Piper V2 protocol (big-endian):
  - Command: `int32 angle(0.001mm)`, `uint16 effort(0.001N*m)`, `uint8 status`, `uint8 set_zero`
  - Feedback: `int32 angle(0.001mm)`, `int16 effort(0.001N*m)`, `uint8 status bits`

## Build (Jazzy)

```bash
colcon build --packages-select piper_gripper_hardware
```

## ros2_control URDF snippet

```xml
<ros2_control name="piper_gripper" type="system">
  <hardware>
    <plugin>piper_gripper_hardware/PiperGripperHardware</plugin>
    <param name="can_interface">can0</param>
    <param name="command_can_id">345</param> <!-- 0x159 -->
    <param name="feedback_can_id">680</param> <!-- 0x2A8 -->
    <param name="min_position_m">0.0</param>
    <param name="max_position_m">0.08</param>
    <param name="initial_position_m">0.0</param>
    <param name="default_effort_mn">1000</param>
    <param name="feedback_timeout_ms">1000</param>
    <param name="command_refresh_interval_ms">100</param>
    <param name="publish_debug_commands">false</param>
    <param name="publish_sent_position">false</param>
    <param name="publish_feedback_position">false</param>
    <param name="debug_topic_prefix"></param>
  </hardware>

  <joint name="gripper_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="effort"/>
    <state_interface name="status_code"/>
    <state_interface name="enabled"/>
    <state_interface name="homed"/>
    <state_interface name="fault"/>
  </joint>
</ros2_control>
```

## Runtime behavior

- Activate sends `status=0x03` (enable + clear error), then periodic `status=0x01` commands.
- Deactivate sends `status=0x00` (disable).
- Command input is `position` in meters, converted to protocol unit (`1e-6 m` per count).
- Exposes feedback: position, effort, status_code, enabled, homed, fault.
- Optional debug topics:
  - `~/gripper_command_debug` + `~/gripper_command_debug_stamped`
  - `~/gripper_sent_position_debug` + `~/gripper_sent_position_debug_stamped`
  - `~/gripper_feedback_debug` + `~/gripper_feedback_debug_stamped`

## Notes and caveats

- In most single-arm setups, only `can_interface` changes (`can0`, `can1`, ...).
- `command_can_id` / `feedback_can_id` are usually fixed defaults:
  - `command_can_id = 0x159`
  - `feedback_can_id = 0x2A8`
- If the arm was configured with master/slave offset command `0x470`, CAN IDs can shift:
  - Control base can shift `15x -> 16x/17x` (gripper command may become `0x169/0x179`)
  - Feedback base can shift `2Ax -> 2Bx/2Cx` (gripper feedback may become `0x2B8/0x2C8`)
- In offset mode, update `command_can_id` and `feedback_can_id` params accordingly.
- Quick check recommendation:
  - sniff bus first (`candump`) and confirm the real gripper TX/RX IDs
  - then align ros2_control params with observed IDs

## CI note

GitHub Actions checks out `alexzhang1030/ros_std_msgs_stamped` into `std_msgs_stamped_src` so
`std_msgs_stamped` is available during CI build.
