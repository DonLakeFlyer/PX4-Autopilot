# Pose Hold Plugin

World plugin for holding a simulated vehicle in a chosen attitude, rotating it, and releasing it again.
Used to exercise the QGroundControl sensor calibration UI (accelerometer and compass) in SITL, where nothing else can rotate the vehicle.

It is loaded from `server.config` and listens for `gz.msgs.StringMsg` commands on `/pose_hold`.

## Commands

| Command | Effect |
|---|---|
| `level`, `nose_down`, `tail_down`, `left_down`, `right_down`, `inverted` | Hold the vehicle in that attitude, keeping its heading, at least 2 m above the ground. It stays there until the next command. |
| `rotate` | Rotate the current pose about the vertical axis at 45 deg/s until the next command. |
| `release` | Return the vehicle to normal physics. |

An optional leading model name selects the vehicle, for example `x500_0 nose_down`.
Without it, the most recently addressed model is used, or else the first non-static model in the world.

```bash
src/modules/simulation/gz_plugins/pose_hold/pose_hold.sh nose_down
src/modules/simulation/gz_plugins/pose_hold/pose_hold.sh rotate
src/modules/simulation/gz_plugins/pose_hold/pose_hold.sh release
```

`pose_hold.sh` wraps `gz topic -t /pose_hold -m gz.msgs.StringMsg -p 'data: "<command>"'`.

The same commands work from the PX4 console, addressed to the vehicle that instance is attached to:

```
pxh> gz_bridge pose nose_down
pxh> gz_bridge pose rotate
pxh> gz_bridge pose release
```

## Behaviour

- Held attitudes make the vehicle static, so the accelerometer reads a steady gravity vector for the orientation detection of the calibration.
- Rotating keeps the vehicle dynamic with its velocity commanded each step, so the gyro reports the rotation, but the accelerometer reads about zero.
  Hold an attitude first, then send `rotate`.
- While rotating steadily and disarmed for several seconds, PX4's online `gyro_calibration` module learns the rotation rate as a gyro offset and saves it to `CAL_GYRO*_*OFF`. It re-learns a zero offset a few seconds after `release`, or set `IMU_GYRO_CAL_EN` to 0 first.
- In Gazebo SITL the magnetometer comes from the gz sensor, so the field follows the vehicle attitude and a compass calibration sees real rotation.
