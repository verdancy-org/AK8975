# AK8975

AKM AK8975 SPI 3-axis magnetometer sensor module for XRobot.

This module initializes the magnetometer over a device-level SPI handle, samples
magnetic field data in a background thread, applies the configured sensor-frame
to application-frame rotation, publishes calibrated magnetic field vectors, and
provides a RamFS shell command for status output and runtime calibration.

The `ak8975_spi` handle is expected to be prepared by the User layer. If the
sensor shares a physical SPI bus with other devices, chip-select handling and bus
locking should be encapsulated in that handle.

## Required Hardware

- `ak8975_spi`
- `ramfs`

## Constructor Arguments

- `rotation`: sensor-frame to application-frame quaternion
- `data_topic_name`: default `"ak8975_mag"`
- `sample_period_ms`: default `20`
- `task_stack_depth`: default `1024`

## Published Topics

- `data_topic_name`: `Eigen::Matrix<float, 3, 1>`, magnetic field vector

## Shell Commands

The module registers `ak8975` in `RamFS`.

- `ak8975` or `ak8975 status`: print chip ID, latest magnetic field data, and calibration state
- `ak8975 cali`: collect magnetic field extrema and update runtime offset / scale calibration

## XRobot Configuration Example

```yaml
- id: mag
  name: AK8975
  constructor_args:
    rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
    data_topic_name: "ak8975_mag"
    sample_period_ms: 20
    task_stack_depth: 1024
```
