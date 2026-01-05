# Invensense ICM42688 6-Axis IMU Driver for Zephyr RTOS

## Overview

This is a comprehensive driver for the **Invensense ICM42688** 6-axis MEMS accelerometer and gyroscope sensor. The driver supports:

- **Communication**: I2C, SPI, and I3C interfaces
- **Accel/Gyro Ranges**: Fully configurable via device tree
- **Sampling Rates**: 8 Hz to 8 kHz (configurable per axis)
- **Advanced Features**:
  - FIFO with packet parsing and optional message queue
  - APEX motion detection (tap, pedometer, tilt, significant motion)
  - Wake-on-Motion (WOM) detection
  - Interrupt triggers (data-ready, motion events)
  - Digital notch filter and anti-aliasing filter
  - Temperature sensor support
  - Offset calibration

## Hardware Requirements

- **Sensor**: Invensense ICM42688
- **Power**: 3.3V (logic and I/O)
- **Communication**: I2C (400 kHz recommended) or SPI (up to 10 MHz)
- **Optional**: INT1 GPIO pin for interrupt support

### Pinout Example (I2C)

```
ICM42688        nRF54L15DK
========================================
VCC         --> 3V3
GND         --> GND
SCL         --> P0.12 (i2c0_scl)
SDA         --> P0.11 (i2c0_sda)
INT1        --> P0.10 (gpio interrupt, optional)
```

## Configuration

### Kconfig Options

Enable in `prj.conf`:

```kconfig
# Basic driver
CONFIG_SENSOR=y
CONFIG_I2C=y
CONFIG_ICM42688=y

# Optional features
CONFIG_ICM42688_TRIGGER=y       # Enable interrupt triggers
CONFIG_ICM42688_APEX=y          # Enable APEX motion detection
CONFIG_ICM42688_FIFO=y          # Enable FIFO
CONFIG_ICM42688_NOTCH_FILTER=y  # Enable notch filter
CONFIG_ICM42688_AAF_FILTER=y    # Enable anti-alias filter
```

### Device Tree Configuration

Add to your board overlay (e.g., `nrf54l15dk_nrf54l15_cpuapp.overlay`):

```dts
&i2c0 {
	status = "okay";
	
	icm42688: imu@68 {
		compatible = "invensense,icm42688";
		status = "okay";
		reg = <0x68>;
		
		/* Interrupt pin (optional) */
		interrupt-gpios = <&gpio0 10 GPIO_ACTIVE_HIGH>;
		
		/* Sensor ranges */
		gyro-range = <2000>;    /* ±2000 deg/s (or 125/250/500/1000) */
		accel-range = <16>;     /* ±16 g (or 2/4/8) */
		
		/* Output data rates */
		gyro-odr = <200>;       /* Hz */
		accel-odr = <200>;      /* Hz */
	};
};
```

## Usage Example

### Basic Sampling

```c
#include <zephyr/drivers/sensor.h>

int main(void)
{
	/* Get device binding */
	const struct device *imu = DEVICE_DT_GET_ONE(invensense_icm42688);
	if (!device_is_ready(imu)) {
		printk("IMU device not ready\n");
		return -ENODEV;
	}
	
	struct sensor_value accel[3], gyro[3], temp;
	
	while (1) {
		/* Read all sensors */
		sensor_sample_fetch(imu);
		
		/* Get accelerometer data (m/s^2) */
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel);
		
		/* Get gyroscope data (rad/s) */
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro);
		
		/* Get temperature (°C) */
		sensor_channel_get(imu, SENSOR_CHAN_DIE_TEMP, &temp);
		
		printk("Accel: %d.%06d %d.%06d %d.%06d m/s^2\n",
		       accel[0].val1, accel[0].val2,
		       accel[1].val1, accel[1].val2,
		       accel[2].val1, accel[2].val2);
		
		printk("Gyro: %d.%06d %d.%06d %d.%06d rad/s\n",
		       gyro[0].val1, gyro[0].val2,
		       gyro[1].val1, gyro[1].val2,
		       gyro[2].val1, gyro[2].val2);
		
		k_sleep(K_MSEC(10));
	}
	
	return 0;
}
```

### Data-Ready Trigger

```c
static void imu_trigger_handler(const struct device *dev,
                                 const struct sensor_trigger *trigger)
{
	struct sensor_value accel[3], gyro[3];
	
	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	
	printk("Data ready interrupt fired!\n");
}

int main(void)
{
	const struct device *imu = DEVICE_DT_GET_ONE(invensense_icm42688);
	
	struct sensor_trigger trig;
	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	
	sensor_trigger_set(imu, &trig, imu_trigger_handler);
	
	while (1) {
		k_sleep(K_FOREVER);
	}
	
	return 0;
}
```

### Wake-on-Motion

```c
int main(void)
{
	const struct device *imu = DEVICE_DT_GET_ONE(invensense_icm42688);
	struct sensor_value val;
	
	/* Enable WOM detection */
	val.val1 = 1;  /* enable */
	sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, 
	               ICM42688_ATTR_WOM_ENABLE, &val);
	
	/* Set threshold (in mg) */
	val.val1 = 50;  /* 50 mg */
	sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
	               ICM42688_ATTR_WOM_THRESHOLD, &val);
	
	return 0;
}
```

## Architecture

### Module Structure

```
drivers/sensor/icm42688/
├── icm42688.c              # Main driver (init, read/write, attr get/set)
├── icm42688.h              # Public API and data structures
├── icm42688_reg.h          # Register definitions
├── icm42688_trigger.c      # Interrupt/trigger handling
├── icm42688_fifo.c         # FIFO packet parsing
├── icm42688_fifo_queue.c   # Message queue for FIFO
├── icm42688_apex.c         # APEX motion detection
├── icm42688_notch.c        # Digital notch filter
├── icm42688_aaf.c          # Anti-aliasing filter
├── icm42688_aaf_table.h    # AAF lookup table
├── Kconfig                 # Configuration options
└── CMakeLists.txt          # Build rules
```

### Key Data Structures

**`icm42688_config`** (device tree, read-only):
- Bus type and device handles (I2C/SPI/I3C)
- GPIO pin for interrupt
- Initial sensor ranges and ODRs

**`icm42688_data`** (runtime state):
- Sensor measurements (accel, gyro, temperature)
- FIFO and trigger state
- Configuration flags (WOM, APEX, filters enabled)
- Calibration offsets
- Mutex for thread safety

## Supported Channels

| Channel | Type | Units | Notes |
|---------|------|-------|-------|
| `SENSOR_CHAN_ACCEL_XYZ` | Acceleration | m/s² | 3-axis |
| `SENSOR_CHAN_GYRO_XYZ` | Angular velocity | rad/s | 3-axis |
| `SENSOR_CHAN_DIE_TEMP` | Temperature | °C | Internal sensor |

## Supported Attributes (Partial List)

| Attribute | Type | Values | Notes |
|-----------|------|--------|-------|
| `SENSOR_ATTR_SAMPLING_FREQUENCY` | Read/Write | 8-8000 Hz | ODR |
| `SENSOR_ATTR_FULL_SCALE` | Read/Write | 2/4/8/16 (accel) | Range in g |
| `SENSOR_ATTR_FULL_SCALE` | Read/Write | 125/250/500/1000/2000 (gyro) | Range in deg/s |
| `ICM42688_ATTR_WOM_ENABLE` | Read/Write | 0/1 | Enable/disable |
| `ICM42688_ATTR_WOM_THRESHOLD` | Read/Write | 0-255 | mg |
| `SENSOR_TRIG_DATA_READY` | Trigger | - | INT pin required |

## Limitations

- **I3C support** requires CONFIG_I3C enabled
- **FIFO** can hold up to 2080 bytes (~416 packets at 16-bit depth)
- **Temperature sensor** is internal to ICM42688; use with caution (limited accuracy)
- **Self-test** can be noisy; best done with sensor stationary and stable

## Debugging

Enable logging:

```kconfig
CONFIG_LOG=y
CONFIG_SENSOR_LOG_LEVEL_DBG=y
```

Check module state with:

```c
const struct device *imu = DEVICE_DT_GET_ONE(invensense_icm42688);
const struct icm42688_data *data = imu->data;
printk("FIFO enabled: %d, APEX enabled: %d\n", 
       data->fifo_enabled, data->apex_enabled);
```

## References

- [ICM42688 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-2/)
- [Zephyr Sensor Driver Framework](https://docs.zephyrproject.org/latest/reference/drivers/sensor.html)
- [Zephyr Device Tree Guide](https://docs.zephyrproject.org/latest/build/dts/index.html)

## License

Apache License 2.0
