# Lower Layer Controller Telemetry Update

## Current Telemetry Format

Based on the analysis of your lower layer controller, the current telemetry format is:

```c
// Current format (12 fields, space-separated)
snprintf(line, sizeof(line),
         "%3.2f %3.2f %3.2f %10ld %10ld %u %u %u %u %u %u %u\r\n", 
         h, r, p,                    // Yaw Roll Pitch (degrees)
         (long)encL, (long)encR,     // Encoder counts
         vbat_1, vbat_2,             // Battery voltages (mV)
         cliffL, cliffC, cliffR,     // Cliff sensors
         emerg, profileDone);        // Status flags
```

## Proposed Extended Format

To support the EKF sensor fusion, we need to add accelerometer and gyroscope data:

```c
// Extended format (15 fields, space-separated)
snprintf(line, sizeof(line),
         "%3.2f %3.2f %3.2f %10ld %10ld %u %u %u %u %u %u %u %6.3f %6.3f %7.4f\r\n", 
         h, r, p,                    // Yaw Roll Pitch (degrees)
         (long)encL, (long)encR,     // Encoder counts
         vbat_1, vbat_2,             // Battery voltages (mV)
         cliffL, cliffC, cliffR,     // Cliff sensors
         emerg, profileDone,         // Status flags
         accelX, accelY,             // Accelerometer X,Y (m/s²)
         gyroZ);                     // Gyroscope Z (rad/s)
```

## Required Changes to AVR Controller

### 1. Update `send_telemetry()` function in `main.c`

```c
static void send_telemetry(bool emerg, bool profileDone)
{
    char line[120];  // Increased buffer size for additional data

    /* ---------- IMU ---------- */
    int16_t h16, r16, p16;
    bno055_get_euler(&h16, &r16, &p16);

    float h = h16 / 16.0f;
    float r = r16 / 16.0f;
    float p = p16 / 16.0f;

    /* ---------- NEW: Get accelerometer and gyroscope data ---------- */
    int16_t ax16, ay16, az16;
    int16_t gx16, gy16, gz16;
    
    // Get accelerometer data (VECTOR_ACCELEROMETER)
    bno055_get_accel(&ax16, &ay16, &az16);
    float accelX = ax16 / 100.0f;  // Convert to m/s²
    float accelY = ay16 / 100.0f;  // Convert to m/s²
    
    // Get gyroscope data (VECTOR_GYROSCOPE) 
    bno055_get_gyro(&gx16, &gy16, &gz16);
    float gyroZ = gz16 / 16.0f;    // Convert to rad/s (BNO055 gyro is 1dps = 16 LSB)

    /* ---------- ADC ---------- */
    uint16_t vbat_1 = analog_get_battery_1_mV();
    uint16_t vbat_2 = analog_get_battery_2_mV();
    uint16_t cliffL = analog_get_cliff_left();
    uint16_t cliffC = analog_get_cliff_front();
    uint16_t cliffR = analog_get_cliff_right();

    /* ---------- Encoders ---------- */
    int32_t encL = encoder_get_left();
    int32_t encR = encoder_get_right();

    /* ---------- Format & ship ---------- */
    snprintf(line, sizeof(line),
             "%3.2f %3.2f %3.2f %10ld %10ld %u %u %u %u %u %u %u %6.3f %6.3f %7.4f\r\n", 
             h, r, p, (long)encL, (long)encR, vbat_1, vbat_2, 
             cliffL, cliffC, cliffR, emerg, profileDone,
             accelX, accelY, gyroZ);

    usb_send_ram(line);
    m_usb_tx_push();
}
```

### 2. Add BNO055 accelerometer/gyroscope functions to `bno055_ll.c`

```c
bool bno055_get_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    if (!bno055_read(0x08, buf, 6))  // ACCEL_DATA_X_LSB_ADDR
        return false;
    
    *ax = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    *ay = (int16_t)(buf[2] | ((uint16_t)buf[3] << 8));
    *az = (int16_t)(buf[4] | ((uint16_t)buf[5] << 8));
    return true;
}

// Note: bno055_get_gyro() already exists in bno055_ll.c
```

### 3. Add function declaration to `bno055_ll.h`

```c
bool bno055_get_accel(int16_t *ax, int16_t *ay, int16_t *az);
```

## BNO055 Data Scaling Reference

From the BNO055 datasheet and Adafruit library analysis:

- **Accelerometer**: 1 m/s² = 100 LSB
- **Gyroscope**: 1 rad/s = 16 LSB (when gyro is configured for rad/s output)
- **Euler angles**: 1 degree = 16 LSB

## Backward Compatibility

The EKF integration code is designed to be backward compatible:

1. **Jetson side**: The parser will attempt to read the additional fields but won't fail if they're not present
2. **Testing**: You can test the EKF with existing telemetry (without accel/gyro) - it will just use encoder + IMU yaw
3. **Gradual upgrade**: Update the Jetson code first, then the AVR controller later

## Testing Strategy

1. **Phase 1**: Test EKF with current telemetry format (encoder + IMU yaw only)
2. **Phase 2**: Update AVR controller to include accelerometer/gyroscope data
3. **Phase 3**: Verify improved EKF performance with full sensor suite

## Expected Benefits

- **Better motion estimation** during acceleration/deceleration
- **Improved heading accuracy** with gyroscope fusion
- **Zero velocity updates** for drift reduction when stationary
- **More robust localization** especially during complex maneuvers

## Implementation Notes

- The accelerometer data enables Zero Velocity Update (ZUPT) when the robot is stationary
- Gyroscope provides high-frequency angular rate measurements to complement IMU yaw
- All sensor fusion happens on the Jetson side - AVR just provides raw sensor data
- The EKF automatically handles different sensor update rates and noise characteristics
