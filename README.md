# Barbell Velocity Tracker

A real-time barbell tracking system using an ESP32 and MPU6050 IMU

## Hardware
- ESP-WROOM-32 Microcontroller
- GY-521 MPU6050 IMU (Accelerometer & Gyrometer)

## Software
- Arduino IDE
- Python (numpy, matplotlib, asyncio, bleak, scipy.signal, scipy.fft, csv, datetime)


## Data Acquisition Overview
### Set MPU6050 parameters

| Parameter  | Value |
| ------------- | ------------- |
| Digital Low Pass Filter (DLPF)  | 94 Hz  |
| Sample Rate  | 100 Hz  |
| Accelerometer Range | ±4G |
| Gyroscope Range | 250° |


### Collect data
- Collect Raw 3-axis acceleration and gyroscopic readings from MPU6050
- I2C transfer from MPU6050 to ESP32
### Package and Transfer Data to Signal Processing Pipeline
- Calibrate accelerometer readings
- Timestamp data points and package into batches (5 readings)
- Bluetooth Low Energy (BLE) transmission


## Data Acquisition: Considerations and Consequences

### MPU6050 Digial Low Pass Filter (DLPF)
94 Hz captures weightlifting acceleration transients (~30-40 Hz) while still filtering noise

### Sample rate
100 Hz Nyquist Frequency, folding, integration accuracy (packet loss), BLE power + windows limitations

### Accelerometer Range 
+- 4G Powerlifting (snatch, clean, power clean) may clip, watch for clipping at 4G (~ 40 m/s^2).
  Increasing range increases effects of quantization, as MPU6050 has a 16-bit ADC, but are negligible (micro g's).
  
### Gyroscope Range
At 250 degrees, powerlifting may result in hyper rotation of the bar during catch movements that would result in gyroscope readings clipping

### Accelerometer Calibration
Offset values were found and applied by taking accelerometer readings while the MPU6050 was stationary and upright

### Bluetooth Transmission BLE & Packet batching
BLE was chosen for its easier prototyping capabilities, lower power consumption, and compatibility with Python through Bleak. 

Bleak's Notify mechanism was used to stream sensor data. Furthermore, five data points were batched into each packet to reduce the density of packet transmission. For example, At 100 Hz, instead of 100 packets sent per second, batches can be sent at 20 packets per second, reducing packet loss and easing the load on the BLE stack.

Traditional Bluetooth should be implemented in any commercial or professional use of this technology to ensure user verification, and reduced packet loss. The current BLE setup's main tradeoff comes in the unsatisfactory packet loss (~30%). Handling packet loss is discussed in the signal processing pipeline.

## Signal Processing Pipeline Overview
- Connect with ESP32 via BLE, using Bleak's BLE package
- Stream data for a set time
- Reconstruct packet data (Python)
- Filter data heavily for clean rep detection
- Apply Kalman Filter for metric analysis
- Perform metric analysis on Kalman Filtered data
- Plot relevant data analyses
- Save session to .csv


## Key Signal Processing Information

### Performing Analyses
The main performance metrics extracted from each lift are: Acceleration Velocity & Power, Forward Backward & Lateral Drift, and Pitch and Roll. 

Total acceleration is calculated through the summation of acceleration across all axes, and drift is described by the non vertical acceleration axes. Pitch and roll can help to describe forward and backward lean, and lateral lean, and are calculated through the use of the Kalman filter.

Velocity presents a problem as it is not directly measured by the MPU6050. A 

### Handling Packet Loss

### Filtering for Rep Detection

### Kalmann Filter (Sensor Fusion)


### Data Visualization

### Saving Data



## Results
