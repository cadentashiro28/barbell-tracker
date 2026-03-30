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

Velocity presents a problem as it is not directly measured by the MPU6050. An Euler-step approach is currently used to integrate the acceleration data along the vertical axis to calculate vertical veocity. Ideally, a rectangular integration step looks like
```ruby
v[i] = v[i - 1] + az[i] * dt
```
The true acceleration signal is a continuous function, knowing this, the Euler-step approach introduces error through approximation. Other approaches should be investigated, one simple change is the trapezoidal approach which aims to represent the continous nature of the signal by averaging the change in accelerations.
```ruby
v[i] = .5 * v[i - 1] + az[i] * dt
```

A key consideration of any step approach is the time step itself. Ideally dt would equal the sample rate, however, dropped packets introduce gaps in integration that need to be handled carefully.

### Handling Packet Loss
Each data point, however, contains a timestamp, giving the exact millisecond that the data point was taken. Thus, dt can be calculated as follows
```ruby
dt = t[i] - t[i - 1]
```
Recall that the acceleration signal is continuous, if many packets are lost, integration is innacurate, due to the inability to predict the behavior of the acceleration signal in the lost time. Therefore, a threshold of timeloss is established.
```ruby
if dt > .05:  # 50 milliseconds
  v[i] = v[i - 1]
  continue
```

### Filtering for Rep Detection
The raw acceleration data contains high frequency movement that is extraneous to counting repetitions. This data is important for measuring peak values around the reversal, however, for purely counting reps, only a small frequency range is needed.

First, the frequencies in the raw data were analyzed using Python's fast fourier transform. Next, high and low filter cutoffs were established to isolate the foundational frequency of the movement. Different movements displayed different frequency patterns, so the specific cutoffs for each movement pattern was stored. 

Finally, the number of reps could be counted from the filtered acceleration signal by counting the number of occurences where acceleration went from a negative peak to a positive peak. In other words, the concentric phase of motion.


### Kalmann Filter (Sensor Fusion)



### Saving Data
Each session is saved to a .csv file containing 


## Results
