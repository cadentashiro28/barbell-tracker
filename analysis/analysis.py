import numpy as np
import matplotlib.pyplot as plt
import asyncio
from bleak import BleakClient, BleakScanner
from scipy.signal import butter, filtfilt, find_peaks
from scipy.fft import fft, fftfreq
import csv
from datetime import datetime

# ─── Configuration ────────────────────────────────────────────────────────────

CHARACTERISTIC_UUID = "abcd1234-ab12-ab12-ab12-abcdef123456"
BATCH_SIZE = 5
DURATION    = 5
BAR_WEIGHT  = 135   # lbs
BODY_WEIGHT = 175   # lbs
KG_TO_LBS   = 0.453592
AZ_OFFSET   = 0
LIFT        = 'bench'

LIFT_PROFILES = {
    'squat': {
        'rep_low':    0.2,
        'rep_high':   1.0,
        'meas_low':   0.5,
        'meas_high':  15.0,
        'mass':       'bar + body',
        'zvr':        0.05,
    },
    'bench': {
        'rep_low':    0.2,
        'rep_high':   1.0,
        'meas_low':   0.5,
        'meas_high':  15.0,
        'mass':       'bar only',
        'zvr':        0.05,
    },
    'deadlift': {
        'rep_low':    0.2,
        'rep_high':   1.0,
        'meas_low':   0.5,
        'meas_high':  5.0,
        'mass':       'bar + body',
        'zvr':        0.05,
    },
}

# ─── Globals ──────────────────────────────────────────────────────────────────

ax_raw     = []
ay_raw     = []
az_raw     = []
gx_raw     = []
gy_raw     = []
gz_raw     = []
timestamps = []
buffer     = ""

# ─── BLE ──────────────────────────────────────────────────────────────────────

def handle_data(sender, data):
    global buffer
    raw = data.decode("utf-8", errors="replace")
    buffer += raw
    while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)
        line = line.strip()
        if not line:
            continue
        try:
            parts  = line.split(",")
            t_base = int(parts[0])
            values = list(map(float, parts[1:]))
            for i in range(BATCH_SIZE):
                timestamps.append(t_base + i * 10)
                ax_raw.append(values[i * 6 + 0])
                ay_raw.append(values[i * 6 + 1])
                az_raw.append(-values[i * 6 + 2])
                gx_raw.append(values[i * 6 + 3])
                gy_raw.append(values[i * 6 + 4])
                gz_raw.append(values[i * 6 + 5])
        except (ValueError, IndexError) as e:
            print(f"Bad line: {repr(line)}")
            print(f"Error: {e}")

async def main():
    ax_raw.clear()
    ay_raw.clear()
    az_raw.clear()
    gx_raw.clear()
    gy_raw.clear()
    gz_raw.clear()
    timestamps.clear()
    global buffer
    buffer = ""

    print("Scanning for ESP32-Squat...")
    device = None
    for attempt in range(5):
        device = await BleakScanner.find_device_by_name("ESP32-Squat", timeout=5.0)
        if device:
            break
        print(f"Not found, retrying ({attempt + 1}/5)...")
        await asyncio.sleep(2.0)

    if not device:
        print("Device not found")
        return

    for attempt in range(3):
        try:
            async with BleakClient(device, timeout=10.0) as client:
                await asyncio.sleep(1.5)
                print("Connected! Recording...")
                await client.start_notify(CHARACTERISTIC_UUID, handle_data)
                await asyncio.sleep(DURATION)
                await client.stop_notify(CHARACTERISTIC_UUID)
                print(f"Done. {len(az_raw)} samples collected.")
                return
        except Exception as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            ax_raw.clear()
            ay_raw.clear()
            az_raw.clear()
            timestamps.clear()
            buffer = ""
            await asyncio.sleep(2.0)

# ─── Filters ──────────────────────────────────────────────────────────────────

def highpass(data, cutoff=0.5, fs=100, order=4):
    nyq  = fs / 2
    b, a = butter(order, cutoff / nyq, btype='high')
    return filtfilt(b, a, data)

def lowpass(data, cutoff=20, fs=100, order=4):
    nyq  = fs / 2
    b, a = butter(order, cutoff / nyq, btype='low')
    return filtfilt(b, a, data)

def apply_filters(data, low, high, fs):
    data = highpass(data, cutoff=low,  fs=fs)
    data = lowpass(data,  cutoff=high, fs=fs)
    return data

def process_axis(raw, gravity=0.0, fs=100):
    profile = LIFT_PROFILES[LIFT]
    sig = np.array(raw, dtype=float)
    sig = sig - gravity
    sig = sig - np.mean(sig)
    sig = apply_filters(sig, profile['meas_low'], profile['meas_high'], fs)
    return sig

# ─── Rep Detection ────────────────────────────────────────────────────────────

def detect_reps(az, t, threshold=0.5, min_rep_duration=0.5):
    fs          = len(az) / t[-1]
    min_samples = int(min_rep_duration * fs)

    neg_peaks, _ = find_peaks(-az, height=threshold, distance=min_samples)
    pos_peaks, _ = find_peaks( az, height=threshold, distance=min_samples)

    reps     = []
    used_pos = set()

    for neg in neg_peaks:
        future_pos = [p for p in pos_peaks if p > neg and p not in used_pos]
        if not future_pos:
            continue
        pos          = future_pos[0]
        rep_duration = t[pos] - t[neg]
        if 0.3 < rep_duration < 5.0:
            reps.append({
                'start_idx':  neg,
                'end_idx':    pos,
                'start_time': t[neg],
                'end_time':   t[pos],
                'duration':   rep_duration,
            })
            used_pos.add(pos)

    return reps

# ─── Signal Processing ────────────────────────────────────────────────────────

class KalmanFilter:
    """
    Fuses accelerometer and gyroscope to estimate bar angle.
    State: [angle, gyro_bias]
    """
    def __init__(self, Q_angle=0.001, Q_bias=0.003, R_measure=0.03):
        self.Q_angle   = Q_angle
        self.Q_bias    = Q_bias
        self.R_measure = R_measure
        self.angle     = 0.0
        self.bias      = 0.0
        self.P         = [[0.0, 0.0], [0.0, 0.0]]

    def update(self, accel_angle, gyro_rate, dt):
        # Predict
        rate         = gyro_rate - self.bias
        self.angle  += dt * rate
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Update
        S            = self.P[0][0] + self.R_measure
        K            = [self.P[0][0] / S, self.P[1][0] / S]
        y            = accel_angle - self.angle
        self.angle  += K[0] * y
        self.bias   += K[1] * y
        P00          = self.P[0][0]
        P01          = self.P[0][1]
        self.P[0][0] -= K[0] * P00
        self.P[0][1] -= K[0] * P01
        self.P[1][0] -= K[1] * P00
        self.P[1][1] -= K[1] * P01

        return self.angle
    
def kalman_process(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, t):
    """
    Fuses accel + gyro using Kalman filter.
    Returns gravity-corrected acceleration for each axis.
    """
    ax = np.array(ax_raw, dtype=float)
    ay = np.array(ay_raw, dtype=float)
    az = np.array(az_raw, dtype=float)
    gx = np.array(gx_raw, dtype=float)
    gy = np.array(gy_raw, dtype=float)

    kf_pitch = KalmanFilter()
    kf_roll  = KalmanFilter()

    kf_pitch.angle = np.arctan2(ax[0], np.sqrt(ay[0]**2 + az[0]**2))
    kf_roll.angle  = np.arctan2(ay[0], np.sqrt(ax[0]**2 + az[0]**2))

    pitch    = np.zeros(len(ax))
    roll     = np.zeros(len(ax))
    pitch[0] = kf_pitch.angle
    roll[0]  = kf_roll.angle

    for i in range(1, len(ax)):
        dt = t[i] - t[i - 1]
        if dt <= 0 or dt > 0.05:
            continue

        # Angle estimates from accelerometer
        accel_pitch = np.arctan2(ax[i], np.sqrt(ay[i]**2 + az[i]**2))
        accel_roll  = np.arctan2(ay[i], np.sqrt(ax[i]**2 + az[i]**2))

        # Fuse with gyro
        pitch[i] = kf_pitch.update(accel_pitch, gy[i], dt)
        roll[i]  = kf_roll.update(accel_roll,   gx[i], dt)

    pitch = pitch - pitch[0]
    roll  = roll  - roll[0]

    # Reconstruct gravity vector from estimated angles
    g_x = np.sin(pitch)
    g_y = -np.sin(roll) * np.cos(pitch)
    g_z = np.cos(pitch) * np.cos(roll)

    # Subtract gravity from each axis
    ax_corrected = ax - (9.81 * g_x)
    ay_corrected = ay - (9.81 * g_y)
    az_corrected = az - (9.81 * g_z)

    return ax_corrected, ay_corrected, az_corrected, pitch, roll

def process_signal(ax_raw, ay_raw, az_raw):
    profile   = LIFT_PROFILES[LIFT]
    mass_type = profile['mass']
    m = (BAR_WEIGHT if mass_type == 'bar only' else BAR_WEIGHT + BODY_WEIGHT) * KG_TO_LBS

    t      = np.array(timestamps) / 1000.0
    t      = t - t[0]
    fs     = len(az_raw) / t[-1]

    print(f"\nSamples        : {len(az_raw)}")
    print(f"Duration       : {t[-1]:.2f}s")
    print(f"Avg sample rate: {fs:.1f} Hz")
    print(f"Dropped packets: {int(t[-1] * 100) - len(az_raw)}")

    # Process each axis
    ax_k, ay_k, az_k, pitch, roll = kalman_process(
        ax_raw, ay_raw, az_raw,
        gx_raw, gy_raw, gz_raw,
        t
    )

    ax = apply_filters(ax_k - np.mean(ax_k), profile['meas_low'], profile['meas_high'], fs)
    ay = apply_filters(ay_k - np.mean(ay_k), profile['meas_low'], profile['meas_high'], fs)
    az = apply_filters(az_k - np.mean(az_k), profile['meas_low'], profile['meas_high'], fs)

    # Resultant acceleration
    a = np.sqrt(ax**2 + ay**2 + az**2)

    # Velocity from vertical axis with zero velocity reset
    v = np.zeros(len(az))
    for i in range(1, len(az)):
        dt = t[i] - t[i - 1]
        if dt > 0.05:
            continue
        v[i] = v[i - 1] + az[i] * dt
        if abs(az[i]) < profile['zvr']:
            v[i] = 0.0

    # Rep detection on heavily filtered vertical axis
    az_reps = apply_filters(
        np.array(az_raw, dtype=float) - 9.81,
        profile['rep_low'],
        profile['rep_high'],
        fs
    )
    reps = detect_reps(az_reps, t)
    print(f"Reps detected  : {len(reps)}")

    # Bar path
    vx = np.cumsum(ax) * (1 / fs)
    vy = np.cumsum(ay) * (1 / fs)
    dx = np.cumsum(vx) * (1 / fs)
    dy = np.cumsum(vy) * (1 / fs)
    print(f"Max fwd/back drift : {np.max(np.abs(dx)):.3f} m")
    print(f"Max lateral drift  : {np.max(np.abs(dy)):.3f} m")

    return t, ax, ay, az, a, v, m, reps, pitch, roll

# ─── Metrics ──────────────────────────────────────────────────────────────────

def get_metrics(a, v, m, reps):
    peak_a = np.max(np.abs(a))
    peak_v = np.max(np.abs(v))
    peak_p = m * peak_a * peak_v

    print(f"\nMax Acceleration : {peak_a:.3f} m/s²")
    print(f"Max Velocity     : {peak_v:.3f} m/s")
    print(f"Est. Peak Power  : {peak_p:.1f} W")

    for i, rep in enumerate(reps):
        s, e = rep['start_idx'], rep['end_idx']
        print(f"\nRep {i + 1}:")
        print(f"  Duration   : {rep['duration']:.2f}s")
        print(f"  Peak Accel : {np.max(np.abs(a[s:e])):.3f} m/s²")
        print(f"  Peak Vel   : {np.max(np.abs(v[s:e])):.3f} m/s")
        print(f"  Peak Power : {np.max(np.abs(v[s:e] * a[s:e] * m)):.1f} W")

# ─── Display Data ────────────────────────────────────────────────────────────────────

def plot_avp(t, a, v, m, reps):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    plots = [
        (a,       'Acceleration (m/s²)', 'steelblue'),
        (v,       'Velocity (m/s)',       'orange'),
        (v*a*m,   'Power (W)',            'green'),
    ]
    for ax, (signal, label, color) in zip(axes, plots):
        ax.plot(t, signal, color=color, linewidth=0.8)
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.set_ylabel(label)
        ax.set_xlabel("Time (s)")
        for rep in reps:
            ax.axvspan(rep['start_time'], rep['end_time'], alpha=0.15, color='gold')
    axes[0].set_title(f"{LIFT.capitalize()} — {len(reps)} reps")
    plt.tight_layout()
    plt.show()

def plot_multiaxis(t, ax, ay, az, reps):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    plots = [
        (az, 'Vertical (m/s²)',       'steelblue'),
        (ax, 'Fwd/Back lean (m/s²)',  'tomato'),
        (ay, 'Lateral drift (m/s²)',  'mediumpurple'),
    ]
    for axis, (signal, label, color) in zip(axes, plots):
        axis.plot(t, signal, color=color, linewidth=0.8)
        axis.axhline(0, color='gray', linewidth=0.5)
        axis.set_ylabel(label)
        axis.set_xlabel("Time (s)")
        for rep in reps:
            axis.axvspan(rep['start_time'], rep['end_time'], alpha=0.15, color='gold')
    axes[0].set_title("Multi-Axis Analysis")
    plt.tight_layout()
    plt.show()

def plot_fft(signal, fs, title="Frequency Spectrum"):
    N  = len(signal)
    yf = np.abs(fft(signal))[:N // 2]
    xf = fftfreq(N, 1 / fs)[:N // 2]
    plt.figure(figsize=(10, 4))
    plt.plot(xf, yf)
    plt.xlim(0, 10)
    plt.title(title)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_angles(t, pitch, roll, reps):
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    plots = [
        (np.degrees(pitch), 'Pitch — Fwd/Back (°)', 'tomato'),
        (np.degrees(roll),  'Roll — Lateral (°)',   'mediumpurple'),
    ]
    for ax, (signal, label, color) in zip(axes, plots):
        ax.plot(t, signal, color=color, linewidth=0.8)
        ax.axhline(0, color='gray', linewidth=0.5)
        ax.set_ylabel(label)
        ax.set_xlabel("Time (s)")
        for rep in reps:
            ax.axvspan(rep['start_time'], rep['end_time'], alpha=0.15, color='gold')
    axes[0].set_title("Bar Angle (Kalman Filtered)")
    plt.tight_layout()
    plt.show()

def save_session(reps, a, v, m):
    filename = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    if not reps:
        print("No reps to save")
        return

    baseline_vel = np.max(np.abs(v[reps[0]['start_idx']:reps[0]['end_idx']]))

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['rep', 'duration', 'peak_accel', 'peak_vel', 'peak_power', 'velocity_loss_%'])

        for i, rep in enumerate(reps):
            s, e = rep['start_idx'], rep['end_idx']

            peak_a   = np.max(np.abs(a[s:e]))
            peak_v   = np.max(np.abs(v[s:e]))
            peak_p   = np.max(np.abs(v[s:e] * a[s:e] * m))
            vel_loss = (baseline_vel - peak_v) / baseline_vel * 100

            writer.writerow([
                i + 1,
                round(rep['duration'], 2),
                round(peak_a, 3),
                round(peak_v, 3),
                round(peak_p, 1),
                round(vel_loss, 1)
            ])

    print(f"Session saved to {filename}")
# ─── Main ─────────────────────────────────────────────────────────────────────

asyncio.run(main())

if len(az_raw) == 0:
    print("No data received.")
    exit()

t, ax, ay, az, a, v, m, reps, pitch, roll = process_signal(ax_raw, ay_raw, az_raw)
get_metrics(a, v, m, reps)
plot_avp(t, a, v, m, reps)
plot_multiaxis(t, ax, ay, az, reps)
plot_angles(t, pitch, roll, reps)
save_session(reps, a, v, m)

# plot_fft(az, fs=len(az)/t[-1], title="Filtered Vertical Spectrum")