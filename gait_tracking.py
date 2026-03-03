from dataclasses import dataclass
from matplotlib import animation
from scipy.interpolate import interp1d
import imufusion
import matplotlib.pyplot as pyplot
import numpy

# =========================
# Load sensor data (CSV)
# Expected columns:
# 0 Time (s)
# 1..3 Gyro X/Y/Z (deg/s)
# 4..6 Accel X/Y/Z (g)
# =========================
data = numpy.genfromtxt(
    r"C:\Users\david\Documents\Proyectos\Gait-Tracking-main\short_walk.csv",
    delimiter=",",
    skip_header=1
)

timestamp = data[:, 0].astype(float)
gyroscope = data[:, 1:4].astype(float)       # deg/s
accelerometer = data[:, 4:7].astype(float)   # g

# =========================
# Estimate sample rate from timestamps
# =========================
dt_raw = numpy.diff(timestamp)
dt_raw = numpy.clip(dt_raw, 1e-4, None)      # avoid zeros / negatives
sample_rate = 1.0 / numpy.median(dt_raw)

# Delta time per sample (seconds)
delta_time = numpy.diff(timestamp, prepend=timestamp[0])
delta_time[0] = 1.0 / sample_rate

print(f"Estimated sample_rate: {sample_rate:.2f} Hz")

# =========================
# Convert units for imufusion
# Gyro: deg/s -> rad/s
# Acc : g -> m/s^2
# =========================
gyroscope = numpy.deg2rad(gyroscope)
accelerometer = accelerometer * 9.80665

# =========================
# Plot raw sensor data (converted or original?)
# Here we plot ORIGINAL semantics but with updated units:
# - Gyro is now rad/s
# - Accel is now m/s^2
# =========================
figure, axes = pyplot.subplots(
    nrows=6, sharex=True,
    gridspec_kw={"height_ratios": [6, 6, 6, 2, 1, 1]}
)
figure.suptitle("Sensor data, Euler angles, and AHRS internal states")

axes[0].plot(timestamp, gyroscope[:, 0], "tab:red", label="Gyroscope X")
axes[0].plot(timestamp, gyroscope[:, 1], "tab:green", label="Gyroscope Y")
axes[0].plot(timestamp, gyroscope[:, 2], "tab:blue", label="Gyroscope Z")
axes[0].set_ylabel("rad/s")
axes[0].grid()
axes[0].legend()

axes[1].plot(timestamp, accelerometer[:, 0], "tab:red", label="Accelerometer X")
axes[1].plot(timestamp, accelerometer[:, 1], "tab:green", label="Accelerometer Y")
axes[1].plot(timestamp, accelerometer[:, 2], "tab:blue", label="Accelerometer Z")
axes[1].set_ylabel("m/s²")
axes[1].grid()
axes[1].legend()

# =========================
# Instantiate AHRS algorithms
# =========================
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

# NOTE: gyro_range in Settings is typically in deg/s as a sensor spec (±2000 dps),
# even if gyro input is rad/s. We keep 2000 to match your MPU6050 full-scale.
ahrs.settings = imufusion.Settings(
    imufusion.CONVENTION_NWU,
    0.5,            # gain
    2000,           # gyroscope range (deg/s spec)
    10,             # acceleration rejection
    0,              # magnetic rejection
    int(5 * sample_rate)  # rejection timeout ~ 5 seconds
)

# =========================
# Process sensor data
# =========================
euler = numpy.empty((len(timestamp), 3))
internal_states = numpy.empty((len(timestamp), 3))

# Store raw earth_acceleration from library (unit may be g or m/s² depending on implementation)
earth_acc_raw = numpy.empty((len(timestamp), 3))

for index in range(len(timestamp)):
    # Offset correct gyro (expects gyro in same units you use for update; here rad/s)
    gyroscope[index] = offset.update(gyroscope[index])

    # Update AHRS
    ahrs.update_no_magnetometer(gyroscope[index], accelerometer[index], delta_time[index])

    # Euler (degrees)
    euler[index] = ahrs.quaternion.to_euler()

    # Internal states
    s = ahrs.internal_states
    internal_states[index] = numpy.array([
        s.acceleration_error,
        s.accelerometer_ignored,
        s.acceleration_recovery_trigger
    ])

    # Earth-frame acceleration (as reported by library)
    earth_acc_raw[index] = ahrs.earth_acceleration

# =========================
# Decide unit of earth_acc_raw (avoid double-scaling)
# Heuristic:
# - If typical magnitude is small (e.g., < ~3), treat as "g" and scale by 9.80665
# - If it's already around m/s² scale during motion, magnitude can be > 3 frequently.
# This is a heuristic; you can force it by setting EARTH_IS_G = True/False.
# =========================
EARTH_IS_G = None  # set True or False to force, or leave None for auto

norm_med = numpy.median(numpy.linalg.norm(earth_acc_raw, axis=1))

if EARTH_IS_G is None:
    # If median magnitude is small, likely "g"
    EARTH_IS_G = (norm_med < 3.0)

print(f"earth_acceleration median norm = {norm_med:.3f} -> treating as {'g' if EARTH_IS_G else 'm/s²'}")

if EARTH_IS_G:
    acceleration = 9.80665 * earth_acc_raw
else:
    acceleration = earth_acc_raw

# =========================
# Plot Euler angles
# =========================
axes[2].plot(timestamp, euler[:, 0], "tab:red", label="Roll")
axes[2].plot(timestamp, euler[:, 1], "tab:green", label="Pitch")
axes[2].plot(timestamp, euler[:, 2], "tab:blue", label="Yaw")
axes[2].set_ylabel("Degrees")
axes[2].grid()
axes[2].legend()

# Plot internal states
axes[3].plot(timestamp, internal_states[:, 0], "tab:olive", label="Acceleration error")
axes[3].set_ylabel("Degrees")
axes[3].grid()
axes[3].legend()

axes[4].plot(timestamp, internal_states[:, 1], "tab:cyan", label="Accelerometer ignored")
pyplot.sca(axes[4])
pyplot.yticks([0, 1], ["False", "True"])
axes[4].grid()
axes[4].legend()

axes[5].plot(timestamp, internal_states[:, 2], "tab:orange", label="Acceleration recovery trigger")
axes[5].set_xlabel("Seconds")
axes[5].grid()
axes[5].legend()

# =========================
# Plot acceleration, moving periods, velocity, position
# =========================
_, axes2 = pyplot.subplots(nrows=4, sharex=True, gridspec_kw={"height_ratios": [6, 1, 6, 6]})

axes2[0].plot(timestamp, acceleration[:, 0], "tab:red", label="X")
axes2[0].plot(timestamp, acceleration[:, 1], "tab:green", label="Y")
axes2[0].plot(timestamp, acceleration[:, 2], "tab:blue", label="Z")
axes2[0].set_title("Earth Acceleration")
axes2[0].set_ylabel("m/s²")
axes2[0].grid()
axes2[0].legend()

# Identify moving periods
is_moving = numpy.empty(len(timestamp), dtype=bool)

for index in range(len(timestamp)):
    is_moving[index] = numpy.linalg.norm(acceleration[index]) > 3.0  # threshold = 3 m/s²

margin = int(0.1 * sample_rate)  # 100 ms

for index in range(len(timestamp) - margin):
    is_moving[index] = any(is_moving[index:(index + margin)])  # add leading margin

for index in range(len(timestamp) - 1, margin, -1):
    is_moving[index] = any(is_moving[(index - margin):index])  # add trailing margin

axes2[1].plot(timestamp, is_moving.astype(int), "tab:cyan", label="Is moving")
pyplot.sca(axes2[1])
pyplot.yticks([0, 1], ["False", "True"])
axes2[1].grid()
axes2[1].legend()

# Calculate velocity (includes integral drift)
velocity = numpy.zeros((len(timestamp), 3))

for index in range(len(timestamp)):
    if is_moving[index]:  # only integrate if moving
        velocity[index] = velocity[index - 1] + delta_time[index] * acceleration[index]
    else:
        velocity[index] = velocity[index - 1]

# Find start and stop indices of each moving period
is_moving_diff = numpy.diff(is_moving.astype(int), append=is_moving[-1].astype(int))

@dataclass
class IsMovingPeriod:
    start_index: int = -1
    stop_index: int = -1

is_moving_periods = []
is_moving_period = IsMovingPeriod()

for index in range(len(timestamp)):
    if is_moving_period.start_index == -1:
        if is_moving_diff[index] == 1:
            is_moving_period.start_index = index
    elif is_moving_period.stop_index == -1:
        if is_moving_diff[index] == -1:
            is_moving_period.stop_index = index
            is_moving_periods.append(is_moving_period)
            is_moving_period = IsMovingPeriod()

# Remove integral drift from velocity
velocity_drift = numpy.zeros((len(timestamp), 3))

for p in is_moving_periods:
    start_index = p.start_index
    stop_index = p.stop_index

    t = [timestamp[start_index], timestamp[stop_index]]
    x = [velocity[start_index, 0], velocity[stop_index, 0]]
    y = [velocity[start_index, 1], velocity[stop_index, 1]]
    z = [velocity[start_index, 2], velocity[stop_index, 2]]

    t_new = timestamp[start_index:(stop_index + 1)]

    velocity_drift[start_index:(stop_index + 1), 0] = interp1d(t, x)(t_new)
    velocity_drift[start_index:(stop_index + 1), 1] = interp1d(t, y)(t_new)
    velocity_drift[start_index:(stop_index + 1), 2] = interp1d(t, z)(t_new)

velocity = velocity - velocity_drift

# Plot velocity
axes2[2].plot(timestamp, velocity[:, 0], "tab:red", label="X")
axes2[2].plot(timestamp, velocity[:, 1], "tab:green", label="Y")
axes2[2].plot(timestamp, velocity[:, 2], "tab:blue", label="Z")
axes2[2].set_title("Velocity")
axes2[2].set_ylabel("m/s")
axes2[2].grid()
axes2[2].legend()

# Calculate position
position = numpy.zeros((len(timestamp), 3))

for index in range(len(timestamp)):
    position[index] = position[index - 1] + delta_time[index] * velocity[index]

# Plot position
axes2[3].plot(timestamp, position[:, 0], "tab:red", label="X")
axes2[3].plot(timestamp, position[:, 1], "tab:green", label="Y")
axes2[3].plot(timestamp, position[:, 2], "tab:blue", label="Z")
axes2[3].set_title("Position")
axes2[3].set_xlabel("Seconds")
axes2[3].set_ylabel("m")
axes2[3].grid()
axes2[3].legend()

# Print error as distance between start and final positions
print("Error: " + "{:.3f}".format(numpy.sqrt(position[-1].dot(position[-1]))) + " m")

# =========================
# Create 3D animation
# =========================
if True:
    figure3d = pyplot.figure(figsize=(10, 10))
    ax3d = pyplot.axes(projection="3d")
    ax3d.set_xlabel("m")
    ax3d.set_ylabel("m")
    ax3d.set_zlabel("m")

    x, y, z = [], [], []
    scatter = ax3d.scatter(x, y, z)

    fps = 30
    samples_per_frame = max(1, int(sample_rate / fps))

    def update(frame):
        index = frame * samples_per_frame
        if index >= len(timestamp):
            index = len(timestamp) - 1

        ax3d.set_title("{:.3f}".format(timestamp[index]) + " s")

        x.append(position[index, 0])
        y.append(position[index, 1])
        z.append(position[index, 2])

        scatter._offsets3d = (x, y, z)

        if (min(x) != max(x)) and (min(y) != max(y)) and (min(z) != max(z)):
            ax3d.set_xlim3d(min(x), max(x))
            ax3d.set_ylim3d(min(y), max(y))
            ax3d.set_zlim3d(min(z), max(z))
            ax3d.set_box_aspect((numpy.ptp(x), numpy.ptp(y), numpy.ptp(z)))

        return scatter

    anim = animation.FuncAnimation(
        figure3d,
        update,
        frames=int(len(timestamp) / samples_per_frame),
        interval=1000 / fps,
        repeat=False
    )

    anim.save("animation.gif", writer=animation.PillowWriter(fps))

pyplot.show()
