import pandas as pd
import matplotlib.pyplot as plt
import numpy
import glob
import os
import math
import matplotlib.ticker as ticker


from core import ULog


def convert_ulog2csv(ulog_file_name, messages, output, delimiter):
    """
    Coverts and ULog file to a CSV file.
    :param ulog_file_name: The ULog filename to open and read
    :param messages: A list of message names
    :param output: Output file path
    :param delimiter: CSV delimiter
    :return: None
    """

    msg_filter = messages.split(',') if messages else None

    ulog = ULog(ulog_file_name, msg_filter)
    data = ulog.data_list

    output_file_prefix = ulog_file_name
    # strip '.ulg'
    if output_file_prefix.lower().endswith('.ulg'):
        output_file_prefix = output_file_prefix[:-4]

    # write to different output path?
    if output:
        base_name = os.path.basename(output_file_prefix)
        output_file_prefix = os.path.join(output, base_name)

    for d in data:
        fmt = '{0}_{1}_{2}.csv'
        output_file_name = fmt.format(output_file_prefix, d.name, d.multi_id)
        fmt = 'Writing {0} ({1} data points)'
        # print(fmt.format(output_file_name, len(d.data['timestamp'])))
        with open(output_file_name, 'w') as csvfile:

            # use same field order as in the log, except for the timestamp
            data_keys = [f.field_name for f in d.field_data]
            data_keys.remove('timestamp')
            data_keys.insert(0, 'timestamp')  # we want timestamp at first position

            # we don't use np.savetxt, because we have multiple arrays with
            # potentially different data types. However the following is quite
            # slow...

            # write the header
            csvfile.write(delimiter.join(data_keys) + '\n')

            # write the data
            last_elem = len(data_keys)-1
            for i in range(len(d.data['timestamp'])):
                for k in range(len(data_keys)):
                    csvfile.write(str(d.data[data_keys[k]][i]))
                    if k != last_elem:
                        csvfile.write(delimiter)
                csvfile.write('\n')


script_dir = os.path.dirname(os.path.abspath(__file__))
list_of_files = glob.glob(script_dir + '/build/posix_sitl_default/logs/*')  # * means all if need specific format then *.csv
latest_file = max(list_of_files, key=os.path.getctime)
list_of_files = glob.glob(latest_file + '/*.ulg')
latest_file = max(list_of_files, key=os.path.getctime)
print(os.path.splitext(latest_file)[0])

convert_ulog2csv(latest_file, 'vehicle_local_position,vehicle_local_position_groundtruth,sensor_combined,extended_kalman,actuator_outputs,vehicle_attitude,vehicle_gps_position,vehicle_attitude_groundtruth,exogenous_kalman,actuator_controls_0', False, ',')

pos = pd.read_csv(os.path.splitext(latest_file)[0] + '_vehicle_local_position_0.csv')
truepos = pd.read_csv(os.path.splitext(latest_file)[0] + '_vehicle_local_position_groundtruth_0.csv')
kalman = pd.read_csv(os.path.splitext(latest_file)[0] + '_extended_kalman_0.csv')
actuators = pd.read_csv(os.path.splitext(latest_file)[0] + '_actuator_outputs_0.csv')
controls = pd.read_csv(os.path.splitext(latest_file)[0] + '_actuator_controls_0_0.csv')
trueatt = pd.read_csv(os.path.splitext(latest_file)[0] + '_vehicle_attitude_groundtruth_0.csv')
exogenous = pd.read_csv(os.path.splitext(latest_file)[0] + '_exogenous_kalman_0.csv')
attitude = pd.read_csv(os.path.splitext(latest_file)[0] + '_vehicle_attitude_0.csv')
sensors = pd.read_csv(os.path.splitext(latest_file)[0] + '_sensor_combined_0.csv')


q0 = trueatt['q[0]']
q1 = trueatt['q[1]']
q2 = trueatt['q[2]']
q3 = trueatt['q[3]']


trueroll = []
truepitch = []
trueyaw = []

ekfroll = []
ekfpitch = []
ekfyaw = []

roll = []
pitch = []
yaw = []

truepos['z'] = [-x for x in truepos['z']]

qall = zip(q0, q1, q2, q3)

for q in qall:
    trueroll.append(math.atan2(2.0 * (q[1] * -q[0] + q[3] * -q[2]), q[1] * q[1] - -q[0] * -q[0] - q[3] * q[3] + -q[2] * -q[2]))
    truepitch.append(-math.asin(2.0 * (-q[0] * -q[2] - q[1] * q[3])))
    trueyaw.append(math.atan2(2.0 * (-q[0] * q[3] + q[1] * -q[2]), q[1] * q[1] + -q[0] * -q[0] - q[3] * q[3] - -q[2] * -q[2]))

temproll = []
for r in trueroll:
    r += 3.14159
    if r > 3.14159:
        r = r - 2*3.14159
    temproll.append(r)
trueroll = temproll

q0 = attitude['q[0]']
q1 = attitude['q[1]']
q2 = attitude['q[2]']
q3 = attitude['q[3]']

qall = zip(q0, q1, q2, q3)

for q in qall:
    ekfroll.append(math.atan2(2.0 * (q[1] * -q[0] + q[3] * -q[2]), q[1] * q[1] - -q[0] * -q[0] - q[3] * q[3] + -q[2] * -q[2]))
    ekfpitch.append(-math.asin(2.0 * (-q[0] * -q[2] - q[1] * q[3])))
    ekfyaw.append(math.atan2(2.0 * (-q[0] * q[3] + q[1] * -q[2]), q[1] * q[1] + -q[0] * -q[0] - q[3] * q[3] - -q[2] * -q[2]))

temproll = []
for r in ekfroll:
    r += 3.14159
    if r > 3.14159:
        r = r - 2*3.14159
    temproll.append(r)
ekfroll = temproll

plt.ylim(-0.01, 0.01)
plt.xlim(0, 33)
plt.xlabel('Time (seconds)')
plt.ylabel('Difference (rad)')

offset = 19

plt.grid()

# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['x_gps'], color='b', linewidth=1.5, label='Z Speed')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['z_gps'], color='purple', linewidth=1.5, label='Altitude GPS')

# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['x'], color='g', linewidth=1.5, label='Nonlinear Observer Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['pitch'], color='purple', linewidth=1.5, label='XKF Pitch')

# plt.scatter([x/1000000.0 - offset for x in kalman['timestamp']], kalman['y'], color='orange', linewidth=0.01, label='Extrapolation')

# plt.plot([x/1000000.0 - offset for x in truepos['timestamp']], truepos['x'], color='g', linewidth=1.5, label='True X')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['d'], color='b', linewidth=1.5, label='Raw')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['e'], color='r', linewidth=1.5, label='Observer')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y_gps'], color='g', linewidth=1.5, label='XKF X Position')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['c'], color='b', linewidth=1.5, label='Xkf')

# ---------------------------- X Position ----------------------------- #
x_gps = []
x_gps_timestamp = []
last_x = -9999
for (time, x) in zip(exogenous['timestamp'], exogenous['a']):
    if x != last_x:
        x_gps.append(x)
        x_gps_timestamp.append(time)
        last_x = x

# plt.plot([x/1000000.0 - offset for x in truepos['timestamp']], truepos['x'], color='r', linewidth=1.5, label='True X Position')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['x_gps'], color='g', linewidth=1.5, label='XKF X Position')
# plt.scatter([x/1000000.0 - offset for x in x_gps_timestamp], x_gps, color='orange', linewidth=1.5, label='GPS Data Points')

# ---------------------------- X Position ----------------------------- #

# ---------------------------- Y Position ----------------------------- #
y_gps = []
y_gps_timestamp = []
last_y = -9999
for (time, y) in zip(exogenous['timestamp'], exogenous['c']):
    if y != last_y:
        y_gps.append(-y)
        y_gps_timestamp.append(time)
        last_y = y

# plt.plot([x/1000000.0 - offset for x in truepos['timestamp']], [-x for x in truepos['y']], color='r', linewidth=1.5, label='True Y Position')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y_gps'], color='g', linewidth=1.5, label='XKF Y Position')
# plt.scatter([x/1000000.0 - offset for x in y_gps_timestamp], y_gps, color='orange', linewidth=1.5, label='GPS Data Points')
# ---------------------------- Y Position ----------------------------- #

# ---------------------------- Z Position ----------------------------- #
# plt.plot([x/1000000.0 - offset for x in truepos['timestamp']], [-x for x in truepos['z']], color='r', linewidth=1.5, label='True Z Position')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [-x for x in exogenous['z_gps']], color='g', linewidth=1.5, label='XKF Z Position')
# ---------------------------- Z Position ----------------------------- #

# ---------------------------- XY Position ----------------------------- #
# plt.plot(exogenous['x_gps'], exogenous['y_gps'], color='g', linewidth=1.5, label='XKF XY Position')
# plt.plot(truepos['x'], [-x for x in truepos['y']], color='r', linewidth=1.5, label='True XY Position')
# plt.scatter(x_gps, y_gps, color='orange', linewidth=1.5, label='GPS Data Points')
# ---------------------------- XY Position ----------------------------- #


# ------------------------------- PITCH ------------------------------ #
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y'], color='purple', linewidth=2, label='Observer Pitch')
# plt.plot([x/1000000.0 - offset for x in trueatt['timestamp']], truepitch, color='r', linewidth=1.5, label='True Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['pitch'], color='g', linewidth=1.5, label='XKF Pitch')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['pitch'], color='b', linewidth=1.5, label='EKF Pitch')
# plt.plot([x/1000000.0 - offset for x in attitude['timestamp']], ekfpitch, color='b', linewidth=2, label='EKF2 pitch')
# ------------------------------- PITCH ------------------------------ #

# ------------------------------- ROLL ------------------------------ #
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['x'], color='purple', linewidth=1.5, label='Observer Roll')
# plt.plot([x/1000000.0 - offset for x in trueatt['timestamp']], trueroll, color='r', linewidth=1.5, label='True Roll')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['roll'], color='g', linewidth=1.5, label='XKF Roll')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['roll'], color='b', linewidth=1.5, label='EKF Roll')
# plt.plot([x/1000000.0 - offset for x in attitude['timestamp']], ekfroll, color='b', linewidth=1.5, label='EKF2 Roll')
# ------------------------------- ROLL ------------------------------ #

# ------------------------------- YAW ------------------------------ #
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['z'], color='purple', linewidth=1.5, label='Observer Yaw')
# plt.plot([x/1000000.0 - offset for x in trueatt['timestamp']], trueyaw, color='r', linewidth=1.5, label='True Yaw')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['yaw'], color='g', linewidth=1.5, label='XKF Yaw')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['yaw'], color='b', linewidth=1.5, label='EKF Yaw')
# plt.plot([x/1000000.0 - offset for x in attitude['timestamp']], ekfyaw, color='b', linewidth=1.5, label='EKF2 pitch')
# ------------------------------- YAW ------------------------------ #

# ------------------------------- DIFFERENCE ------------------------------ #
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [x-y for (x, y) in zip(exogenous['roll'], exogenous['g'])], color='r', linewidth=1.5, label='Roll Error')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [x-y for (x, y) in zip(exogenous['yaw'], exogenous['i'])], color='b', linewidth=1.5, label='Yaw Error')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [x-y for (x, y) in zip(exogenous['pitch'], exogenous['h'])], color='g', linewidth=1.5, label='Pitch Error')
# ------------------------------- DIFFERENCE ------------------------------ #

# ------------------------------- EXTRAPOLATION ------------------------------ #
# plt.plot([x/1000000.0 - offset for x in truepos['timestamp']], truepos['x'], color='r', linewidth=1.5, label='True X Position')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['a'], color='b', linewidth=2.5, label='Zero Order Hold GPS X Position')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['g'], color='g', linewidth=2.5, label='Extrapolated GPS X Position')
# plt.scatter([x/1000000.0 - offset for x in x_gps_timestamp], x_gps, color='orange', linewidth=2, label='GPS Data Points', zorder=10)
# ------------------------------- EXTRAPOLATION ------------------------------ #

# ------------------------------- RK4 VS EULER ------------------------------ #
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y'], color='purple', linewidth=2, label='Observer Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [x-y for (x, y) in zip(exogenous['roll'], exogenous['g'])], color='r', linewidth=1.5, label='Roll Error RK4')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [x-y for (x, y) in zip(exogenous['x_gps'], exogenous['g'])], color='g', linewidth=1.5, label='Roll Error Euler')
plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], [x-y for (x, y) in zip(exogenous['x_gps'], exogenous['roll'])], color='r', linewidth=1.5, label='Euler - RK4')
# plt.plot([x/1000000.0 - offset for x in trueatt['timestamp']], truepitch, color='r', linewidth=1.5, label='True Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['pitch'], color='g', linewidth=1.5, label='XKF Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y_gps'], color='b', linewidth=1.5, label='XKF Pitch')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['pitch'], color='b', linewidth=1.5, label='EKF Pitch')
# plt.plot([x/1000000.0 - offset for x in attitude['timestamp']], ekfpitch, color='b', linewidth=2, label='EKF2 pitch')
# ------------------------------- RK4 VS EULER ------------------------------ #

plt.legend(loc='upper left')

plt.savefig(script_dir + '/figures/euler_vs_rk4_difference.png', bbox_inches='tight', dpi=300)

plt.show()
