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

convert_ulog2csv(latest_file, 'vehicle_local_position,vehicle_local_position_groundtruth,sensor_combined,extended_kalman,actuator_outputs,vehicle_attitude,vehicle_attitude_groundtruth,exogenous_kalman', False, ',')

pos = pd.read_csv(os.path.splitext(latest_file)[0] + '_vehicle_local_position_0.csv')
truepos = pd.read_csv(os.path.splitext(latest_file)[0] + '_vehicle_local_position_groundtruth_0.csv')
kalman = pd.read_csv(os.path.splitext(latest_file)[0] + '_extended_kalman_0.csv')
actuators = pd.read_csv(os.path.splitext(latest_file)[0] + '_actuator_outputs_0.csv')
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

# plt.ylim(0.09, 0.27)
# plt.xlim(0, 12)
plt.xlabel('Time (seconds)')
plt.ylabel('Pitch (rad)')

offset = 35.5

# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['x_gps'], color='b', linewidth=1.5, label='Z Speed')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['z_gps'], color='purple', linewidth=1.5, label='Altitude GPS')

# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['x'], color='g', linewidth=1.5, label='Nonlinear Observer Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['pitch'], color='purple', linewidth=1.5, label='XKF Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y'], color='b', linewidth=1.5, label='observer Pitch')
# plt.plot([x/1000000.0 - offset for x in truepos['timestamp']], truepos['y'], color='g', linewidth=1.5, label='observer Pitch')
plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y'], color='b', linewidth=1.5, label='observer Pitch')
plt.plot([x/1000000.0 - offset for x in sensors['timestamp']], [x for x in sensors['accelerometer_m_s2[0]']], color='b', linewidth=1.5, label='observer Pitch')
plt.plot([x/1000000.0 - offset for x in sensors['timestamp']], [x for x in sensors['accelerometer_m_s2[1]']], color='g', linewidth=1.5, label='observer Pitch')
plt.plot([x/1000000.0 - offset for x in sensors['timestamp']], [x for x in sensors['accelerometer_m_s2[2]']], color='r', linewidth=1.5, label='observer Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y_gps'], color='r', linewidth=1.5, label='observer Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['b'], color='y', linewidth=1.5, label='observer Pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y'], color='b', linewidth=1.5, label='Gyro pitch')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['y_gps'], color='lightseagreen', linewidth=1.5, label='ty filtered')
# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['z_gps'], color='b', linewidth=1.5, label='ty')
# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['pitch'], color='b', linewidth=1.5, label='EKF pitch')
plt.plot([x/1000000.0 - offset for x in trueatt['timestamp']], truepitch, color='r', linewidth=1.5, label='True pitch')
# plt.plot([x/1000000.0 - offset for x in attitude['timestamp']], ekfpitch, color='g', linewidth=1.5, label='EKF2 pitch')

# plt.plot([x/1000000.0 - offset for x in kalman['timestamp']], kalman['yaw'], color='b', linewidth=1.5, label='EKF filtered yaw')

# plt.plot([x/1000000.0 - offset for x in exogenous['timestamp']], exogenous['yaw'], color='g', linewidth=1.5, label='XKF filtered yaw')

plt.legend(loc='upper left')

# plt.savefig(script_dir + '/foo.png', bbox_inches='tight', dpi=300)

plt.show()
