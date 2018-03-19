import pandas as pd
import matplotlib.pyplot as plt
import numpy
import glob
import os

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
print(latest_file)

convert_ulog2csv(latest_file, 'extended_kalman_pos,sensor_combined,actuator_outputs', False, ',')


# sensors = pd.read_csv('1/sensors.csv')

# pos = pd.read_csv('1/local_pos.csv')
# true_pos = pd.read_csv('1/local_pos_real.csv')

# att = pd.read_csv('1/att.csv')
# true_att = pd.read_csv('1/att_real.csv')

# q_array = [[att['q[0]'][i], att['q[1]'][i], att['q[2]'][i], att['q[3]'][i]] for (i, q) in enumerate(att['q[0]'])]
# true_q_array = [[true_att['q[0]'][i], true_att['q[1]'][i], true_att['q[2]'][i], true_att['q[3]'][i]] for (i, q) in enumerate(true_att['q[0]'])]

# phi = [numpy.arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])) for q in q_array]
# theta = [numpy.arcsin(2*(q[0]*q[2]-q[3]*q[1])) for q in q_array]
# psi = [numpy.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])) for q in q_array]

# true_phi = [numpy.arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2])) for q in true_q_array]
# true_theta = [numpy.arcsin(2*(q[0]*q[2]-q[3]*q[1])) for q in true_q_array]
# true_psi = [numpy.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3])) for q in true_q_array]


# pos = pd.read_csv('2/pos.csv')
# kalman = pd.read_csv('2/kalman.csv')


# plt.scatter(att['timestamp'], phi, color='r')
# plt.scatter(true_att['timestamp'], true_phi, color='b')

# plt.scatter(att['timestamp'], theta, color='r')
# plt.scatter(true_att['timestamp'], true_theta, color='b')

# plt.scatter(att['timestamp'], psi, color='r')
# plt.scatter(true_att['timestamp'], true_psi, color='b')

# plt.plot(sensors['accelerometer_m_s2[0]'])
# plt.plot(sensors['accelerometer_m_s2[1]'])
# plt.plot(sensors['accelerometer_m_s2[2]'])

# plt.plot(sensors['magnetometer_ga[0]'])
# plt.plot(sensors['magnetometer_ga[1]'])
# plt.plot(sensors['magnetometer_ga[2]'])

# plt.plot(sensors['gyro_rad[0]'])
# plt.plot(sensors['gyro_rad[1]'])
# plt.plot(sensors['gyro_rad[2]'])

# plt.scatter(pos.filter(items=['x']), pos.filter(items=['y']), color='b')
# plt.scatter(true_pos.filter(items=['x']), true_pos.filter(items=['y']), color='r')

# plt.scatter(att.filter(items=['timestamp']), att.filter(items=['q[0]']), color='b')
# plt.scatter(true_att.filter(items=['timestamp']), true_att.filter(items=['q[0]']), color='r')

# plt.scatter(pos['y'], pos['x'], color='b')
# plt.scatter(kalman['y'], kalman['x'], color='r')

# plt.scatter(pos['timestamp'], pos['x'], color='b')
# plt.scatter(kalman['timestamp'], kalman['x'], color='r')

# plt.scatter(true_att.filter(items=['timestamp']), true_att.filter(items=['q[0]']), color='r')

plt.show()
