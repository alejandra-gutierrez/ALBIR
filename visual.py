# -*- coding: UTF-8 -*-
"""
@Project : ICL-BioEng-ALBiR-PixyBot
@File    : visual.py
@IDE     : PyCharm
@Author  : Peter
@Date    : 19/02/2022 20:45
@Brief   :
"""
import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from collections import defaultdict

config = {
    "font.family": 'Times New Roman',
    "font.size": 16,
    "mathtext.fontset": 'stix',
    "axes.titlesize": 20
}
rcParams.update(config)


class ServoData:
    def __init__(self, path: str):
        self.path = path
        data = pd.read_csv(self.path, sep=',', header=None).values

        try:
            self.__frequency = float(re.findall(r"calibrationCurve(\d+\.?\d+)", os.path.basename(self.path))[0])
        except:
            self.__frequency = float(re.findall(r"calibrationCurve(\d+)", os.path.basename(self.path))[0])

        self.__time_stamp = data[0, :]
        self.__angular_error = data[1, :]
        self.__servo_commands = data[2, :]

    @property
    def time_step(self) -> np.ndarray:
        return self.__time_stamp

    @property
    def angular_error(self) -> np.ndarray:
        return self.__angular_error

    @property
    def servo_commands(self) -> np.ndarray:
        return self.__servo_commands

    @property
    def frequency(self) -> float:
        return self.__frequency

    def __del__(self):
        return 'Delete'


class BodePlot:
    # Bode plot - gain and phase as a functions pf input signal frequencies
    # - Gain: system response compared to the input signal, the gain is teh amplitude of servo positive vs time
    # - Phase: system response timing relative to the input signal, the phase lag between these 2 signals
    def __init__(self, data_root: str):
        self.data_root = data_root

    def parse_amplitude(self, servo_data: ServoData) -> float:
        valid_idx = np.where(servo_data.servo_commands < -20)[0][0]
        max_value = np.max(servo_data.servo_commands[valid_idx:])
        min_value = np.min(servo_data.servo_commands[valid_idx:])

        return (max_value - min_value) / 2

    def parse_theoretical_amplitude(self, servo_data: ServoData) -> float:
        """In the theoretical amplitude, we need to add error and servo commands."""
        data = servo_data.servo_commands + servo_data.angular_error
        valid_idx = np.where(data < -20)[0][0]
        max_value = np.max(data[valid_idx:])
        min_value = np.min(data[valid_idx:])

        return (max_value - min_value) / 2

    def parse_error_amplitude(self, servo_data: ServoData) -> float:
        valid_idx = np.where(servo_data.angular_error < -2)[0][0]
        max_value = np.max(servo_data.angular_error[valid_idx:])
        min_value = np.min(servo_data.angular_error[valid_idx:])

        return (max_value - min_value) / 2

    def parse_time(self, servo_data: ServoData):
        theoretical_data = servo_data.servo_commands + servo_data.angular_error
        idx_theo = np.where(theoretical_data < -20)[0][0]
        idx_actual = np.where(servo_data.servo_commands < -20)[0][0]

        return np.abs(servo_data.time_step[idx_actual] - servo_data.time_step[idx_theo])

    def run(self):
        # Get the all csv files.
        paths = [os.path.join(self.data_root, path) for path in os.listdir(self.data_root) if path.endswith('.csv')]
        # Get the corresponding data
        amplitudes_phaseLag_frequency = []

        rms = defaultdict(list)
        for path in paths:
            servo_data = ServoData(path)

            amplitude = self.parse_amplitude(servo_data)
            # Calculate the rms data.
            rms['servo_amplitude'].append(np.sqrt(amplitude))
            rms['error'].append(np.sqrt(self.parse_error_amplitude(servo_data)))

            phase_time = self.parse_time(servo_data)
            # Calculate the phase lag
            phase_lag = phase_time * servo_data.frequency * 2 * np.pi

            # Calculate the normalized amplitude
            normalized_amplitude = amplitude / self.parse_theoretical_amplitude(servo_data)
            frequency = servo_data.frequency
            amplitudes_phaseLag_frequency.append((frequency, normalized_amplitude, phase_lag))
            # phase_frequency_lags.append((frequency, phase_lag))

        # Sort the amplitude and frequency pair.
        amplitudes_phaseLag_frequency = sorted(amplitudes_phaseLag_frequency, key=lambda x: x[0])
        amplitudes, frequencies, phase_lags = [], [], []
        for frequency, amplitude, phase_lag in amplitudes_phaseLag_frequency:
            amplitudes.append(amplitude)
            frequencies.append(frequency)
            phase_lags.append(phase_lag)

        fig = plt.figure()
        plt.plot(frequencies, amplitudes, label='Gain')
        plt.plot(frequencies, phase_lags, label='Phase lag')
        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Normalized amplitude [-]")
        plt.title('Bode plot')
        plt.grid()
        plt.legend()
        plt.savefig('./images/Bode_plot.svg', format='svg', dpi=1600, bbox_inches="tight")
        plt.show()


if __name__ == "__main__":
    # csv_data_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    # csv_filepath = os.path.join(csv_data_root, 'calibrationCurve0.1Hz_1.csv')
    csv_filepath = "/Users/alejandra.gutierrez/Desktop/calibrationCurve0.1Hz_1.csv"
    data = ServoData(csv_filepath)

    # Visualize the raw data
    plt.plot(data.time_step, data.angular_error, label='error')
    plt.plot(data.time_step, data.servo_commands, label='servo commands')
    plt.plot(data.time_step, data.servo_commands + data.angular_error, label='Theoretical commands')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [$^\circ$]')
    plt.title('The servo performance')
    plt.legend()
    plt.grid()
    plt.savefig('calibrationCurve01Hz_1.svg', format='svg', dpi=1600, bbox_inches="tight")
    plt.show()

    # Plot the bode graph
    # bode_plot = BodePlot(csv_data_root)
    # bode_plot.run()
