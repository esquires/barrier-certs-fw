import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import lvdb


def calc_radius(vel, bank):
    return vel**2 / 9.81 / np.tan(bank)


def calc_turn_rate_from_radius(radius, vel):
    return vel / radius


def calc_turn_rate_from_bank(vel, bank):
    return -9.81 * np.tan(bank) / vel


def plot_bank_rise_time():
    df = pd.read_csv('scripts/bank_rise.csv')
    df['bank'] = np.rad2deg(df['w'])
    plt.plot(df['t'], df['bank'], label='actual')

    goal_bank = 30
    est_tau = 0.625
    plt.plot(df['t'], goal_bank * (1 - np.exp(-df['t'] / est_tau)), label='actual')
    plt.show()

def turn_rate_to_bank(vel, turn_rate):
    return -np.arctan2(vel * turn_rate, 9.81)

def t_delay_bank(curr_bank, gamma_bank):
    lvdb.set_trace()
    # make sure they have diff signs so it is worst case
    if abs(curr_bank - gamma_bank) < abs(curr_bank + gamma_bank):
        gamma_bank = -gamma_bank
    diff = abs(curr_bank - gamma_bank)
    tau_bank = 0.625
    reached_diff = np.deg2rad(1)
    return max(0, tau_bank * np.log(diff / reached_diff))


vmin = 15
vmax = 25
bmax = np.deg2rad(30)
turn_rate_max = calc_turn_rate_from_bank(vmax, bmax)

pct_offset = 0.1
gamma_vel = vmin + pct_offset * (vmax - vmin)
gamma_turn_rate = (1 - pct_offset) * turn_rate_max
gamma_bank = turn_rate_to_bank(gamma_vel, gamma_turn_rate)
radius = calc_radius(vmax, bmax)

print(('vmin = {}, vmax = {}, bmax = {}, '
       'max_turn_rate = {:.2f}, radius = {:.2f}').format(
    vmin, vmax, bmax, np.rad2deg(turn_rate_max), radius))

t_delay = t_delay_bank(bmax, gamma_bank)
print('worst delay for bank = {}'.format(t_delay))

base_sensor_range = 325
sensor_range = base_sensor_range + 2 * t_delay * vmax

print('update for sensing range = {}'.format(sensor_range))
plot_bank_rise_time()
