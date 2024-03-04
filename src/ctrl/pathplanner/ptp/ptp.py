from ...postypes.configuration import configuration
from ...postypes.trajectory import trajectory

import math
import numpy as np

try:
    import matplotlib.pyplot as plt
    ALLOW_PLOTTING = True
except ImportError:
    ALLOW_PLOTTING = False


class Ptp:
    def __init__(self):
        self.MODE = "async"  # async, sync, full_sync
        self.T_Ipo = 0.005

        self.q_start_config = None
        self.q_end_config = None
        self.v_initial = [120, 115, 120, 190, 180, 260]
        self.a_initial = [500 for _ in range(6)]
        self.last_s_t_values = [0, 0, 0, 0, 0, 0]
        self.timesteps = []

        # values for the leading joint
        self.lead_rotation_to_travel = 0
        self.lead_trajectory_time    = 0
        self.lead_joint_number       = 0

    def get_ptp_trajectory(self, start_cfg: configuration, end_cfg: configuration, d=0, m="sync") -> trajectory:

        self.MODE = m
        self.q_start_config = list(np.degrees(start_cfg.get_configuration()))
        self.q_end_config   = list(np.degrees(end_cfg.get_configuration()))

        self.determine_leading_joint()

        print("configs:", self.q_start_config, self.q_end_config)

        # --- loop over joints to get times, speeds and accelerations -----------
        q_t_list = []
        for n, (q_s, q_z) in enumerate(zip(self.q_start_config, self.q_end_config)):

            # variables that stay the same for sync and async and lead/non-lead joint
            s_e     = self.get_rotation_to_travel(q_s, q_z)
            v_m_hat = self.v_initial[n]
            b_m_hat = self.a_initial[n]

            # genereate timesteps
            timesteps = np.arange(1e-10, self.lead_trajectory_time, self.T_Ipo)

            # skip calculations if there is no distance to travel
            if math.isclose(s_e, 0):
                q_t_list.append([q_s for _ in timesteps])
                print("skipping for joint ", n)
                continue

            # --- asynchronous equations ------------------------------
            if self.MODE == 'async':
                v_m_calc = math.sqrt(s_e * self.a_initial[n])
                if self.v_initial[n] > v_m_calc and s_e != 0: v_m = v_m_calc
                else: v_m = self.v_initial[n]  # speed limit
                b_m = self.a_initial[n]
                t_b = self.get_acceleration_duration(v_m, b_m)
                t_e = self.get_trajectory_duration(s_e, v_m, t_b)
                t_v = self.get_constant_velocity_duration(t_b, t_e)

            # --- synchronous leading joint equations ----------------
            elif self.MODE == 'sync' and n == self.lead_joint_number:
                t_b = math.ceil((v_m_hat  / (b_m_hat * self.T_Ipo))) * self.T_Ipo
                t_v = math.ceil((s_e      / (v_m_hat * self.T_Ipo))) * self.T_Ipo
                t_e = t_v + t_b
                v_m = s_e / t_v
                b_m = s_e / (t_v * t_b)

            # --- synchronous non-leading joint equations ------------
            else:
                t_e = self.lead_trajectory_time
                v_m_hat = ((b_m_hat * t_e) / 2) - math.sqrt(((b_m_hat**2) * (t_e**2)) / 4 - s_e * b_m_hat)
                t_v = math.ceil((s_e / (v_m_hat * self.T_Ipo))) * self.T_Ipo
                v_m = s_e / t_v  # in the diagram it says t_e
                t_b = t_e - t_v
                t_e = t_v + t_b
                b_m = v_m / t_b

            q_t_list.append([self.trajectory(t, q_s, q_z, n, t_e, t_b, t_v, b_m, v_m, d) for t in timesteps])

        # collect output trajectories
        trajectory_output = trajectory()
        [trajectory_output.add_configuration(configuration(list(np.radians(list(t))))) for t in zip(*q_t_list)]

        if ALLOW_PLOTTING: self.plot(trajectory_output)

        return trajectory_output

    def determine_leading_joint(self):
        for n, (q_s, q_z) in enumerate(zip(self.q_start_config, self.q_end_config)):
            # get current joints metrics to determine leading joint
            rotation_to_travel = self.get_rotation_to_travel(q_s, q_z)
            acceleration_time  = self.get_acceleration_duration(self.v_initial[n], self.a_initial[n])
            trajectory_time    = self.get_trajectory_duration(rotation_to_travel, self.v_initial[n], acceleration_time)

            # save information about leading joint
            if rotation_to_travel > self.lead_rotation_to_travel: self.lead_joint_number = n
            self.lead_rotation_to_travel = max(rotation_to_travel, self.lead_rotation_to_travel)
            self.lead_trajectory_time    = max(trajectory_time, self.lead_trajectory_time)  # slowest joint

    def trajectory(self, t, q_s, q_z, n_joint, t_e, t_b, t_v, b_m, v_m, derive_x_times=0):

        # --- no movement ---------------------
        q_t = 0
        if q_s == q_z:
            q_t = 0

        # --- acceleration phase --------------
        elif 0 < t <= t_b:
            if   derive_x_times == 0: q_t = q_s + sign(q_z - q_s) * (1/2) * b_m * t**2
            elif derive_x_times == 1: q_t = sign(q_z - q_s) * b_m * t
            elif derive_x_times == 2: q_t = sign(q_z - q_s) * b_m

        # --- constant velocity phase ---------
        elif t_b < t <= t_v:
            if   derive_x_times == 0: q_t = q_s + sign(q_z - q_s) * (v_m * t - ((v_m**2) / (2*b_m)))
            elif derive_x_times == 1: q_t = sign(q_z - q_s) * v_m
            elif derive_x_times == 2: q_t = 0

        # --- deceleration phase --------------
        elif t_v < t <= t_e:
            if   derive_x_times == 0: q_t = q_s + sign(q_z - q_s) * (v_m * t_v - (b_m/2) * (t_e - t)**2)
            elif derive_x_times == 1: q_t = sign(q_z - q_s) * b_m * (t_e - t)
            elif derive_x_times == 2: q_t = -sign(q_z - q_s) * b_m

        else:
            # t may not be larger than t_e unless async
            if self.MODE != "async": print("WARNING", t)
            q_t = self.last_s_t_values[n_joint]
            return q_t

        self.last_s_t_values[n_joint] = q_t
        return q_t

    @staticmethod
    def get_trajectory_duration(rotation_to_travel, maximum_velocity, acceleration_duration):
        # trajectory duration ^= t_e, maximum_velocity ^= v_m, acceleration_duration ^= t_b
        return (rotation_to_travel / maximum_velocity) + acceleration_duration  # -> t_e

    @staticmethod
    def get_acceleration_duration(maximum_velocity, maximum_acceleration):
        # accceleration_duration ^= t_b, maximum_velocity ^= v_m, maximum_acceleration ^= b_m
        return maximum_velocity / maximum_acceleration  # -> t_b

    @staticmethod
    def get_constant_velocity_duration(acceleration_duration, trajectory_duration):
        # constant_velocity_duration ^= t_v
        # assumption: acceleration time is as long as deceleration time
        return trajectory_duration - acceleration_duration

    @staticmethod
    def get_rotation_to_travel(q_start, q_goal):
        # q_start ^= q_s, q_goal ^= q_z, rotation_to_travel ^= s_e
        return abs(q_start - q_goal) # -> s_e

    def plot(self, traj):

        # Given configurations
        # problem moving from negative to more negative
        t = 0.004

        # Create a single subplot
        fig, ax = plt.subplots()

        # --- DISTANCE --------------------------------
        myTrajectory = traj
        # Set y-axis range and label
        ax.set_ylim(-180, 180)
        ax.set_ylabel('Joint Angle [°]')

        # Add grid lines
        ax.yaxis.set_major_locator(plt.MultipleLocator(10))
        ax.yaxis.set_minor_locator(plt.MultipleLocator(5))
        ax.grid(which='both', linestyle='--', linewidth=0.5)

        for i in range(6):
            joint_data = [config.get_configuration()[i]*180/math.pi for config in myTrajectory]

            # Plot the line for each joint
            ax.plot(np.arange(0, len(myTrajectory.get_all_configuration())*t, t), joint_data, label=f'Joint {i + 1}')

            # Plot start and goal points for each joint
            ax.scatter(0, self.q_start_config[i], marker='o', color=f'C{i}', label='_nolegend_')
            ax.scatter((len(myTrajectory.get_all_configuration()) - 1)*t, self.q_end_config[i], marker='o',
                       color=f'C{i}', label='_nolegend_')

        # Set x-axis label
        ax.set_xlabel('Time [s]')

        # Add a legend
        ax.legend()

        plt.show()


def sign(x):
    if   x <  0: return -1
    elif x == 0: return  0
    elif x >  0: return +1
