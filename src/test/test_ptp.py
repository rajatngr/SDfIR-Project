from ctrl.postypes.configuration import configuration
from ctrl.pathplanner.ptp import ptp
import matplotlib.pyplot as plt
import numpy as np


class TestPtp:
    def test_get_ptp_trajectory(self):

        ptpObj = ptp.Ptp()

        # Given configurations
        myConfig1 = configuration([-180, -90, +45, -170, 0, 0])
        myConfig2 = configuration([180, -103.0235, -80.55, 15, 0, 0])
        # problem moving from negative to more negative
        t = 0.004

        # Create a single subplot
        fig, ax = plt.subplots(2, 2)

        for idx, mode in enumerate(["async", "sync"]):
            # --- DISTANCE --------------------------------
            myTrajectory = ptpObj.get_ptp_trajectory(myConfig1, myConfig2, d=0, m=mode)
            # Set y-axis range and label
            ax[0][idx].set_ylim(-180, 180)
            ax[0][idx].set_ylabel('Joint Angle [°]')

            # Add grid lines
            ax[0][idx].yaxis.set_major_locator(plt.MultipleLocator(10))
            ax[0][idx].yaxis.set_minor_locator(plt.MultipleLocator(5))
            ax[0][idx].grid(which='both', linestyle='--', linewidth=0.5)

            for i in range(6):
                joint_data = [config.get_configuration()[i] for config in myTrajectory]

                # Plot the line for each joint
                ax[0][idx].plot(np.arange(0, len(myTrajectory.get_all_configuration())*t, t), joint_data, label=f'Joint {i + 1}')

                # Plot start and goal points for each joint
                ax[0][idx].scatter(0, myConfig1.get_configuration()[i], marker='o', color=f'C{i}', label='_nolegend_')
                ax[0][idx].scatter((len(myTrajectory.get_all_configuration()) - 1)*t, myConfig2.get_configuration()[i], marker='o',
                           color=f'C{i}', label='_nolegend_')

            # Set x-axis label
            ax[0][idx].set_xlabel('Time [s]')

            # Add a legend
            ax[0][idx].legend()

            # --- SPEED -----------------------------------
            myTrajectory = ptpObj.get_ptp_trajectory(myConfig1, myConfig2, d=1, m=mode)
            # Set y-axis range and label
            ax[1][idx].set_ylabel('Joint speed [°/s]')

            # Add grid lines
            ax[1][idx].yaxis.set_major_locator(plt.MultipleLocator(10))
            ax[1][idx].yaxis.set_minor_locator(plt.MultipleLocator(5))
            ax[1][idx].grid(which='both', linestyle='--', linewidth=0.5)

            for i in range(6):
                joint_data = [config.get_configuration()[i] for config in myTrajectory]

                # Plot the line for each joint
                ax[1][idx].plot(np.arange(0, len(myTrajectory.get_all_configuration())*t, t), joint_data, label=f'Joint {i + 1}')

            # Set x-axis label
            ax[1][idx].set_xlabel('Time [s]')

            # Add a legend
            ax[1][idx].legend()

        plt.show()

    def test_trajectory(self):
        ptpObj = ptp.Ptp()

        q_start = 0
        q_end   = 20

        x = np.arange(0, 4, 0.1)

        y  = [ptpObj.trajectory(val, q_start, q_end) for val in x]
        y1 = [ptpObj.trajectory(val, q_start, q_end, 1) for val in x]
        y2 = [ptpObj.trajectory(val, q_start, q_end, 2) for val in x]
        print(y)

        fig, ax = plt.subplots(1,3)
        ax[0].plot(x, y)
        ax[1].plot(x, y1)
        ax[2].plot(x, y2)
        plt.show()

    def test_parameter(self):
        ptpObj = ptp.Ptp()

        q_start = 0
        q_end   = 20

        x = np.arange(0, 10, 0.01)

        y  = [ptpObj.parameter(val, q_start, q_end) for val in x]
        y1 = [ptpObj.parameter(val, q_start, q_end, 1) for val in x]
        y2 = [ptpObj.parameter(val, q_start, q_end, 2) for val in x]
        print(y)

        fig, ax = plt.subplots(1,3)
        ax[0].plot(x, y)
        ax[1].plot(x, y1)
        ax[2].plot(x, y2)
        plt.show()