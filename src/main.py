import os
import time
import sys

from ctrl.sdir_ctrl import SdirCtrl
from ctrl.com.json_handler import JsonHandler
from ctrl.postypes.SixDPos import SixDPos
from ctrl.postypes.configuration import configuration

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


# Define operation modes
class OpMode:
    CFG_2_POS = 0
    POS_2_CFG = 1
    PTP = 2
    PTPSYNC = 3
    LIN = 4


jh = [0] * 6


def initial():
    global client, sim, jh

    portNb = 23000

    try:
        # Initialize the CoppeliaSim remote API client
        client = RemoteAPIClient('127.0.0.1', portNb)

        # Use client.require to get the 'sim' object
        sim = client.getObject('sim')

        # Load your scene file in CoppeliaSim if needed
        scene_path = os.path.abspath('../sim/Kuka KR120 2700-2 SDIR.ttt')
        print("Connected to CoppeliaSim")

        # Load the scene and start the simulation
        sim.loadScene(scene_path)
        sim.startSimulation()
        print("Simulation Started")

        # Get object handles for the robot joints
        for i in range(6):
            Joint = "KR120_2700_2_joint" + str(i + 1)
            jh[i] = sim.getObjectHandle(Joint)

    except Exception as e:
        print(f"Error during initialization: {str(e)}")
        raise e

    return client


def main():
    global client, sim
    print("This is the entry point of the SDIR programming project")
    ctrl = SdirCtrl()  # Create an instance of SdirCtrl
    c = [0.0] * 6

    try:
        initial()  # Connect to the simulator
    except Exception as e:
        print("Error during initialization, exiting.")
        return

    running = True
    while running:  # Use the flag in the loop condition

        signal_value = sim.getStringSignal("callsignal")

        if signal_value is not None and len(signal_value) > 0:

            t = signal_value.decode('utf-8')
            json_handler = JsonHandler(t)

            if json_handler.get_op_mode() == OpMode.POS_2_CFG:
                pos = SixDPos(json_handler.get_data()[0])
                result_cfg = ctrl.get_config_from_pos(pos)
                json_return_string = json_handler.get_json_string(result_cfg)
                sim.setStringSignal("returnsignal", json_return_string)
                scriptHandle = sim.getScriptHandle('Coord_Dialog')
                sim.callScriptFunction("returnSignal", scriptHandle)

            if json_handler.get_op_mode() == OpMode.CFG_2_POS:
                cfg = configuration(json_handler.get_data()[0])
                return_pos = ctrl.get_pos_from_config(cfg)
                json_return_string = json_handler.get_json_string(return_pos)
                sim.setStringSignal("returnsignal", json_return_string)
                scriptHandle = sim.getScriptHandle('Coord_Dialog')
                sim.callScriptFunction("returnSignal", scriptHandle)

            if json_handler.get_op_mode() == OpMode.PTP:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_ptp(start_cfg, end_cfg, "async")
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    # Synchronize with V-REP simulation environment
                    time.sleep(0.05)

            if json_handler.get_op_mode() == OpMode.PTPSYNC:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_ptp(start_cfg, end_cfg, "sync")
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    # Synchronize with V-REP simulation environment
                    time.sleep(0.05)

            if json_handler.get_op_mode() == OpMode.LIN:
                start_cfg = configuration(json_handler.get_data()[0])
                end_cfg = configuration(json_handler.get_data()[1])
                trajectory = ctrl.move_robot_lin(start_cfg, end_cfg)
                for cur_cfg in trajectory.get_all_configuration():
                    c[0] = float(cur_cfg[0])
                    c[1] = float(cur_cfg[1])
                    c[2] = float(cur_cfg[2])
                    c[3] = float(cur_cfg[3])
                    c[4] = float(cur_cfg[4])
                    c[5] = float(cur_cfg[5])
                    script_handle = sim.getScriptHandle('KR120_2700_2')
                    sim.callScriptFunction("runConfig", script_handle, len(c), c, "", "")
                    # Synchronize with V-REP simulation environment
                    time.sleep(0.05)

            sim.clearStringSignal("callsignal")

            time.sleep(0.05)  # Add a delay for synchronization


if __name__ == "__main__":
    main()