import sys
import logging
import csv
import time

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from datetime import datetime
from enum import Enum

from functools import reduce
logging.basicConfig(level=logging.INFO)
logging.getLogger().setLevel(logging.INFO)


class UR10_RTDE():
    keep_running = True

    def __init__(self, robo_host: str, robo_port: int, config_filename: str, record: bool = False):
        self.record = record

        # get recipes!
        conf = rtde_config.ConfigFile(config_filename)
        self.state_names, self.state_types = conf.get_recipe('state')
        self.control_names, self.control_types = conf.get_recipe('control')

        # connect, get controller version
        self.con = rtde.RTDE(robo_host, robo_port)
        self.con.connect()
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(self.state_names, self.state_types)
        self.control = self.con.send_input_setup(self.control_names, self.control_types)


    def writeout(self, *msg: str):
        current_time = datetime.now().strftime("%H:%M:%S.%f")
        print(current_time + ":", *msg)


    def begin(self):
        # Start data synchronization
        if not self.con.send_start():
            sys.exit()

    def process(self):
        self.begin()
        while True:
            # Check the connection
            self.state = self.con.receive()
            if self.state is None:
                self.writeout("Conn lost")
                break
            self.writeout(self.state.actual_TCP_pose)

            self.control.input_double_register_0 = 1
            self.con.send(self.control)
    
    def wrap_process(self):
        self.end()

    def end(self):
        # Close the connection
        self.con.send_pause()
        self.con.disconnect()


if __name__ == "__main__":
    HOST, PORT = '10.1.5.187', 30004

    # Try and run the process
    try:
        robo = UR10_RTDE(HOST, PORT, 'portmark_base.xml')
        robo.process()
    except KeyboardInterrupt as keyexc:
        print("Keyboard interrupt")
    finally:
        robo.wrap_process()