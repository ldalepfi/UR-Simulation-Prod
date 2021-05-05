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

cartons_enum = Enum('cartons', 'frozen_small frozen_large chilled_small chilled_medium chilled_large testing')
carton = cartons_enum.frozen_small

class UR10_RTDE():
    keep_running = True
    tasks = []

    controls = Enum('controls', 'left2right right2left')

    current_task = None
    task_active = None
    task_done = None
    homed = None
    printing = None
    prog_running = None

    __state = None
    rec_positions = []
    rec_joint_angles = []
    rec_prints = []
    rec_speeds = []
    rec_tspeeds = []

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, state):
        current_task = state.output_int_register_0
        task_active = state.output_bit_register_64
        task_done = state.output_bit_register_65
        homed = state.output_bit_register_67
        printing = state.output_bit_register_68
        prog_running = state.output_bit_register_74

        if self.current_task != current_task:
            self.writeout("CURRENT TASK", self.name_task(self.current_task), "->", self.name_task(current_task))
            self.current_task = current_task

        if self.task_active != task_active:
            self.writeout("TASK ACTIVE", self.task_active, "->", task_active)
            self.task_active = task_active

        if self.task_done != task_done:
            self.writeout("TASK DONE", self.task_done, "->", task_done)
            self.task_done = task_done

        if self.printing != printing:
            #self.writeout("PRINTING", self.printing, "->", printing)
            self.printing = printing

        if self.homed != homed:
            self.writeout("HOMED", self.homed, "->", homed)
            self.homed = homed

        if self.prog_running != prog_running:
            self.writeout("RUNNING", self.prog_running, "->", prog_running)
            self.prog_running = prog_running

        self.__state = state

    def __init__(self, robo_host: str, robo_port: int, config_filename: str, record: bool = False):
        self.record = record

        # get recipes!
        conf = rtde_config.ConfigFile(config_filename)
        self.state_names, self.state_types = conf.get_recipe('state')
        self.gantry_names, self.gantry_types = conf.get_recipe('gantry')
        self.internal_names, self.internal_types = conf.get_recipe('internal')
        self.home_names, self.home_types = conf.get_recipe('home')
        self.control_names, self.control_types = conf.get_recipe('control')
        self.positions_names, self.positions_types = conf.get_recipe('positions')


        # connect, get controller version
        self.con = rtde.RTDE(robo_host, robo_port)
        self.con.connect()
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(self.state_names, self.state_types)
        self.gantry = self.con.send_input_setup(self.gantry_names, self.gantry_types)
        self.internal = self.con.send_input_setup(self.internal_names, self.internal_types)
        self.home = self.con.send_input_setup(self.home_names, self.home_types)
        self.control = self.con.send_input_setup(self.control_names, self.control_types)
        self.positions = self.con.send_input_setup(self.positions_names, self.positions_types)


    def name_task(self, value: int):
        if (value is None) or (value == 0):
            return "None"
        return self.controls(value).name

    def writeout(self, *msg: str):
        current_time = datetime.now().strftime("%H:%M:%S.%f")
        print(current_time + ":", *msg)

    def add_task(self, task: tuple):
        self.tasks += [task]

    def set_defaults(self):
        # Next task


        self.internal.input_bit_register_64 = 0
        self.internal.input_bit_register_65 = 0

        self.home.input_bit_register_76 = 0

        # Control
        self.control.input_int_register_0 = 0


    def begin(self):
        # Start data synchronization
        if not self.con.send_start():
            sys.exit()

    def process(self):
        self.begin()

        self.gantry.input_bit_register_74 = 1
        self.gantry.input_bit_register_75 = 0
        self.con.send(self.gantry)

        self.positions.input_double_register_0 = 0
        self.positions.input_double_register_1 = 0
        self.positions.input_double_register_2 = 0
        self.positions.input_double_register_3 = 0
        self.positions.input_double_register_6 = 0
        self.positions.input_double_register_9 = 0
        self.con.send(self.positions)

        self.home.input_bit_register_76 = 0
        self.con.send(self.home)

        self.control.input_int_register_0 = 0
        self.con.send(self.control)

        program_counter = 0
        control_ack = True
        home_ack = True

        while True:
            # Check the connection
            self.state = self.con.receive()
            if self.state is None:
                self.writeout("Conn lost")
                break

            if not self.prog_running:
                next = eval(input("What next? {1: resume, 2: restart, 3: home}"))
                if next == 1:
                    self.internal.input_bit_register_64 = 1
                    self.internal.input_bit_register_65 = 0
                    self.con.send(self.internal)
                elif next == 2:
                    self.internal.input_bit_register_64 = 0
                    self.internal.input_bit_register_65 = 1
                    self.con.send(self.internal)
                elif next == 3:
                    home_ack = False
                    self.home.input_bit_register_76 = 1
                    self.con.send(self.home)
                    self.internal.input_bit_register_64 = 0
                    self.internal.input_bit_register_65 = 1
                    self.con.send(self.internal)
                else:
                    continue

                while not self.prog_running:
                    self.state = self.con.receive()
                    if self.state is None:
                        self.writeout("Conn lost")
                        break
                    time.sleep(0.01)

                self.internal.input_bit_register_64 = 0
                self.internal.input_bit_register_65 = 0
                self.con.send(self.internal)

            if self.record and (not (program_counter % 3)):
                self.rec_positions.append(self.state.actual_TCP_pose)
                self.rec_joint_angles.append(self.state.actual_q)
                self.rec_prints.append(self.printing)
                self.rec_speeds.append(self.state.actual_TCP_speed)
                self.rec_tspeeds.append(self.state.target_TCP_speed)

            # Check if tasks are queued
            if len(self.tasks) < 1:
                task_type, task_args = None, None
                if control_ack and home_ack:
                    self.writeout("")
                    self.writeout("")
                    self.writeout("TASKS ALL DONE!")
                    break
            else:
                task_type, task_args = self.tasks[0]  # if len(self.tasks) > 0 else None, None

            if self.prog_running:
                # Send a task...

                if task_type == "gantry":
                    self.gantry.input_bit_register_74 = task_args[0]
                    self.gantry.input_bit_register_75 = task_args[1]
                    self.con.send(self.gantry)

                    # Pop task from the list
                    self.tasks = self.tasks[1:]

                elif task_type == "home" and control_ack:
                    self.home.input_bit_register_76 = task_args[0]
                    self.con.send(self.home)

                    # Pop task from the list
                    self.tasks = self.tasks[1:]

                    home_ack = False
                    self.writeout("")
                    self.writeout("SENT CONTROL home")


                elif (control_ack) and home_ack:
                    if (self.current_task == 0) and (not self.task_active):
                        # Get the next fifo queue
                        # decide what to do
                        if task_type == "control":
                            self.positions.input_double_register_0 = task_args[1]
                            self.positions.input_double_register_1 = task_args[2]
                            self.positions.input_double_register_2 = task_args[3]
                            self.positions.input_double_register_3 = task_args[4]
                            self.positions.input_double_register_6 = task_args[5]
                            self.positions.input_double_register_9 = task_args[6]
                            self.con.send(self.positions)

                            self.control.input_int_register_0 = task_args[0]
                            self.con.send(self.control)

                            #self.last_task = self.tasks[0:]
                            self.tasks = self.tasks[1:]

                            control_ack = False
                            self.writeout("")
                            self.writeout("SENT CONTROL", self.name_task(task_args[0]))

                else:
                    if not home_ack and self.homed:
                        self.home.input_bit_register_76 = 0
                        self.con.send(self.home)
                        self.writeout("HOME ACK")
                        home_ack = True

                    elif not control_ack and (self.current_task != 0) and (not self.task_active) and (self.task_done):
                        self.control.input_int_register_0 = 0
                        self.con.send(self.control)
                        self.writeout("CONTROL ACK")
                        control_ack = True

            program_counter += 1
    
    def wrap_process(self):
        self.end()

        if self.record:
            #file_name = "data" + datetime.now().strftime(" %d-%b %H%M") + ".csv"
            file_name = "data.csv"
            print("Writing out to file " + file_name)
            with open(file_name, 'w', newline='') as f:
                writer = csv.writer(f)
                xy_head = ["x", "y", "z", "rx", "ry", "rz"]
                joint_head = ["q1", "q2", "q3", "q4", "q5", "q6"]
                speed_head = ["vx", "vy", "vz", "wx", "wy", "wz"]
                tspeed_head = ["vx_t", "vy_t", "vz_t", "wx_t", "wy_t", "wz_t"]
                writer.writerow(xy_head + joint_head + speed_head + tspeed_head + ["print"])
                for p, q, v, v_t, pr in zip(self.rec_positions, self.rec_joint_angles, self.rec_speeds, self.rec_tspeeds, self.rec_prints):
                    writer.writerow(p + q + v + v_t + [pr])

    def end(self):
        self.internal.input_bit_register_64 = 0
        self.internal.input_bit_register_65 = 0
        self.con.send(self.internal)

        # Close the connection
        self.con.send_pause()
        self.con.disconnect()


def print_coord_to_tasks(*print_coords: list, starting: int = 1, alternating: bool = True):
    pts = []
    print_coords_list = list(print_coords) if len(print_coords) > 1 else print_coords[0]
    next_control = starting
    for print_coord in print_coords_list:
        control_enum = ((next_control - 1) % 2) + 1 if alternating else starting
        pts.append(("control", [control_enum] + print_coord))
        next_control += 1
    return pts

def generate_coords(stack_format, side="A", perfect=True):
    # [X1, X2, X3, Y1, Y2, Y3, Z1, Z2, Z3]

    # Alternating every other layer
    frozen_config_func = lambda layers, not_flip: \
        [True ^ (not not_flip) if (x + 1) % 2 else False ^ (not not_flip) for x in range(layers)]

    # Side A has layers (n) of 1:n-2, n
    chilled_config_func = lambda layers, not_flip: \
        [False ^ (not not_flip) if (x + 2) == layers else True ^ (not not_flip) for x in range(layers)]


    sideA = side == "A"
    if stack_format == cartons_enum.frozen_small:
        cz, cx, cy = 527, 370, 115
        config = frozen_config_func(8, sideA)
    elif stack_format == cartons_enum.frozen_large:
        cz, cx, cy = 527, 370, 165
        config = frozen_config_func(6, sideA)
    elif stack_format == cartons_enum.chilled_small:
        cz, cx, cy = 527, 365, 115
        config = chilled_config_func(10, sideA)
    elif stack_format == cartons_enum.chilled_medium:
        cz, cx, cy = 527, 365, 177
        config = chilled_config_func(7, sideA)
    elif stack_format == cartons_enum.chilled_large:
        cz, cx, cy = 527, 365, 205
        config = chilled_config_func(6, sideA)
    elif stack_format == cartons_enum.testing:
        cz, cx, cy = 527, 365, 205
        config = frozen_config_func(5, sideA)
    else:
        raise ValueError("Invalid Configuration")

    coords = []
    y_coord = 100
    for layer, printing in enumerate(config):
        if layer >= 1:
            y_coord += cy

        if printing:
            x_coord = [-cx/2 - 40, cx/2 - 40, 1.5 * cx - 40]
            z_coord = 0
            coord = x_coord + [y_coord, z_coord, z_coord + 50]
            coords.append(coord)

    return [[y/1000 for y in x] for x in coords]

if __name__ == "__main__":
    #HOST, PORT = '12.10.11.21', 30004
    HOST, PORT = 'ursim', 30004

    # What do do for setup/teardown
    entry_tasks = [("gantry", [1, 0])]
    exit_tasks = [("home", [1]), ("gantry", [0, 1])]

    print_coords_a = generate_coords(carton, side="A", perfect=True)
    print_tasks_a = print_coord_to_tasks(print_coords_a, alternating=True)

    print_coords_b = generate_coords(carton, side="B", perfect=True)
    print_tasks_b = print_coord_to_tasks(print_coords_b, alternating=True)

    # Combine tasks
    task_list = entry_tasks + print_tasks_a + exit_tasks
    #task_list += entry_tasks + print_tasks_b + exit_tasks
    print("TASK QUEUE:")
    for task in task_list:
        print(task)
    print("")


    # Try and run the process
    try:
        robo = UR10_RTDE(HOST, PORT, 'portmark.xml', record=True)
        for task in task_list:
            robo.add_task(task)

        start = time.time()
        robo.process()
        end = time.time()
        print(f"Cycle time is {end - start}")
    except KeyboardInterrupt as keyexc:
        print("Keyboard interrupt")

    finally:
        robo.wrap_process()
