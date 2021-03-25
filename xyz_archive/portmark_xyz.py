import sys
import logging
import csv
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from datetime import datetime
from enum import Enum
from functools import reduce

logging.basicConfig(level=logging.INFO)
logging.getLogger().setLevel(logging.INFO)


class UR10_RTDE():
    keep_running = True
    tasks = []

    controls = Enum('controls', 'home left2right right2left')

    current_task = None
    task_active = None
    task_done = None
    printing = None

    __state = None
    positions = []

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, state):
        current_task = state.output_int_register_0
        task_active = state.output_bit_register_64
        task_done = state.output_bit_register_65
        printing = state.output_bit_register_67

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
            self.writeout("PRINTING", self.printing, "->", printing)
            self.printing = printing

        self.__state = state

    def __init__(self, robo_host: str, robo_port: int, config_filename: str, record: bool = False):
        conf = rtde_config.ConfigFile(config_filename)
        self.record = record

        # get recipes!
        self.state_names, self.state_types = conf.get_recipe('state')
        self.control_names, self.control_types = conf.get_recipe('control')
        self.gantry_names, self.gantry_types = conf.get_recipe('gantry')
        self.print_names, self.print_types = conf.get_recipe('print')

        # connect, get controller version
        self.con = rtde.RTDE(robo_host, robo_port)
        self.con.connect()
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(self.state_names, self.state_types)
        self.control = self.con.send_input_setup(self.control_names, self.control_types)
        self.gantry = self.con.send_input_setup(self.gantry_names, self.gantry_types)
        self.printer = self.con.send_input_setup(self.print_names, self.print_types)

    def name_task(self, value: int):
        if (value is None) or (value is 0):
            return "None"
        return self.controls(value).name

    def writeout(self, *msg: str):
        current_time = datetime.now().strftime("%H:%M:%S.%f")
        print(current_time + ":", *msg)

    def add_task(self, task: tuple):
        self.tasks += [task]

    def set_defaults(self):
        # next task
        self.control.input_int_register_0 = 0
        self.con.send(self.control)

        self.gantry.input_bit_register_64 = 1
        self.gantry.input_bit_register_65 = 1
        self.con.send(self.gantry)

        self.printer.input_double_register_0 = 0
        self.printer.input_double_register_1 = 0
        self.printer.input_double_register_2 = 0
        self.printer.input_double_register_3 = 0
        self.printer.input_double_register_4 = 0
        self.printer.input_double_register_5 = 0
        self.printer.input_double_register_6 = 0
        self.printer.input_double_register_7 = 0
        self.printer.input_double_register_8 = 0
        self.con.send(self.printer)

    def begin(self):
        # Start data synchronization
        if not self.con.send_start():
            sys.exit()

    def process(self):
        self.begin()
        self.set_defaults()

        program_counter = 0
        control_ack = True
        while True:
            # Check the connection
            self.state = self.con.receive()
            if self.state is None:
                self.writeout("Conn lost")
                break

            if self.record and (not (program_counter % 3)):
                self.positions.append(self.state.actual_TCP_pose)

            # Check if tasks are queued
            if len(self.tasks) < 1:
                self.writeout("tasks done!")
                break

            # Send a task...
            if (self.current_task == 0) and (not self.task_active) and (control_ack):
                # Get the next fifo queue
                task_type, task_args = self.tasks[0]
                self.writeout("TASK TYPE:", task_type)

                # decide what to do
                if task_type == "control":
                    self.control.input_int_register_0 = task_args
                    self.con.send(self.control)
                    control_ack = False
                    self.writeout("")
                    self.writeout("SENT CONTROL", self.name_task(task_args))

                elif task_type == "print":
                    self.printer.input_double_register_0 = task_args[0]
                    self.printer.input_double_register_1 = task_args[1]
                    self.printer.input_double_register_2 = task_args[2]
                    self.printer.input_double_register_3 = task_args[3]
                    self.printer.input_double_register_4 = task_args[4]
                    self.printer.input_double_register_5 = task_args[5]
                    self.printer.input_double_register_6 = task_args[6]
                    self.printer.input_double_register_7 = task_args[7]
                    self.printer.input_double_register_8 = task_args[8]
                    self.con.send(self.printer)

                elif task_type == "gantry":
                    self.gantry.input_bit_register_64 = task_args[0]
                    self.gantry.input_bit_register_65 = task_args[1]
                    self.con.send(self.gantry)

                else:
                    raise ValueError("task type")

                # Pop task from the list
                self.tasks = self.tasks[1:]

            elif (self.current_task != 0) and (not self.task_active) and (self.task_done):
                self.control.input_int_register_0 = 0
                self.con.send(self.control)
                control_ack = True

            program_counter += 1

    def wrap_process(self):
        self.set_defaults()
        self.end()

        if self.record:
            file_name = "data" + datetime.now().strftime(" %d-%b %H%M") + ".csv"
            with open(file_name, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z", "rx", "ry", "rz"])
                writer.writerows(self.positions)

    def end(self):
        # Close the connection
        self.con.send_pause()
        self.con.disconnect()


def print_coord_to_tasks(*print_coords: list, starting: int = 2, alternating: bool = True):
    pts = []
    print(print_coords, len(print_coords))
    print_coords_list = list(print_coords) if len(print_coords) > 1 else print_coords[0]
    for print_coord in print_coords_list:
        pts.append(("print", print_coord))
        control_enum = (starting % 2) + 2 if alternating else starting
        pts.append(("control", control_enum))
        starting += 1
    return pts


def generate_coords(stack_format, side="A", perfect=True):
    # [X1, X2, X3, Y1, Y2, Y3, Z1, Z2, Z3]
    frozen_config_func = lambda layers, not_flip: [True ^ (not not_flip) if (x + 1) % 2 else False ^ (not not_flip) for
                                                   x in range(layers)]
    chilled_config_func = lambda layers, not_flip: [
        False ^ (not not_flip) if (x + 2) == layers else True ^ (not not_flip) for x in range(layers)]

    if stack_format.value == 1:
        # Frozen small~
        cz, cx, cy = 527, 370, 115
        config = frozen_config_func(8, side == "A")
    elif stack_format.value == 2:
        # Frozen large~
        cz, cx, cy = 527, 370, 165
        config = frozen_config_func(6, side == "A")
    elif stack_format.value == 3:
        # Chilled small~
        cz, cx, cy = 527, 365, 115
        config = chilled_config_func(10, side == "A")
    elif stack_format.value == 4:
        # Chilled medium~
        cz, cx, cy = 527, 365, 177
        config = chilled_config_func(7, side == "A")
    elif stack_format.value == 5:
        # Chilled large~
        cz, cx, cy = 527, 365, 205
        config = chilled_config_func(6, side == "A")
    else:
        raise ValueError("Bad type")

    coords = []
    for layer, printing in enumerate(config):
        if printing:
            x_coord = [-1 * cx, 0, 1 * cx]
            y_coord = [layer * cy, layer * cy, layer * cy]
            z_coord = [0, 0, 0]
            coords.append([x / 1000 for x in reduce(lambda a, b: a + b, list(zip(x_coord, y_coord, z_coord)))])

    return coords


if __name__ == "__main__":

    # What do do for setup/teardown
    entry_tasks = [("gantry", [True, False]), ("control", 1)]
    exit_tasks = [("control", 1), ("gantry", [False, True])]

    # What to do inbetween
    cartons_enum = Enum('cartons', 'frozen_small frozen_large chilled_small chilled_medium chilled_large')
    print_coords = generate_coords(cartons_enum.chilled_medium, side="A", perfect=True)
    print_tasks = print_coord_to_tasks(print_coords)

    # Combine tasks
    task_list = entry_tasks + print_tasks + exit_tasks

    # Try and run the process
    try:
        robo = UR10_RTDE('ursim', 30004, 'portmark_)odl.xml', record=True)
        for task in task_list:
            robo.add_task(task)
        robo.process()
    except KeyboardInterrupt as keyexc:
        print("Keyboard interrupt")
    except Exception as e:
        print(e)
    finally:
        robo.wrap_process()