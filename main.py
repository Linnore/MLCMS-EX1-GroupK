import tkinter as tk
from tkinter import ttk
import time
import threading
import math
import random
import numpy.random as random2
from tkinter import messagebox

PEDESTRIAN_COLOUR = "#BA0A1B"
TARGET_COLOUR = "#9ADB54"
OBSTACLE_COLOUR = "#FEEEEE"
BACKGROUND_COLOUR = "#000000"
HIGHLIGHT_COLOUR = "#FF00FF"
MEASURING_AREA_COLOUR = "#FFFF00"


# Status code
EMPTY = 0
PEDESTRIAN = 1
TARGET = 2
OBSTACLE = 3

# Cost function code
EUCLIDEAN = 1
DIJKSTRA = 2

DIRECTIONS = [(0, -1), (-1, 0), (1, 0), (0, 1), (0, 0)]

DEFAULT_SCALE = 8

# Each Cell: 1/3m x 1/3m
NUM_OF_UNIT_LEN_PER_METER = 3


class Pedestrian:
    def __init__(self, x, y, speed=1.33, age=20):
        self.x = x
        self.y = y

        self.isMoving = False
        self.new_x = 0
        self.new_y = 0

        self.age = age

        self.speed = speed  # m/s
        self.lag = 1/(speed*3)  # s/0.33m
        self.clock = 0

        self.isInMA = False  # Flag that whether the pd is in a speed measuring area
        self.coord_enterMA = tuple()  # Coordinates where the pd enters the MA
        self.coord_leaveMA = tuple()  # Coordinates where the pd leaves the MA
        self.time_enter = float()
        self.time_leave = float()

        self.time_reach_target = float()
        self.distance_to_target = float()

        self.removed = False  # Flag that whether the pd reaches the target and thus is removed


class Cellular_Automaton:

    def __init__(self, width, height, scale):
        self.width = width
        self.height = height
        self.scale = scale

        self.running = True  # flag of whether the window is running
        self.paused = True  # flag of whether the simulation is paused
        self.delay = .1  # control crowd.isMoving speed
        self.now = 0

        self.initState()
        self.initCost()
        self.initMeasuringArea()

        self.RiMEA_test_mode = 0

        # By default, the mouse click on canvas does nothing
        # Adding mode will be changed by buttons in "Adding" part
        self.crt_adding_mode = EMPTY

        self.RiMEA_test1_counter = 50
        self.RiMEA_test1_results = []
        self.RiMEA_test6_counter = 50
        self.RiMEA_test6_results = []
        self.RiMEA_test7_pd = []
        self.RiMEA_test7_counter = 10

        self.cost_func = DIJKSTRA

        self.gui = Cellular_Automaton_GUI(self, width, height, scale)

        self.start()

    def initState(self):
        self.population = []
        self.coord_target = []
        self.crt_state = [[EMPTY] * self.height for i in range(self.width)]

    def initCost(self):
        self.dist_cost = [[self.width*self.height] *
                          self.height for i in range(self.width)]
        self.avd_cost = [[0] * self.height for i in range(self.width)]

        self.rmax = 1  # 1m
        self.dmax = self.rmax * NUM_OF_UNIT_LEN_PER_METER
        self.avoid_cost_by_diff = [[0] * self.dmax for i in range(self.dmax)]
        for i in range(self.dmax):
            for j in range(self.dmax+1-abs(i)):
                if j != self.dmax:
                    dx = i/3
                    dy = j/3
                    self.avoid_cost_by_diff[i][j] = math.exp(
                        1/((dx**2+dy**2)-self.rmax**2))

        # Assign large cost to r=0 cases where pedestrians collide
        self.avoid_cost_by_diff[0][0] = self.width+self.height

    def initMeasuringArea(self):
        # Whether a given cell is in a speed measuring area
        self.isMA = [[False] * self.height for i in range(self.width)]
        # Measuring Area Configuration. Will be set up in RiMEA test 4
        self.MA_L_Boundary = int()
        self.MA_R_Boundary = int()
        self.MA_U_Boundary = int()
        self.MA_D_Boundary = int()
        self.speed_measured_pd_list = []

    def start(self):
        self.t = threading.Thread(target=self.clock)
        self.t.start()
        self.gui.window.mainloop()

    def toggle_status(self, event):
        if self.paused:
            x = event.x//self.scale
            y = event.y//self.scale

            if x >= 0 and x < self.width and y >= 0 and y < self.height:
                if self.crt_adding_mode != self.crt_state[x][y]:
                    if self.crt_state[x][y] == PEDESTRIAN:
                        self.remove_pedestrian(x, y)
                    elif self.crt_state[x][y] == TARGET:
                        self.coord_target.remove((x, y))

                    if self.crt_adding_mode == PEDESTRIAN:
                        self.add_pedestrian(Pedestrian(x, y))
                    elif self.crt_adding_mode == TARGET:
                        self.add_target(x, y)
                    else:
                        self.add_obstacle(x, y)

    def clock(self):
        while self.running:
            if not self.paused:
                self.now += self.delay
                self.gui.timer.configure(text=str(round(self.now, 2)))
                self.updateState()

            time.sleep(self.delay)

    def updateState(self):
        for crt_pd in self.population:
            # Speed control
            crt_pd.clock += self.delay
            if crt_pd.clock < crt_pd.lag - 0.0001:
                continue  # remain unmoved
            else:
                crt_pd.clock -= crt_pd.lag  # reset clocking of current pedestrian

            x = crt_pd.x
            y = crt_pd.y

            # Remove the avoidance cost that currrent pd contributes to itself
            self.remove_avoidance_cost(x, y)

            # Find the neighbor with lowest cost
            minCost = 2147483647
            new_x = 0
            new_y = 0
            for (dx, dy) in DIRECTIONS:
                if x+dx >= 0 and x+dx < self.width and y+dy >= 0 and y+dy < self.height:
                    if self.crt_state[x+dx][y+dy] != TARGET:
                        total_cost = self.dist_cost[x+dx][y +
                                                          dy] + self.avd_cost[x+dx][y+dy]
                    else:
                        total_cost = 0
                    if minCost > total_cost:
                        new_x = x+dx
                        new_y = y+dy
                        minCost = total_cost

            # Recover avoidance cost contributed by current pd
            self.add_avoidance_cost(x, y)

            if new_x != x or new_y != y:
                crt_pd.isMoving = True
                crt_pd.new_x = new_x
                crt_pd.new_y = new_y

        for crt_pd in self.population:
            if crt_pd.isMoving:
                crt_pd.isMoving = False
                x = crt_pd.x
                y = crt_pd.y
                new_x = crt_pd.new_x
                new_y = crt_pd.new_y

                if self.crt_state[new_x][new_y] == PEDESTRIAN:
                    # This means a collision will occur. So the crt pd has to stop.
                    continue

                # After 5s, start speed measuring for RiMEA test 4
                if self.now - 5 >= 0.00001:

                    # Enter the measuring area from left boundary
                    if not self.isMA[x][y] and self.isMA[new_x][new_y]:
                        if new_x == self.MA_L_Boundary and x == new_x-1:
                            crt_pd.isInMA = True
                            crt_pd.coord_enterMA = (new_x, new_y)
                            crt_pd.time_enterMA = self.now
                    # Leave the MA from right boundary
                    if self.isMA[x][y] and not self.isMA[new_x][new_y] and crt_pd.isInMA:
                        crt_pd.isInMA = False
                        if x == self.MA_R_Boundary and x == new_x-1:
                            crt_pd.coord_leaveMA = (new_x, new_y)
                            crt_pd.time_leaveMA = self.now
                            self.speed_measured_pd_list.append(crt_pd)
                        # Leave the MA from other boundaries
                        else:
                            crt_pd.coord_enterMA = tuple()
                            crt_pd.time_enterMA = float()

                # Update GUI and avoidance cost
                self.remove_avoidance_cost(x, y)
                self.crt_state[x][y] = EMPTY

                if self.isMA[x][y]:
                    self.gui.changeColour(x, y, MEASURING_AREA_COLOUR)
                else:
                    self.gui.changeColour(x, y, BACKGROUND_COLOUR)

                if self.crt_state[new_x][new_y] == EMPTY:
                    self.gui.changeColour(new_x, new_y, PEDESTRIAN_COLOUR)
                    self.crt_state[new_x][new_y] = PEDESTRIAN
                    self.add_avoidance_cost(new_x, new_y)
                    crt_pd.x, crt_pd.y = new_x, new_y
                elif self.crt_state[new_x][new_y] == TARGET:
                    crt_pd.removed = True  # This pd reaches the target and will disappear
                    crt_pd.time_reach_target = self.now

        # Remove those "removed-mark" pedestrians
        i = 0
        l = len(self.population)
        while i < l:
            if self.population[i].removed == True:
                self.population.pop(i)
                l -= 1
            else:
                i += 1
        # Stop if all pedestrians reach the target.
        if len(self.population) == 0:
            self.paused = True

        if self.RiMEA_test_mode == 1 and self.RiMEA_test1_counter > 0 and len(self.population) == 0:
            self.paused = True

            self.RiMEA_test1_results.append(self.now)
            self.now = 0
            self.RiMEA_test1_counter -= 1
            self.gui.click_button_test1()
            self.gui.click_button_run()

            if self.RiMEA_test1_counter == 0:
                self.output_test1_results()
                self.RiMEA_test_mode = 0
                self.RiMEA_test1_counter = 2


        if self.RiMEA_test_mode == 4 and (self.now > 35 or len(self.population) == 0):
            self.paused = True
            self.output_test4_results()
            self.RiMEA_test_mode = 0

        if self.RiMEA_test_mode == 6 and self.RiMEA_test6_counter > 0 and len(self.population) == 0:
            self.paused = True
            self.RiMEA_test6_counter -= 1
            self.gui.click_button_test6()
            self.gui.click_button_run()

            self.RiMEA_test6_results.append(self.now)
            self.now = 0
            if self.RiMEA_test6_counter == 0:
                self.output_test6_results()
                self.RiMEA_test_mode = 0
                self.RiMEA_test6_counter = 50

        if self.RiMEA_test_mode == 7 and len(self.population) == 0:
            self.output_test7_results(self.RiMEA_test7_counter)
            self.RiMEA_test7_counter -= 1
            if self.RiMEA_test7_counter == 0:
                self.RiMEA_test_mode = 0
                self.RiMEA_test7_counter = 10
            else:
                self.gui.click_button_test7()
                self.gui.click_button_run()

    def get_euclidean_distance(self, dist):
        for i in range(self.width):
            for j in range(self.height):
                dist[i][j] = self.width + self.height
        for (tar_x, tar_y) in self.coord_target:
            for x in range(self.width):
                for y in range(self.height):
                    if self.crt_state[x][y] != OBSTACLE:
                        tmp = ((x-tar_x)**2 + (y-tar_y)**2)**0.5
                        if dist[x][y] > tmp:
                            dist[x][y] = tmp

    def get_dijkstra_distance(self, dist):
        for i in range(self.width):
            for j in range(self.height):
                dist[i][j] = self.width + self.height

        neighbors = [(0, self.coord_target)]

        for (distance, neighbor_list) in neighbors:
            for x, y in neighbor_list:
                if dist[x][y] > distance and self.crt_state[x][y] != OBSTACLE:
                    dist[x][y] = distance
                    neighbors.append(
                        (distance + 1/3, self.get_neighbors(x, y)))

    def get_neighbors(self, x, y):
        neighbors = []
        for (dx, dy) in DIRECTIONS:
            if x+dx >= 0 and x+dx < self.width and y+dy >= 0 and y+dy < self.height:
                neighbors.append((x+dx, y+dy))
        return neighbors

    def output_test1_results(self):
        min = float('inf')
        max = float('-inf')
        f = open("output/Test1_results.txt", "w")

        f.write("RiMEA Test Case 1: Moving straight with different speeds (" + str(
            len(self.RiMEA_test1_results)) + " Iterations)" + "\n" + "\n")

        for i in self.RiMEA_test1_results:
            if (i < min):
                min = i
            if (i > max):
                max = i

            speed = 40/i
            f.write("Time: "+str(i) + " Speed: "+str(speed)+"m/s"+"\n")

        avg = sum(self.RiMEA_test1_results) / len(self.RiMEA_test1_results)
        speedmin = 40/min
        f.write("\n" + "minimal time: " + str(min) +
                ", maximal Speed: "+str(speedmin)+"m/s" + "\n")
        speedavg = 40 / avg
        f.write("average time: " + str(avg) +
                ", average Speed: "+str(speedavg)+"m/s" + "\n")
        speedmax = 40 / max
        f.write("maximal time: " + str(max) +
                ", minimal Speed: "+str(speedmax)+"m/s" + "\n")

        f.close()

    def output_test4_results(self):
        numOfpd = len(self.speed_measured_pd_list)
        if numOfpd == 0:
            with open(r'./output/Test4_results'+str(self.gui.test4_density)+'.txt', 'w') as output:
                output.write(
                    "Number of pedestrians that pass through the measuring area: 0\n")
                output.write("Average speed: 0m/s\n")

            return

        speedList = []
        for pd in self.speed_measured_pd_list:
            speedList.append(self.hamiltonDist(
                pd.coord_enterMA, pd.coord_leaveMA)/((pd.time_leaveMA-pd.time_enterMA)*3))
        avg_speed = sum(speedList)/numOfpd
        with open('Test4_results'+str(self.gui.test4_density)+'.txt', 'w') as output:
            output.write(
                "Number of pedestrians that pass through the measuring area: %d\n" % numOfpd)
            output.write("Average speed: %.2fm/s\n" % avg_speed)
            output.write("Pedestrian list:\n")
            for i in range(numOfpd):
                output.write("Pedestrian %d, %.2fm/s, from (%d, %d) to (%d, %d), consumed %.2fs\n" % (
                    i+1, speedList[i], self.speed_measured_pd_list[i].coord_enterMA[0], self.speed_measured_pd_list[i].coord_enterMA[1],
                    self.speed_measured_pd_list[i].coord_leaveMA[0], self.speed_measured_pd_list[i].coord_leaveMA[1],
                    self.speed_measured_pd_list[i].time_leaveMA-self.speed_measured_pd_list[i].time_enterMA))

    def output_test6_results(self):
        min = float('inf')
        max = float('-inf')
        f = open("output/Test6_results.txt.txt", "w")

        f.write("RiMEA Test Case 6:.isMoving around a corner (" + str(
            len(self.RiMEA_test6_results)) + " Iterations)" + "\n" + "\n")

        for i in self.RiMEA_test6_results:
            if (i < min):
                min = i
            if (i > max):
                max = i
            f.write(str(i) + "\n")

        avg = sum(self.RiMEA_test6_results) / len(self.RiMEA_test6_results)

        f.write("\n" + "minimal value:" + str(min) + "\n")
        f.write("average value:" + str(avg) + "\n")
        f.write("maximum value:" + str(max) + "\n")

        f.close()

    def output_test7_results(self, run):
        with open(".\output\Test7_results_"+str(run)+"_.txt", "w") as output:
            output.write(
                "No.\tAge\tAssigned_Speed\tSimulated_Speed\tDistance\tTime\n")
            i = 1
            for pd in self.RiMEA_test7_pd:
                output.write("%d\t%d\t%.4f\t%.4f\t%.2f\t%.2f\n" % (
                    i, pd.age, pd.speed, pd.distance_to_target/pd.time_reach_target, pd.distance_to_target, pd.time_reach_target))
                i += 1
        pass

    def hamiltonDist(self, p, q):
        return abs(p[0]-q[0]) + abs(p[1]-q[1])

    def add_avoidance_cost(self, x, y):
        for dx in range(-self.dmax+1, self.dmax, 1):
            for dy in range(-self.dmax+abs(dx), self.dmax+1-abs(dx), 1):
                if abs(dy) != self.dmax:
                    if x+dx >= 0 and x+dx < self.width and y+dy >= 0 and y+dy < self.height:
                        self.avd_cost[x+dx][y +
                                            dy] += self.avoid_cost_by_diff[abs(dx)][abs(dy)]

    def remove_avoidance_cost(self, x, y):
        for dx in range(-self.dmax+1, self.dmax, 1):
            for dy in range(-self.dmax+abs(dx), self.dmax+1-abs(dx), 1):
                if abs(dy) != self.dmax:
                    if x+dx >= 0 and x+dx < self.width and y+dy >= 0 and y+dy < self.height:
                        self.avd_cost[x+dx][y +
                                            dy] -= self.avoid_cost_by_diff[abs(dx)][abs(dy)]

    def add_pedestrian(self, pd):
        self.population.append(pd)
        self.crt_state[pd.x][pd.y] = PEDESTRIAN
        self.gui.changeColour(pd.x, pd.y, PEDESTRIAN_COLOUR)
        self.add_avoidance_cost(pd.x, pd.y)

    def add_obstacle(self, x, y):
        self.crt_state[x][y] = OBSTACLE
        self.gui.changeColour(x, y, OBSTACLE_COLOUR)

    def add_target(self, x, y):
        self.coord_target.append((x, y))
        self.crt_state[x][y] = TARGET
        self.gui.changeColour(x, y, TARGET_COLOUR)

    def add_measuring_area(self, l, r, u, d):
        self.MA_L_Boundary = l
        self.MA_R_Boundary = r
        self.MA_U_Boundary = u
        self.MA_D_Boundary = d
        for x in range(l, r+1, 1):
            for y in range(u, d+1, 1):
                self.isMA[x][y] = True
                self.gui.changeColour(x, y, MEASURING_AREA_COLOUR)

    def remove_pedestrian(self, x, y):
        self.crt_state[x][y] = EMPTY
        self.gui.changeColour(x, y, BACKGROUND_COLOUR)
        self.population = list(
            filter(lambda p: p.x == x and p.y == y, self.population))
        self.remove_avoidance_cost(x, y)


class Cellular_Automaton_GUI:
    def __init__(self, CA, width, height, scale):
        self.CA = CA
        self.width = width
        self.height = height
        self.scale = scale

        self.control_panel_width = 300
        self.control_panel_height = 400
        self.window = tk.Tk()
        self.window.title("MLCMS_Group_K_Cellular_Automaton")
        self.window.geometry("{}x{}".format(
            self.control_panel_width + self.CA.width*self.scale,
            max(self.control_panel_height, self.CA.height*self.scale)))
        self.window.resizable(False, False)

        # Lefthandside will be the control panel
        self.frm_left = tk.Frame(self.window)
        self.frm_left.grid(row=0, column=0, sticky='NWSE')

        # Righthandside will be the canvas
        self.frm_right = tk.Frame(self.window)
        self.frm_right.grid(row=0, column=1, sticky='E')

        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_columnconfigure(1, weight=1)

        self.canvas = tk.Canvas(
            self.frm_right,
            width=self.CA.width*self.scale, height=self.CA.height*self.scale
        )
        self.canvas.grid(row=0, column=0, columnspan=4, sticky="E")
        self.canvas.pack()
        self.makeGrid()
        # Mouse clicks/draws on the canvas will toggle cells' status
        self.canvas.bind("<Button-1>", lambda a: self.CA.toggle_status(a))
        self.canvas.bind("<B1-Motion>", lambda a: self.CA.toggle_status(a))

        self.numOfRows = 0
        self.create_setting_part()
        self.create_adding_part()

        self.title_cost = tk.Label(
            self.frm_left, text='Cost Function', font='Helvetica 10 bold')
        self.title_cost.grid(row=self.numOfRows, column=0,
                             columnspan=4, sticky='W')
        self.numOfRows += 1

        self.cost_func = tk.IntVar()
        # Use the Euclidean distance as cost function by default
        self.cost_func.set(DIJKSTRA)

        self.euclidean_radiobutton = tk.Radiobutton(
            self.frm_left, text="Euclidean", variable=self.cost_func, value=EUCLIDEAN, command=self.click_radiobutton_cost_func)
        self.euclidean_radiobutton.grid(
            row=self.numOfRows, column=1, sticky='NWSE')

        self.dijkstra_radiobutton = tk.Radiobutton(
            self.frm_left, text="Dijkstra", variable=self.cost_func, value=DIJKSTRA, command=self.click_radiobutton_cost_func)
        self.dijkstra_radiobutton.grid(
            row=self.numOfRows, column=2, sticky='NWSE')
        self.numOfRows += 1

        self.create_preset_task_part()
        self.create_RiMEA_part()
        self.create_control_bar()

        self.window.protocol("WM_DELETE_WINDOW", self.close)

    #
    def close(self):
        self.CA.running = False
        self.window.destroy()
        quit()

    def create_setting_part(self):
        """
        Create everything about the "Setting" part in our gui.
        """
        self.title_setting = tk.Label(
            self.frm_left, text='Setting', font='Helvetica 10 bold')
        self.title_setting.grid(row=self.numOfRows, column=0, sticky='W')
        self.numOfRows += 1

        self.label_CA_size = tk.Label(self.frm_left, text='Size of the CA:')
        self.label_CA_size.grid(row=self.numOfRows, column=1, sticky='W')

        self.text_CA_size = tk.Entry(self.frm_left, width=10)
        self.text_CA_size.insert(0, '{}*{}'.format(self.width, self.height))
        self.text_CA_size.grid(row=self.numOfRows, column=2, sticky='W')

        self.button_reset = tk.Button(
            self.frm_left, text='Reset', command=self.click_button_reset)
        self.button_reset.grid(row=self.numOfRows, column=3, sticky='W')
        self.numOfRows += 1

    def create_adding_part(self):
        """
        Create everything about the "Adding" part in our gui
        """
        self.title_adding = tk.Label(
            self.frm_left, text='Click the Button and Draw', font='Helvetica 10 bold', anchor='w')
        self.title_adding.grid(
            row=self.numOfRows, column=0, columnspan=4, sticky='W')
        self.numOfRows += 1

        self.button_pedestrian = tk.Button(
            self.frm_left, text='Add Pedestrian', command=self.click_button_add_pedestrian, bg=PEDESTRIAN_COLOUR)
        self.button_pedestrian.grid(
            row=self.numOfRows, column=0, columnspan=4, sticky='NWSE')
        self.numOfRows += 1

        self.button_obstacle = tk.Button(
            self.frm_left, text="Add Obstacle", command=self.click_button_add_obstacle, bg=OBSTACLE_COLOUR)
        self.button_obstacle.grid(
            row=self.numOfRows, column=0, columnspan=4, sticky='NWSE')
        self.numOfRows += 1

        self.button_target = tk.Button(
            self.frm_left, text="Add Target", command=self.click_button_add_target, bg=TARGET_COLOUR)
        self.button_target.grid(
            row=self.numOfRows, column=0, columnspan=4, sticky='NWSE')
        self.numOfRows += 1

    def create_preset_task_part(self):
        self.title_preset_task = tk.Label(
            self.frm_left, text='Preset Tasks', font='Helvetica 10 bold', anchor='w')
        self.title_preset_task.grid(
            row=self.numOfRows, column=0, columnspan=4, sticky='W')
        self.numOfRows += 1

        self.button_task2 = tk.Button(
            self.frm_left, text='Task 2', command=self.click_button_task2)
        self.button_task2.grid(row=self.numOfRows, column=1, sticky='NWSE')

        self.button_task3 = tk.Button(
            self.frm_left, text='Task 3', command=self.click_button_task3)
        self.button_task3.grid(row=self.numOfRows, column=2, sticky='NWSE')

        self.button_task4 = tk.Button(
            self.frm_left, text='Task 4', command=self.click_button_task4)
        self.button_task4.grid(row=self.numOfRows, column=3, sticky='NWSE')
        self.numOfRows += 1

    def create_RiMEA_part(self):
        self.title_RiMEA = tk.Label(
            self.frm_left, text='RiMEA Tests', font='Helvetica 10 bold', anchor='w')
        self.title_RiMEA.grid(
            row=self.numOfRows, column=0, columnspan=4, sticky='W')
        self.numOfRows += 1

        self.button_test1 = tk.Button(
            self.frm_left, text='Test 1', command=self.click_button_test1)
        self.button_test1.grid(row=self.numOfRows, column=0, sticky='NWSE')

        self.button_test4 = tk.Button(
            self.frm_left, text='Test 4', command=self.click_button_test4)
        self.button_test4.grid(row=self.numOfRows, column=1, sticky='NWSE')

        self.button_test6 = tk.Button(
            self.frm_left, text='Test 6', command=self.click_button_test6)
        self.button_test6.grid(row=self.numOfRows, column=2, sticky='NWSE')

        self.button_test7 = tk.Button(
            self.frm_left, text='Test 7', command=self.click_button_test7)
        self.button_test7.grid(row=self.numOfRows, column=3, sticky='NWSE')
        self.numOfRows += 1

    def create_control_bar(self):
        self.title_control = tk.Label(
            self.frm_left, text="Control", font='Helvetica 10 bold', anchor='w')
        self.title_control.grid(row=self.numOfRows, column=0, sticky='W')
        self.numOfRows += 1

        self.run_button = tk.Button(
            self.frm_left, text="Run", command=self.click_button_run)
        self.run_button.grid(row=self.numOfRows, column=1, sticky='NWSE')

        self.pause_button = tk.Button(
            self.frm_left, text="Pause", command=self.click_button_pause)
        self.pause_button.grid(row=self.numOfRows, column=2, sticky='NWSE')

        self.timer = tk.Label(self.frm_left, text="0.0")
        self.timer.grid(row=self.numOfRows, column=3, sticky='NWSE')
        self.numOfRows += 1

    def click_button_reset(self, new_scale=DEFAULT_SCALE):

        new_width, new_height = map(int, self.text_CA_size.get().split('*'))
        self.resizeCA(new_width, new_height, new_scale)
        self.CA.initState()
        self.CA.initCost()
        self.CA.initMeasuringArea()
        self.CA.now = 0

    def click_button_add_pedestrian(self):
        self.CA.crt_adding_mode = PEDESTRIAN

    def click_button_add_obstacle(self):
        self.CA.crt_adding_mode = OBSTACLE

    def click_button_add_target(self):
        self.CA.crt_adding_mode = TARGET

    def click_radiobutton_cost_func(self):
        self.CA.cost_func = self.cost_func.get()

    def click_button_run(self):
        self.CA.paused = False
        if self.CA.cost_func == EUCLIDEAN:
            self.CA.get_euclidean_distance(self.CA.dist_cost)
        elif self.CA.cost_func == DIJKSTRA:
            self.CA.get_dijkstra_distance(self.CA.dist_cost)

    def click_button_pause(self):
        self.CA.paused = True
        self.CA.now -= self.CA.delay

    def click_button_task2(self):
        self.text_CA_size.delete(0, 'end')
        self.text_CA_size.insert(0, '50*50')
        self.click_button_reset()
        self.CA.add_pedestrian(Pedestrian(4, 24))
        self.CA.add_target(24, 24)

    def click_button_task3(self):
        self.text_CA_size.delete(0, 'end')
        self.text_CA_size.insert(0, '50*50')
        self.click_button_reset()

        self.CA.add_pedestrian(Pedestrian(18, 0))
        self.CA.add_pedestrian(Pedestrian(40, 10))
        self.CA.add_pedestrian(Pedestrian(49, 29))
        self.CA.add_pedestrian(Pedestrian(19, 49))
        self.CA.add_pedestrian(Pedestrian(0, 30))
        self.CA.add_target(24, 24)

    def click_button_task4(self):
        self.text_CA_size.delete(0, 'end')
        self.text_CA_size.insert(0, '50*50')
        self.click_button_reset()

        for x in range(20, 30, 1):
            self.CA.add_obstacle(x, 20)
        for y in range(20, 30, 1):
            self.CA.add_obstacle(30, y)
        for x in range(30, 19, -1):
            self.CA.add_obstacle(x, 30)

        self.CA.add_pedestrian(Pedestrian(25, 25))
        self.CA.add_pedestrian(Pedestrian(22, 27))
        self.CA.add_pedestrian(Pedestrian(27, 27))
        self.CA.add_target(40, 25)

    def click_button_test1(self):

        if(self.CA.RiMEA_test1_counter == 50):

            self.text_CA_size.delete(0, 'end')
            self.text_CA_size.insert(0, '140*20')
            self.click_button_reset()
        else:
            self.CA.now = 0

        self.CA.RiMEA_test_mode = 1
        for x in range(20, 120):
            self.CA.add_obstacle(x, 12)
            self.CA.add_obstacle(x, 6)
        for y in range(6, 13):
            self.CA.add_obstacle(19, y)
            self.CA.add_obstacle(120, y)
        for y in range(7, 12):
            self.CA.add_target(119, y)

        age = 20
        y = random.randint(7, 11)
        speed = random.uniform(4.5/3.6, 5.1/3.6)  # Random Pedestrian Speed

        # this is basically rescaling our unit
        speed = speed*(33/40)
        self.CA.add_pedestrian(Pedestrian(20, y, speed, age))

    def test1_callback(self):
        if tk.messagebox.askokcancel("Quit", "Do you really wish to quit?"):
            self.window_test1.destroy()

    def click_button_test4(self):
        self.text_CA_size.delete(0, 'end')
        self.text_CA_size.insert(0, '153*21')  # 150*15 i.e. 50m*5m
        self.click_button_reset(new_scale=4)

        self.CA.RiMEA_test_mode = 4

        for x in range(self.CA.width):
            self.CA.add_obstacle(x, 2)
            self.CA.add_obstacle(x, self.CA.height-3)

        for y in range(9, 12):
            self.CA.add_target(self.CA.width-2, y)

        for y in range(2, 9):
            self.CA.add_obstacle(self.CA.width-3, y)

        for y in range(12, self.CA.height-3):
            self.CA.add_obstacle(self.CA.width-3, y)

        self.window_test4 = tk.Tk()
        self.window_test4.title("RiMEA_Test4_Configuration")
        self.window_test4.geometry("500x100")
        self.window_test4.resizable(False, False)
        self.window_test4.protocol("WM_DELETE_WINDOW", self.test4_callback)

        self.density_list = ["Select Density", "0.5 P/m^2", "1 P/m^2",
                             "2 P/m^2", "3 P/m^2", "4 P/m^2", "5 P/m^2", "5.5 P/m^2", "6 P/m^2", "6.5 P/m^2", "7 P/m^2", "8 P/m^2"]
        self.combobox_density = ttk.Combobox(
            self.window_test4, values=self.density_list)
        self.combobox_density.pack()
        self.combobox_density.current(0)

        self.button_confirm_density = tk.Button(
            self.window_test4, text="Confirm", command=self.click_button_density_confirm)
        self.button_confirm_density.pack()

        self.CA.add_measuring_area(l=98, r=102, u=8, d=12)

    def test4_callback(self):
        if tk.messagebox.askokcancel("Quit", "Do you really wish to quit?"):
            self.window_test4.destroy()

    def click_button_density_confirm(self):
        # if self.combobox_density.get()
        self.test4_density = float(
            self.combobox_density.get().split(" P/m^2")[0])

        numOfPd = int(self.test4_density * 50 * 5)

        pd_cell_labels = random.sample(range(150*15), numOfPd)
        pd_coord = []
        for cell_label in pd_cell_labels:
            x = cell_label % 150
            y = cell_label // 150 + 3
            self.CA.add_pedestrian(Pedestrian(x, y))

        self.window_test4.destroy()

    def click_button_test6(self):

        self.CA.RiMEA_test_mode = 6

        if(self.CA.RiMEA_test6_counter == 50):

            # resetting canvas
            self.cost_func.set(DIJKSTRA)

            self.text_CA_size.delete(0, 'end')
            self.text_CA_size.insert(0, '50*50')
            self.click_button_reset()

            # adding the corner with its boundaries
            for x in range(7, 42):
                self.CA.add_obstacle(x, 42)
            for y in range(7, 43):
                self.CA.add_obstacle(42, y)
            for x in range(7, 36):
                self.CA.add_obstacle(x, 35)
            for y in range(7, 35):
                self.CA.add_obstacle(35, y)
            for y in range(35, 43):
                self.CA.add_obstacle(6, y)

            # targets/ "finish line"
            for x in range(36, 42):
                self.CA.add_target(x, 7)

        # adding the corner with its boundaries
        for x in range(7, 42):
            self.CA.add_obstacle(x, 42)
        for y in range(7, 43):
            self.CA.add_obstacle(42, y)
        for x in range(7, 36):
            self.CA.add_obstacle(x, 35)
        for y in range(7, 35):
            self.CA.add_obstacle(35, y)
        for y in range(35, 43):
            self.CA.add_obstacle(6, y)

        #targets/ "finish line"
        for x in range(36, 42):
            self.CA.add_target(x, 7)

        # uniform distribution of pedesterians in starting area

        pedestrian_coords = []
        for x in range(7, 24):
            for y in range(36, 42):
                pedestrian_coords.append(Pedestrian(x, y))

        pedestrians = random.sample(pedestrian_coords, 20)

        for p in pedestrians:
            self.CA.add_pedestrian(p)

        pass

    def click_button_test7(self):

        self.CA.RiMEA_test_mode = 7

        self.text_CA_size.delete(0, 'end')
        self.text_CA_size.insert(0, '153*56')
        self.click_button_reset(new_scale=4)

        for x in range(self.CA.width):
            self.CA.add_obstacle(x, 2)
            self.CA.add_obstacle(x, self.CA.height-3)

        for y in range(3, self.CA.height-3):
            self.CA.add_target(self.CA.width-3, y)

        # Generate random starting positions for the sample

        # pedestrian_coords = []
        # for x in range(50):
        #     for y in range(3, self.CA.height-3):
        #         pedestrian_coords.append((x, y))

        #pos = random.sample(pedestrian_coords, 50)
        pos = []
        for y in range(3, self.CA.height-3):
            pos.append((3, y))

        # Generate random age sample with size=50 from N(50, 20^2)
        age_sample = random2.normal(50, 20, 50)

        # Read in speed for coorespending age
        speed_info = {}
        with open("speed_age_config.txt", "r") as speed_config:
            for line in speed_config.readlines():
                info = line.split(" ")
                age = int(info[0])
                mu = float(info[1])
                sigma = float(info[2])
                speed_info[age] = (mu, sigma)

        # Generate pedestrians according to RiMEA guideline figure 2

        self.CA.test7_pd = []
        for i in range(50):
            age = int(age_sample[i])
            if age < 5:
                age = 5
            elif age > 80:
                age = 80
            speed = random2.normal(speed_info[age][0], speed_info[age][1])
            pd = Pedestrian(pos[i][0], pos[i][1], speed, age)
            pd.distance_to_target = (self.CA.width-3-pos[i][0])/3
            self.CA.RiMEA_test7_pd.append(pd)
            self.CA.add_pedestrian(pd)

        # pass

    def changeColour(self, x, y, col):
        self.canvas.itemconfig(self.cells[(x, y)], fill=col)

    def resizeCA(self, nwidth, nheight, nscale=DEFAULT_SCALE):
        self.CA.width = nwidth
        self.CA.height = nheight
        self.scale = self.CA.scale = nscale
        if nheight >= 50:
            self.height = nheight

        self.window.geometry("{}x{}".format(
            self.control_panel_width + self.CA.width*self.scale,
            max(self.control_panel_height, self.height*self.scale)))

        self.frm_right.config(width=nwidth*self.scale,
                              height=nheight*self.scale)
        self.canvas.config(width=nwidth*self.scale, height=nheight*self.scale)

        self.makeGrid()

    def makeGrid(self):
        """
        Construct the grid in the canvas such that each cell has specified scale.
        """
        self.cells = {}

        for x in range(0, self.CA.width):
            for y in range(0, self.CA.height):
                self.cells[(x, y)] = self.canvas.create_rectangle(
                    x*self.scale,
                    y*self.scale,
                    (x*self.scale)+self.scale,
                    (y*self.scale)+self.scale,
                    fill=BACKGROUND_COLOUR,
                    activefill=HIGHLIGHT_COLOUR)


main = Cellular_Automaton(50, 50, DEFAULT_SCALE)
