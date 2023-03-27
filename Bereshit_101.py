'''
This class represents the basic flight controller of the Bereshit spacecraft.
'''

import math
import Moon

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.last_error = 0
        self.last_time = 0

    def compute(self, error, time):
        dt = time - self.last_time
        de = error - self.last_error
        self.last_error = error
        self.last_time = time
        if dt == 0:
            return 0.5
        if self.kp * error + self.kd * de / dt > 1:
            return 1
        elif self.kp * error + self.kd * de / dt < 0:
            return 0
        else:
            return self.kp * error + self.kd * de / dt

# All this variable type is float.
WEIGHT_EMP = 165  # kg
WEIGHT_FULE = 420  # kg
WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE  # kg
# https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
MAIN_ENG_F = 430  # N
SECOND_ENG_F = 25  # N
MAIN_BURN = 0.15  # liter per sec, 12 liter per m'
SECOND_BURN = 0.009  # liter per sec 0.6 liter per m'
ALL_BURN = MAIN_BURN + 8 * SECOND_BURN


def accMax(weight: float) -> float:
    return acc_fun(weight, True, 8)


def acc_fun(weight: float, main: bool, seconds: int) -> float:
    t = 0
    if main:
        t += MAIN_ENG_F
    t += seconds * SECOND_ENG_F
    ans = t / weight
    return ans


# 14095, 955.5, 24.8, 2.0
if __name__ == '__main__':
    print("Simulating Bereshit's Landing:")
    # starting point:
    vs = 24.8
    hs = 932
    dist = 181 * 1000
    ang = 58.3  # zero is vertical (as in landing)
    alt = 13748  # 2:25:40 (as in the simulation) # https://www.youtube.com/watch?v=JJ0VfRL9AMs
    time = 0
    dt = 1  # sec
    acc = 0  # Acceleration rate (m/s^2)
    fuel = 121  #
    weight = WEIGHT_EMP + fuel
    print("time, vs, hs, dist, alt, ang,weight,acc")
    NN = 0.7  # rate[0,1]
    # ***** main simulation loop ******
    pdconrtoller = PDController(0.4, 0.2)
    speeds = []
    alts = []
    times = []
    while alt > 0:
        speeds.append(vs)
        alts.append(alt)

        times.append(time)
        if time % 10 == 0 or alt < 100:
            print("time\t, vs\t, hs\t, dist\t\t, alt\t, ang\t,weight\t\t,acc")
            print("%.2f" % time, "\t, %.2f" % vs, ", %.2f" % hs, ", %.2f" % dist, "\t, %.2f" % alt, ", %.2f" % ang, "\t, %.2f" % weight, "\t, %.2f" % acc)
        # over 2 km above the ground
        if alt > 2000:  # maintain a vertical speed of [20-25] m/s
            NN = pdconrtoller.compute(vs - 20, time)
            # if vs > 25:
            #   NN += 0.003 * dt
            # # more power for braking
            # if vs < 20:
            #     NN -= 0.003 * dt
            # less power for braking

        # lower than 2 km - horizontal speed should be close to zero
        else:
            if ang > 3:
                ang -= 3
            # rotate to vertical position.
            else:
                ang = 0
            # NN = 0.5  # brake slowly, a proper PID controller here is needed!
            # NN = pdconrtoller.compute(vs, time)
            if hs < 2:
                hs = 0
            if alt > 200:  # close to the ground
                NN = pdconrtoller.compute(vs-10, time)
            if alt > 100:
                NN = pdconrtoller.compute(vs-5, time)
            else:
                NN = pdconrtoller.compute(vs-2, time)

                # if vs < 5:
                #     NN = 0.7  # if it is slow enough - go easy on the brakes
                #     # NN = pdconrtoller.compute(vs, time)

        if alt < 10:  # no need to stop
            # NN = 0.4
            NN = pdconrtoller.compute(vs-.1, time)


        # main computations
        ang_rad = math.radians(ang)
        h_acc = math.sin(ang_rad) * acc
        v_acc = math.cos(ang_rad) * acc
        vacc = Moon.getAcc(hs)
        time += dt
        dw = dt * ALL_BURN * NN

        if fuel > 0:
            fuel -= dw
            weight = WEIGHT_EMP + fuel
            acc = NN * accMax(weight)
        else:  # ran out of fuel
            acc = 0

        v_acc -= vacc

        if hs > 0:
            hs -= h_acc * dt

        dist -= hs * dt
        vs -= v_acc * dt
        alt -= dt * vs
    print("time\t, vs\t, hs\t, dist\t\t, alt\t, ang\t,weight\t\t,acc")
    print("%.2f" % time, "\t, %.2f" % vs, ", %.2f" % hs, ", %.2f" % dist, "\t, %.2f" % alt, ", %.2f" % ang, "\t, %.2f" % weight, "\t, %.2f" % acc)

    # use matplot lib to plot the results
    # speed over time
    import matplotlib.pyplot as plt

    plt.plot(times, speeds)
    plt.xlabel('time')
    plt.ylabel('Speed')
    plt.title('Speed vs time')
    plt.show()

    # altitude over time
    plt.plot(times, alts)
    plt.xlabel('time')
    plt.ylabel('Altitude')
    plt.title('Altitude vs time')
    plt.show()