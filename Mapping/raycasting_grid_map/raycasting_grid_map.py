"""

Ray casting 2D grid map example

author: Atsushi Sakai (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt

EXTEND_AREA = 15.0

show_animation = True


def calc_grid_map_config(ox, oy, xyreso, agent_x,agent_y):
    minx = round(min(ox) - EXTEND_AREA / 2.0)
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))
    # print("minx:"+str(minx))
    # print("miny:"+str(miny))
    # print("maxx:"+str(maxx))
    # print("maxy:"+str(maxy))
    # print("xw:"+str(xw))
    # print("yw:"+str(yw))

    return minx, miny, maxx, maxy, xw, yw


class precastDB:

    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle


def precasting(minx, miny, xw, yw, xyreso, yawreso):

    precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]
    num_angleid = int(round((math.pi * 2.0) / yawreso))
    print("num_angleid"+str(num_angleid))
    # print(precast)

    for ix in range(xw):
        for iy in range(yw):
            #get x,y coordinates
            px = ix * xyreso + minx
            py = iy * xyreso + miny

            #distance from robot(origin)
            d = math.sqrt(px**2 + py**2)
            angle = atan_zero_to_twopi(py, px)
            angleid = int(math.floor(angle / yawreso))

            pc = precastDB()

            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle

            precast[angleid].append(pc)

    return precast

def generate_ray_casting_grid_map(ox, oy, xyreso, yawreso, agent_x=0.0, agent_y=0.0,  agent_yaw=0.0):

    minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso, agent_x, agent_y )
    #make grid map with xw, yw
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    
    precast = precasting(minx, miny, xw, yw, xyreso, yawreso)
    # print(precast)

    #if agent angle is 0 
    fov_type, fov_anglelist = gnerate_fov_angles(math.pi/2, math.pi)
    print(len(fov_anglelist))

    # generate fov grid
    num_angleid = int(round((math.pi * 2.0) / yawreso))
    for angleid in range(num_angleid):
        angle = angleid*yawreso
        #fov_type means the angle range 
        if fov_type==0:
            if angle <= fov_anglelist[1] and angle >=fov_anglelist[0]:
                print(str(angleid)+" , "+str(angle))
                angleidx = int(math.floor(angle / yawreso))
                gridlist = precast[angleid]
                #case for in fov
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 0.0
            else:
                #case for out of fov
                gridlist = precast[angleid]
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 1.0
        else:
            if angle <= fov_anglelist[0] or angle >=fov_anglelist[1]:
                print(str(angleid)+" , "+str(angle))
                angleidx = int(math.floor(angle / yawreso))
                gridlist = precast[angleid]
                #case for in fov
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 0.0
            else:
                #case for out of fov
                gridlist = precast[angleid]
                for grid in gridlist:
                    pmap[grid.ix][grid.iy] = 1.0


        # print(angle)

   #managing obstacles
    for (x, y) in zip(ox, oy):

        d = math.sqrt(x**2 + y**2)
        angle = atan_zero_to_twopi(y, x)
        angleid = int(math.floor(angle / yawreso))

        gridlist = precast[angleid]
        # for grid in gridlist:
            # print("grid x "+str(grid.ix)+ ", grid y : " + str(grid.iy))

        ix = int(round((x - minx) / xyreso))
        iy = int(round((y - miny) / xyreso))

        for grid in gridlist:
            if grid.d > d:
                pmap[grid.ix][grid.iy] = 0.5

        pmap[ix][iy] = 0.0

    return pmap, minx, maxx, miny, maxy, xyreso


#shoulde be written in terms of radian
def gnerate_fov_angles(agent_yaw, fov_range):

    fov_type=0
    min_angle = agent_yaw-fov_range/2.0
    max_angle = agent_yaw+fov_range/2.0
    fov_angles=[]


    if max_angle >2*math.pi:
        fov_type=1
        max_angle-=2*math.pi
        temp = min_angle
        min_angle = max_angle
        max_angle - min_angle
        
    fov_angles.append(min_angle)
    fov_angles.append(max_angle)

    return fov_type, fov_angles
        

def draw_heatmap(data, minx, maxx, miny, maxy, xyreso):
    x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                    slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
    plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
    plt.axis("equal")


def main():
    print(__file__ + " start!!")

    xyreso = 0.25  # x-y grid resolution [m]
    yawreso = math.radians(5)  # yaw angle resolution [rad]
    agent_yaw= math.pi/2
    num_obs=3

    for i in range(num_obs):
        # ox = [-2.02]
        # oy = [0.023]
        ox = (np.random.rand(num_obs) - 0.5) * 10.0
        oy = (np.random.rand(num_obs) - 0.5) * 10.0
        # print("ox")
        # print(ox)
        # print("oy")
        # print(oy)
        pmap, minx, maxx, miny, maxy, xyreso = generate_ray_casting_grid_map(
            ox, oy, xyreso, yawreso, agent_yaw)
        if show_animation:
            plt.cla()
            draw_heatmap(pmap, minx, maxx, miny, maxy, xyreso)
            plt.plot(ox, oy, "xr")
            plt.plot(0.0, 0.0, "ob")
            plt.pause(1.0)
        input("enter to continue")


if __name__ == '__main__':
    main()
