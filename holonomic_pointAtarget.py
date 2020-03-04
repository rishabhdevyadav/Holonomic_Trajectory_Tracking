
import math
import sys
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
import pdb

dt = 0.01
Kp_lin = 10.0 
Kp_ang=10.0
Kp_track=10.0
Kp_point_head=1.0
show_animation = True

import cubic_spline_planner


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0,omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega=omega

def update(state, vx_ref,vy_ref,w_ref,a_lin,a_ang):
    state.x=state.x + vx_ref * dt 
    state.y=state.y + vy_ref * dt 
    state.yaw=state.yaw + w_ref*dt
    state.yaw=pi_2_pi(state.yaw)
    state.v = state.v + a_lin* dt
    state.omega=state.omega + a_ang*dt
    return state

def PIDControl(Kp,target, current):
    ac = Kp_lin * (target - current)
    return ac

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_nearest_index(state, cx, cy, cyaw):
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)
    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y
    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1
    #return ind, mind
    return ind


def plot_arrow(x, y, yaw, length=1, width=2, fc="r", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
         plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
            fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

# def speed_reduce_curvature(state,target_ind,ck,target_linearspeed,target_angularspeed):
#     if math.tan(ck[target_ind])<=0.1:
#         target_linearspeed=target_linearspeed*1
#         target_angularspeed=target_angularspeed*1
#         a_linear = PIDControl(Kp_lin,target_linearspeed, state.v)
#         a_angular=PIDControl(Kp_ang,target_angularspeed,state.omega)

#     if math.tan(ck[target_ind])>=0.9:
#         target_linearspeed=target_linearspeed*0.7
#         target_angularspeed=target_angularspeed*0.7
#         a_linear = PIDControl(Kp_lin,target_linearspeed, state.v)
#         a_angular=PIDControl(Kp_ang,target_angularspeed,state.omega)

#     if math.tan(ck[target_ind])>0.1 and math.tan(ck[target_ind])<0.9:
#         curv = [math.tan(0.1), math.tan(0.9)]
#         curve_k = [0.95,0.7]
#         speed_k=np.interp(math.tan(ck[target_ind]), curv, curve_k)
#         target_linearspeed=target_linearspeed*speed_k
#         target_angularspeed=target_angularspeed*speed_k
#         a_linear = PIDControl(Kp_lin,target_linearspeed, state.v)
#         a_angular=PIDControl(Kp_ang,target_angularspeed,state.omega)
#     return state,a_linear,a_angular

#def PID_Error(Kp, error):
#    return spd = Kp*error

def yaw_point(state,xp, yp):
    yaw_new=(np.rad2deg(pi_2_pi(state.yaw)))
    if yaw_new<0:
        yaw_new=yaw_new+360

    dy=-state.y+yp
    dx=-state.x+xp
    theta = pi_2_pi(math.atan2(dy, dx))
    theta=np.rad2deg(theta)
    if theta<0:
        theta=theta+360

    error=(theta-yaw_new)
    if error > 180:
        error=error-360
    if error <-180:
        error =error +360
    e_th=np.deg2rad(error)
    return error

def PIDcontroller(state,cx,cy,cyaw,ck,target_ind,target_linearspeed,target_angularspeed):
    target_ind=calc_nearest_index(state, cx, cy, cyaw)
    ck=np.absolute(ck)*[10]
    #state,a_linear,a_angular=speed_reduce_curvature(state,target_ind,ck,target_linearspeed,target_angularspeed)
    a_linear = PIDControl(Kp_lin,target_linearspeed, state.v)
    a_angular=PIDControl(Kp_ang,target_angularspeed,state.omega)
    vx_ref=state.v*np.cos(cyaw[target_ind])
    vy_ref=state.v*np.sin(cyaw[target_ind])

    PID_spd_x=(PIDControl(Kp_track,cx[target_ind+2]+0.5,state.x))
    PID_spd_y=(PIDControl(Kp_track,cy[target_ind+2]+0.5,state.y))
    #PID_spd_w=(PID_Error(Kp_point_head,yaw_point(state, 0, 0)))
    PID_spd_w=Kp_point_head*yaw_point(state, 25, 25)

    vx_ref=vx_ref+PID_spd_x
    vy_ref=vy_ref+PID_spd_y
    w_ref=state.omega + PID_spd_w

    return vx_ref, vy_ref,w_ref,target_ind,a_linear,a_angular

def main():
    ax = [0,10,20,30,40,50]
    ay = [math.sin(ix / 5.0) * ix / 2.0 for ix in ax]

    goal = [ax[-1], ay[-1]]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.05)
    target_linearspeed=20
    target_angularspeed = 0
    T = 100.0  # max simulation time
    state = State(x=0, y=0, yaw=0.0, v=0.0,omega=0.0) #intial state
    lastIndex = len(cx) - 1
    time = 0.0
    x,y,yaw,v = [state.x],[state.y],[state.yaw],[state.v]
    t,w1,w2,w3,w4,vx,vy,w = [0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]
   
    target_ind = calc_nearest_index(state, cx, cy,cyaw)

    while T >= time and lastIndex > target_ind+8:
        vx_ref, vy_ref,w_ref,target_ind,a_linear,a_angular= PIDcontroller(state,cx,cy,cyaw,ck,target_ind,target_linearspeed,target_angularspeed)
        state = update(state, vx_ref,vy_ref,w_ref,a_linear,a_angular)

        L=0.4+0.4
        J_inv = np.array([[1,-1,-L/2], [1,1,L/2],[1,1,-L/2],[1,-1,L/2]]) 
        in_vel = np.array([[vx_ref], [vy_ref], [w_ref]])
        wheel_velocity=np.matmul(J_inv,in_vel)

        w1.append(wheel_velocity[0,0])
        w2.append(wheel_velocity[1,0])
        w3.append(wheel_velocity[2,0])
        w4.append(wheel_velocity[3,0])
        vx.append(vx_ref)
        vy.append(vy_ref)
        w.append(w_ref)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            point1 = [25, 25]
            point2 = [state.x, state.y]
            x_values = [point1[0], point2[0]]
            y_values = [point1[1], point2[1]]
            plt.plot(x_values, y_values, 'g--')
            
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")  # show the desired trajectory
            plt.plot(x, y, "-b", label="trajectory") #show tracked trajectory
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v)[:4])
            plt.pause(0.001)        

    assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        #plt.show()
    o_stack = np.column_stack((w1,w2,w3,w4))
    v_stack = np.column_stack((vx,vy,w))
    a=np.array(zip(v_stack,o_stack))
    #print(a)

if __name__ == '__main__':
    main()
