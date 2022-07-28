from cProfile import label
import math
import matplotlib
import matplotlib.pyplot as plt
#%matplotlib inline

# set up matplotlib
is_ipython = 'inline' in matplotlib.get_backend()
if is_ipython:
    from IPython import display

plt.ion()


import numpy as np
from scipy.spatial.transform import Rotation as Rot

# Covariance for EKF simulation
# 过程噪声的协方差矩阵
R = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
#观测的协方差矩阵
Q = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.5, 0.5]) ** 2

dt = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

show_animation = True

# 作为输入控制线速度和角速度
def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s] 角速度
    u = np.array([[v], [yawrate]])
    return u


def observation(xTrue, xd, u):
    # 无噪声情况下认为是真实值
    xTrue = motion_model(xTrue, u)
    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud

# 运动模型，作为估计值
def motion_model(x, u):
    A = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[dt * math.cos(x[2, 0]), 0],
                  [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt],
                  [1.0, 0.0]])

    # @ 是矩阵乘法
    x = A @ x + B @ u

    return x

# 观测模型，只观测x,y位置
def observation_model(x):
    C = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    z = C @ x
    return z


# 状态矩阵求导，并不是Ax+Bu中的A，而是合在一起时的矩阵
# 或者说是Ax+Bu计算得到的矩阵
def jacob_f(x, u):
    yaw = x[2, 0]
    v = u[0, 0]
    G = np.array([
        [1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw)],
        [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return G

# 观测矩阵求导
def jacob_h():
    # Jacobian of Observation Model
    C = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return C

# 预测 +　更新
# param 
# @xEst  上一次的状态最优估计量
# @PEst  上一次的协方差矩阵更新
# @z     本次观测
# @u     本次输入
def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    # 状态矩阵的雅克比
    G = jacob_f(xEst, u)
    PPred = G @ PEst @ G.T + R

    #  Update
    C = jacob_h()
    zPred = observation_model(xPred)
    S = C @ PPred @ C.T + Q
    # 卡尔曼增益计算
    K = PPred @ C.T @ np.linalg.inv(S)
    #最优估计
    xEst = xPred + K @ (z - zPred)
    #协方差矩阵更新
    PEst = (np.eye(len(xEst)) - K @ C) @ PPred
    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def main():
    time = 0.0

    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue
    hz = np.zeros((2, 1))

    while SIM_TIME >= time:
        time += dt
        u = calc_input()#速度和角速度恒定

        # 产生数据
        xTrue, z, xDR, ud = observation(xTrue, xDR, u)

        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)

        # store data history
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz = np.hstack((hz, z))

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(hz[0, :], hz[1, :], ".g")
            plt.plot(hxTrue[0, :].flatten(),
                     hxTrue[1, :].flatten(), "-b",label='True pose')
            plt.plot(hxDR[0, :].flatten(),
                     hxDR[1, :].flatten(), "-k",label='pose with noise')
            plt.plot(hxEst[0, :].flatten(),
                     hxEst[1, :].flatten(), "-r",label='ekf est')
            plot_covariance_ellipse(xEst, PEst)
            plt.axis("equal")
            plt.grid(True)
            plt.legend()
            
            plt.show()
            
            plt.pause(0.001)
            # if is_ipython:
            #     display.clear_output(wait=True)
            #     display.display(plt.gcf())      
    plt.savefig('imu_gps_ekf_fusion.png')      
    
if __name__ == '__main__':
    main()


