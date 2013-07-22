#!/usr/bin/env python
from dmp_interface import *


def create_lab_demo(size):
    x = np.linspace(0, 4, size)
    ts = 0.2
    y = np.sin(2 * (x - ts)) + np.exp(-16 * (x - ts) ** 2)
    return y.tolist()

if __name__ == '__main__':
    rospy.init_node('simple_dmp_learn')

    import pylab as plt

    y = create_lab_demo(500)
    dims = 1
    dt = 1.0
    K = 50
    D = 2.0 * np.sqrt(K)
    num_bases = 50
    traj = [[y[i]] for i in xrange(len(y))]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)
    makeSetActiveRequest(resp.dmp_list)

    print 'generating plan'
    # Now, generate a plan
    x_0 = [y[0]]  # Plan starting at a different point than demo
    x_dot_0 = [y[0]]
    t_0 = 0
    # goal = [y[-1]]         #Plan to a different goal than demo
    goal = [0.5]
    goal_thresh = [0.2]
    seg_length = -1  # Plan until convergence to goal
    tau = 1.2 * resp.tau  # Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5  # dt is rather large, so this is > 1
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    # aggregate reproduced trajectories
    rp = []
    rv = []
    # print plan

    reproduced_traj = plan.plan
    for p in reproduced_traj.points:
        rp.append(p.positions[0])
        rv.append(p.velocities[0])

    plt.plot(y, label='Demonstrated Policy')
    plt.plot(rp, label='Reproduced Policy (1.2*tau)')
    plt.legend(loc='lower right')
    plt.show()

    # print plan



    # Create a DMP from a 3-D trajectory
    # dims = 3
    # dt = 1.0
    # K = 50
    # D = 2.0 * np.sqrt(K)
    # num_bases = 4
    # traj = [[1.0,1.0,1.0],[2.0,2.0,1.8],[3.0,4.0,4.5],[6.0,8.0,7.5]]
    # resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    # Set it as the active DMP
    # makeSetActiveRequest(resp.dmp_list)

    # Now, generate a plan
    # x_0 = [0.0,0.0,0.0]          #Plan starting at a different point than demo
    # x_dot_0 = [0.0,0.0,0.0]
    # t_0 = 0
    # goal = [8.0,7.0,6.0]         #Plan to a different goal than demo
    # goal_thresh = [0.2,0.2,0.2]
    # seg_length = -1          #Plan until convergence to goal
    # tau = 1.2 * resp.tau       #Desired plan should take twice as long as demo
    # dt = 1.0
    # integrate_iter = 5       #dt is rather large, so this is > 1
    # plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
    #                        seg_length, tau, dt, integrate_iter)

    # print plan
