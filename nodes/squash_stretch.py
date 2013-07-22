#!/usr/bin/env python
from dmp_interface import *
import pylab as plt
import scipy as sp


def create_demo_traj_squash(size, dim=1):
    if dim == 1:
        x = np.linspace(0, np.pi / 2.0, size)
        y = abs(np.cos(x))
    else:
        x = np.linspace(np.pi / 2.0, np.pi, size)
        y = abs(np.cos(x))

    return y.tolist()


def create_demo_arc(size):
    x = np.linspace(0, 1, size)
    return (np.exp(x ** 2) / np.sqrt(2 * np.pi)).tolist()


def create_demo_secondary_action(size):
    pass


def create_demo_acticipation(size):
    pass


def squash_and_stretch():
    y1 = create_demo_traj_squash(500, 1)
    y2 = create_demo_traj_squash(500, 2)
    # y2 = create_demo_arc(500)

    dims = 2
    dt = 1.0
    K = 50
    D = 2.0 * np.sqrt(K)
    num_bases = 50

    assert(len(y1) == len(y2))
    traj = [[y1[i], y2[i]] for i in xrange(len(y1))]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)
    makeSetActiveRequest(resp.dmp_list)

    print 'generating plan'
    # Now, generate a plan
    x_0 = [y1[0], y2[0]]  # Plan starting at a different point than demo
    x_dot_0 = [y1[0], y2[0]]
    # x_dot_0 = [y1[0], y1[-1]]
    t_0 = 0
    goal = [y1[-1], y2[-1]]  # Plan to a different goal than demo
    goal_thresh = [0.2, 0.2]
    seg_length = -1  # Plan until convergence to goal
    tau = 1.2 * resp.tau  # Desired plan should take twice as long as demo
    dt = 1.0
    integrate_iter = 5  # dt is rather large, so this is > 1
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    # aggregate reproduced trajectories
    rp1, rp2 = [], []
    rv1, rv2 = [], []
    # print plan

    reproduced_traj = plan.plan
    for p in reproduced_traj.points:
        rp1.append(p.positions[0])
        rv1.append(p.velocities[0])
        rp2.append(p.positions[1])
        rv2.append(p.velocities[1])

    plt.figure('squash_stretch dmp learning')
    plt.plot(y1 + y2, label='Demonstrated Policy')
    plt.plot(rp1 + rp2, label='Reproduced Policy (1.2*tau)')
    plt.legend(loc='lower right')

    plt.figure('velocities')
    plt.plot(rv1 + rv2, label='Velocities')
    plt.show()


def arc():
    y = create_demo_arc(500)

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
    goal = [y[-1]]  # Plan to a different goal than demo
    goal_thresh = [0.2]
    seg_length = -1  # Plan until convergence to goal
    tau = 0.8 * resp.tau  # Desired plan should take twice as long as demo
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

    plt.figure('Arc dmp learning')
    plt.plot(y, label='Demonstrated Policy')
    plt.plot(rp, label='Reproduced Policy (0.8*tau)')
    plt.legend(loc='lower right')

    plt.figure('Velocities')
    plt.plot(rv, label='Velocities')
    plt.show()


if __name__ == '__main__':
    rospy.init_node('squash_stretch')

    squash_and_stretch()
    arc()
