from pyorca import RVOSimulator
import numpy as np
import irsim

def set_preferred_velocities(sim, goals):
    for i in range(sim.get_num_agents()):
        goal_vector = np.array(goals[i]) - np.array(sim.get_agent_position(i))

        if np.linalg.norm(goal_vector)**2 > 1.0:
            goal_vector = goal_vector / np.linalg.norm(goal_vector)

        goal_vector = tuple(goal_vector)

        sim.set_agent_pref_velocity(i, goal_vector)

        angle = np.random.rand() * np.pi * 2.0 / 0x7fff
        dist = np.random.rand() * 0.0001 / 0x7fff

        pre_v = np.array(sim.get_agent_pref_velocity(i))
        angle_vector = np.array([np.cos(angle), np.sin(angle)])
        vel = pre_v + dist * angle_vector
        vel_tuple = tuple(vel)

        sim.set_agent_pref_velocity(i, vel_tuple)


def reached_goal(sim, goals):
    for i in range(sim.get_num_agents()):

        pos = np.array(sim.get_agent_position(i))
        goal = np.array(goals[i])

        if np.linalg.norm(pos - goal)**2 > 400.0:
            return False
    return True


sim = RVOSimulator(0.25, 15.0, 10, 5.0, 5.0, 2.0, 2.0)
goals = []

for i in range(5):
    for j in range(5):
        sim.add_agent((55.0 + i * 10.0, 55.0 + j * 10.0))
        sim.add_agent((-55.0 - i * 10.0, 55.0 + j * 10.0))
        sim.add_agent((55.0 + i * 10.0, -55.0 - j * 10.0))
        sim.add_agent((-55.0 - i * 10.0, -55.0 - j * 10.0))

        goals.append((-75.0, -75.0))
        goals.append((75.0, -75.0))
        goals.append((-75.0, 75.0))
        goals.append((75.0, 75.0))


obstacle1 = [(10.0, 40.0), (40.0, 40.0), (40.0, 10.0), (10.0, 10.0)]
obstacle2 = [(10.0, 40.0), (10.0, 10.0), (40.0, 10.0), (40.0, 40.0)]
obstacle3 = [(10.0, -40.0), (40.0, -40.0), (40.0, -10.0), (10.0, -10.0)]
obstacle4 = [(10.0, -40.0), (10.0, -10.0), (40.0, -10.0), (40.0, -40.0)]

sim.add_obstacle(obstacle1)
sim.add_obstacle(obstacle2)
sim.add_obstacle(obstacle3)
sim.add_obstacle(obstacle4)

sim.process_obstacles()

env = irsim.make()


while not reached_goal(sim, goals):



    set_preferred_velocities(sim, goals)
    sim.do_step()

    



    print(sim.get_agent_position(0))




