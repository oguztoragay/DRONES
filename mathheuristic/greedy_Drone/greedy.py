import numpy as np
import random
from itertools import chain
from random_instance import generate

def greedy_drone(instance):
    print('----------------------- Greedy Drone -----------------------')
    # 0=t_matrix, 1=due_date, 2=monitor_times, 3=slots, 4=charges, 5=i_times, 6=membership, 7=families, 8=f
    due_date = instance[1]
    monitor_times = instance[2]
    families = instance[7]
    slots = instance[3]
    # assignments = {}
    assignments = []
    for i in range(len(instance[4])):
        assignments.append(np.zeros([slots]))
    kk = random_drone(instance, assignments)
    up_bound = sum(due_date)
    evaluate_assignment(instance, kk)

def evaluate_assignment(instance, assignments):
    c_time = np.zeros(assignments.shape)
    tightness = np.zeros(assignments.shape)
    due = list([instance[1][-1]])
    due = due +[i for i in instance[1][1:]]
    for i in range(assignments.shape[0]):
        for j in range(assignments.shape[1]):
            c_time[i,j] = instance[2][assignments[i][j]-1] + c_time[i,j-1] + instance[0][i-1][assignments[i][j]-1]
            tightness[i,j] = due[assignments[i][j]-1] - c_time[i,j]
    # print('A', assignments)
    # print('C', c_time)
    # print('L:', tightness)
    l_max = -1*tightness.min()
    print('l_max', l_max)


def random_drone(instance, assignments):
    due_date = instance[1]
    total_slots = instance[3]*len(instance[4])
    nodes = list(chain.from_iterable(instance[7]))
    nodes.extend([instance[7][-1][0]] * (total_slots-len(due_date)))
    random.shuffle(nodes)
    assignments = np.reshape(nodes, (len(instance[4]), instance[3]))
    return assignments

if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    # random.seed(1)
    ndrones = 3
    condition = 'mini_fixed'
    inst = generate(ndrones, condition)
    greedy_drone(instance=inst)