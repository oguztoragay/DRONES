import numpy as np
import random
from itertools import chain
from random_instance import generate

def greedy_drone(instance):
    print('----------------------- Greedy Drone -----------------------')
    fam_mat = {}
    for i in range(1,len(instance[7])-1):
        fam_mat[i] = instance[7][i]
    node_keys = list(range(2,instance[7][-1][0]+1))
    data_dic = {} #[node, due, monitor, family]
    for i in node_keys:
        data_dic[i] = [i, instance[1][i-1], instance[2][i-1], instance[6][i-1]]
    idle = instance[7][-1][0]
    assignments = np.ones([len(instance[4]), instance[3]], dtype=int)*idle
    assignments2 = list(np.zeros([len(instance[4]), instance[3]], dtype=int))
    kk = random_drone(instance, assignments)
    assignments2 = kk
    up_bound = 0
    for i in data_dic.keys():
        up_bound += data_dic[i][1]
    data_dic = dict(sorted(data_dic.items(), key=lambda item: item[1][1]))
    to_assign = list(data_dic.keys())
    print(to_assign)
    while to_assign:
        nn = to_assign.pop(0)
        possible_list = possibles(node = nn,assigned = assignments2, data_dic=data_dic)
        print(possible_list)
        dr = random.randint(0,assignments.shape[0]-1)
        slt = [i for i in range(len(assignments[dr])) if assignments[dr][i] == idle].pop(0)
        assignments[dr][slt] = int(nn)
    evaluate_assignment(instance, assignments, data_dic)


def possibles(node, assigned, data_dic):
    assigned = [[3, 0, 0, 0],[5, 2, 0, 0], [0, 0, 0, 0], [2, 4, 6, 0]]
    pos_list = list(np.zeros(np.size(assigned, 0), dtype=int))
    for i in range(ndrones):
        pos_list[i] = np.argmin(assigned[i])
    for i in range(len(pos_list)):
        if pos_list[i]:
            if data_dic[assigned[i][int(pos_list[i])-1]][-1] == data_dic[node][-1]:
                print('these two node from the same family')
    return pos_list




def evaluate_assignment(instance, assignments, data_dic):
    c_time = np.zeros(assignments.shape)
    tightness = np.zeros(assignments.shape)
    for i in range(assignments.shape[0]):
        for j in range(assignments.shape[1]):
            c_time[i,j] = data_dic[assignments[i][j]][2] + c_time[i,j-1] + instance[0][i-1][assignments[i][j]-1]
            tightness[i,j] = data_dic[assignments[i][j]][0] - c_time[i,j]
    print('A', assignments)
    print('C', c_time)
    print('L:', tightness)
    l_max = -1*tightness.min()
    print('l_max', l_max)


def random_drone(instance, assignments):
    due_date = instance[1]
    total_slots = instance[3]*len(instance[4])
    nodes = list(chain.from_iterable(instance[7]))
    nodes.extend([instance[7][-1][0]] * (total_slots-len(due_date)))
    random.shuffle(nodes)
    assigned = np.reshape(nodes, assignments.shape)
    return assigned

if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    random.seed(1)
    ndrones = 4
    condition = 'mini_fixed'
    inst = generate(ndrones, condition)
    greedy_drone(instance=inst)