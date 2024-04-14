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
    data_dic = {} #[node, due, monitor, family, start, complete]
    for i in node_keys:
        data_dic[i] = [i, instance[1][i-1], instance[2][i-1], instance[6][i-1], 1000, 1000]
    idle = instance[7][-1][0]
    assignments = list(np.zeros([len(instance[4]), instance[3]], dtype=int))
    up_bound = 0
    for i in data_dic.keys():
        up_bound += data_dic[i][1]
    data_dic = dict(sorted(data_dic.items(), key=lambda item: item[1][1]))
    to_assign = list(data_dic.keys())
    times = {}

    print(to_assign)
    while to_assign:
        nn = to_assign.pop(0)
        possible_list = possibles(node = nn,assigned = assignments, data_dic=data_dic, idle=idle)
        drone = random.choice([i for i in range(len(possible_list[1])) if possible_list[1][i]])
        slot = possible_list[0][drone]
        assignments[drone][slot] = int(nn)
        # update_sc(instance=instance, assignments=assignments, data_dic=data_dic, idle=idle)
    evaluate_assignment(instance, assignments, data_dic)

# def update_sc(instance, assignments, data_dic, idle):
#     for i in range(len(assignments)):
#         for j in range(len(assignments)):
#             c_time[i, j] = data_dic[assignments[i][j]][2] + c_time[i, j - 1] + instance[0][i - 1][
#                 assignments[i][j] - 1]
#             s_time[i, j] = data_dic[assignments[i][j]][0] - c_time[i, j]

def possibles(node, assigned, data_dic, idle):
    pos_list = list(np.zeros(np.size(assigned, 0), dtype=int))
    pos_list_bin = list(np.ones(np.size(assigned, 0), dtype=int))
    for i in range(ndrones):
        pos_list[i] = np.argmin(assigned[i])
    for i in range(len(pos_list)):
        if pos_list[i]:
            if data_dic[assigned[i][int(pos_list[i])-1]][3] == data_dic[node][3]:
                print('these two nodes are from the same family')
                pos_list_bin[i] = 0
    return pos_list, pos_list_bin, assigned


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
    ndrones = 3
    condition = 'mini_fixed'
    inst = generate(ndrones, condition)
    greedy_drone(instance=inst)

    # # constraint 20:
    # for f in families:
    #     for j in f:
    #         m.cons20.add(sum(m.v[j + 1, r, i] - m.v[j, r, i] for r in slot_set for i in drones_set) <= i_times)
    # # constraint 21:
    # for f in families:
    #     for j in f:
    #         m.cons21.add(sum(m.v[j + 1, r, i] - m.v[j, r, i] for r in slot_set for i in drones_set) >= 0)
    # # constraint 22 & 23:
    # for f in families:
    #     for j in f:
    #         m.cons22.add(sum(m.e[j + 1, r, i] for r in slot_set for i in drones_set) >= sum(
    #             m.v[j, r, i] for r in slot_set for i in drones_set))
    #         # m.cons23.add(sum(m.e[j+1, r, i] - m.v[j, r, ii] for r in slot_set for ii in drones_set-{i}) >= 0)