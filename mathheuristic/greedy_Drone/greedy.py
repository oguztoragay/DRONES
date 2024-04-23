import numpy as np
import random
from itertools import chain
from random_instance import generate

#### Start and conclusion times defined for each visit node which are saved in data_dic
#### Times defined the start time and conclusion time of the visit assigned to (i,j) --> (drone, slot) position of the drone.
def greedy_drone(instance):
    print('----------------------- Greedy Drone -----------------------')
    fam_mat = {} # families = {1: [2, 3], 2: [4, 5, 6]}
    # SB families f = [[2], [4, 5], [7], [9, 10]]
    families = instance[6]
    for i in range(1,len(instance[7])-1):
        fam_mat[i] = instance[7][i]
    node_keys = list(range(2,instance[7][-1][0]+1))
    data_dic = {} #[node, due, monitor, family, start, complete]
    for i in node_keys:
        data_dic[i] = [i, instance[1][i-1], instance[2][i-1], instance[6][i-1], 1000, 1000]
    idle = instance[7][-1][0]
    assignments = [[0 for _ in range(instance[3])] for _ in range(len(instance[4]))]
    data_dic = dict(sorted(data_dic.items(), key=lambda item: item[1][1]))
    to_assign = list(data_dic.keys())
    times = {}
    for i in range(instance[4].shape[0]): # (i, j) represents (drone, slot)
        for j in range(-1, instance[3]):
            times[(i,j)] = [0, 0]
    while to_assign:
        remain = len(to_assign)
        nn = to_assign.pop(0)
        possible_list = possibles(nn, assignments, data_dic, idle, families, remain)
        drone = random.randint(0,len(possible_list)-1)
        slot = possible_list[drone]
        assignments[drone][slot] = int(nn)
        times[(drone,slot)] = [times[(drone,slot-1)][1] + instance[0][assignments[drone][slot-1]][nn-1],times[(drone,slot-1)][1] + instance[0][assignments[drone][slot-1]][nn-1] + data_dic[nn][2]]
        data_dic[nn][4:6] = times[(drone,slot)]
        # update_sc(instance, assignments, data_dic, idle, times)
    # evaluate_assignment(instance, assignments, times, data_dic)

# def update_sc(instance, assignments, data_dic, idle, times):
#     for i in range(len(assignments)):
#         for j in range(len(assignments[0])):
#             data_dic[assignments[i][j]][4] = data_dic[assignments[i][j]][2] + times[i, j - 1] + instance[0][i - 1][assignments[i][j] - 1]
#             data_dic[assignments[i, j]][4] = data_dic[assignments[i][j]][2] + times[i, j - 1] + instance[0][i - 1][assignments[i][j] - 1]

def possibles(node, assigned, data_dic, idle, families, remain):
    pos_slots = list(np.zeros(np.size(assigned, 0), dtype=int))
    if remain == idle-1:
        return pos_slots
    else:
        for i in range(ndrones):
            try:
                pos_slots[i] = assigned[i].index(0)
            except ValueError:
                pos_slots[i] = len(assigned[i])
            ## over the drones in the possibles to check the families
            ## When is not possible put IDLE and +1 the slot.
        min_slot = {}
        for i in range(len(pos_slots)):
            in_drone_families = [families[assigned[i][nd]] for nd in range(len(assigned[i]))]
            largest_appearance = max([index for index, value in enumerate(in_drone_families) if value == families[node - 1]], default=-1)
            min_slot[i] =largest_appearance
        min_slot_max = max(min_slot.values()) + 1
        pos_slots = [max(i , min_slot_max) for i in pos_slots]
            # if largest_appearance > -1:
            #     pos_slots = [max(_+1,largest_appearance+1) for _ in pos_slots]
            #     pos_slots[i] = largest_appearance + 1
        print('wait')
        # mojud = [0 for _ in range(len(assigned))]
        # for i in range(len(pos_slots)):
        #     mojud[i] = assigned[i][pos_slots[i]-1]
        # for i in mojud:
        #     if families[i-1] == families[node-1]:
        #         pos_slots = [i+1 for i in pos_slots]
        # for i in range(len(pos_slots)):
        #     if pos_slots[i] > 1 and mojud[i] != 0:
        #         if data_dic[mojud[i]][3] == data_dic[node][3]:
        #             print('these two nodes have the same family')
        #             # assigned[i][pos_slots[i]] = idle
        #             # pos_slots[i] += 1


        return pos_slots


# def evaluate_assignment(instance, assignments, times, data_dic):
#     # c_time = np.zeros(len(data_dic.keys()))
#     # s_time = np.zeros(len(data_dic.keys()))
#     # for i in range(np.shape(assignments)[0]):
#     #     for j in range(np.shape(assignments)[1]):
#     #         c_time[i,j] = data_dic[assignments[i][j]][2] + c_time[i,j-1] + instance[0][i-1][assignments[i][j]-1]
#     #         tightness[i,j] = data_dic[assignments[i][j]][0] - c_time[i,j]
#     # print('A', assignments)
#     # print('C', c_time)
#     # print('L:', tightness)
#     # l_max = -1*tightness.min()
#     print('l_max')#, l_max)


# def random_drone(instance, assignments):
#     due_date = instance[1]
#     total_slots = instance[3]*len(instance[4])
#     nodes = list(chain.from_iterable(instance[7]))
#     nodes.extend([instance[7][-1][0]] * (total_slots-len(due_date)))
#     random.shuffle(nodes)
#     assigned = np.reshape(nodes, assignments.shape)
#     return assigned

if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    random.seed(1)
    ndrones = 3
    condition = 'SB'
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