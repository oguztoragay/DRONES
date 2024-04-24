import numpy as np
import random
from random_instance import generate

#### Start and conclusion times defined for each visit node which are saved in data_dic
#### Times defined the start time and conclusion time of the visit assigned to (i,j) --> (drone, slot) position of the drone.
def greedy_drone(ndrones, instance):
    print('----------------------- Greedy Drone -----------------------')
    fam_mat = {}
    families = instance[6]
    for i in range(1,len(instance[7])-1):
        fam_mat[i] = instance[7][i]
    node_keys = list(range(2,instance[7][-1][0]))
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
        possible_list = possibles(nn, assignments, data_dic, idle, families, remain, ndrones)
        drone = random.randint(0,len(possible_list)-1)
        slot = possible_list[drone]
        assignments[drone][slot] = int(nn)
        times[(drone,slot)] = [times[(drone,slot-1)][1] + instance[0][assignments[drone][slot-1]][nn-1],times[(drone,slot-1)][1] + instance[0][assignments[drone][slot-1]][nn-1] + data_dic[nn][2]]
        data_dic[nn][4:6] = times[(drone,slot)]
    return assignments, times, data_dic

def possibles(node, assigned, data_dic, idle, families, remain, ndrones):
    pos_slots = list(np.zeros(np.size(assigned, 0), dtype=int))
    if remain == idle-1:
        return pos_slots
    else:
        for i in range(ndrones):
            try:
                pos_slots[i] = assigned[i].index(0)
            except ValueError:
                pos_slots[i] = len(assigned[i])
        min_slot = {}
        for i in range(len(pos_slots)):
            in_drone_families = [families[assigned[i][nd]] for nd in range(len(assigned[i]))]
            largest_appearance = max([index for index, value in enumerate(in_drone_families) if value == families[node - 1]], default=-1)
            min_slot[i] =largest_appearance
        min_slot_max = max(min_slot.values()) + 1
        pos_slots = [max(i , min_slot_max) for i in pos_slots]
        return pos_slots


def greedy_sol(s, ins):
    np.set_printoptions(suppress=True)
    ndrones = s
    inst = ins
    tries = 1
    while True:
        try:
            generated = greedy_drone(ndrones, instance=inst)
            break
        except:
            tries += 1
            continue
    return generated, tries