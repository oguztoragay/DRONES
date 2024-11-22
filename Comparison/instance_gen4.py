## 11/15/2024 change in the code to implement slotting of the 24 hours and randomly assign slots for each visit
## to facilitate the due times, early times, and i_times assignments.

import sys
import itertools
import random
import numpy as np
import geopy.distance
import operator
from functools import reduce
from itertools import chain
random.seed(3377)

drone_speed = 100  # kilometers per hour
arcs = {'DP': [34.02889809043227, -117.83417609430023, 34.02889809043227, -117.83417609430023, 0],
        'SB1': [34.06748049349774, -117.58627467784373, 34.06733829280365, -117.56805711511826, 2],
        'SB2': [34.073664262003916, -117.54478073959717, 34.07880072708419, -117.54469490891294, 3],
        'SB3': [34.055221667702725, -117.54707552414482, 34.042942672497496, -117.55046155255096, 2],
        'SB4': [34.13509556151658, -117.5918373464612, 34.13665850403775, -117.55952209385272, 3],
        'LA1': [34.068996100878536, -118.23639721457458, 34.08111761574252, -118.22502464857632, 4],
        'LA2': [34.00204584929778, -118.28106671082655, 34.02099821349527, -118.27820873785127, 3],
        'LA3': [34.1820571382235, -118.47097754846425, 34.238471386274036, -118.47316244104785, 4],
        'LA4': [34.01006543492014, -118.4164022951023, 34.034039070699464, -118.43717332130281, 3],
        'LA5': [33.970081316208265, -118.37465654195792, 33.98578244932628, -118.39797964305346, 4],
        'LA6': [34.06144667253944, -118.2153262230426, 34.0759258090762, -118.21921047652454, 3],
        'LA7': [33.99983225156185, -118.14864437645414, 33.978268943166086, -118.1272725354462, 3],
        'LA8': [33.96772442600475, -118.08341546454861, 33.93582806531739, -118.09937997228951, 3],
        'RS1': [33.94245747674634, -117.27950645054175, 33.94101794657498, -117.2530191660786, 3],
        'RS2': [34.01310741213049, -117.44103478257155, 34.01129317274416, -117.43197964538643, 2],
        'RS3': [33.88454308524973, -117.62931466068288, 33.88289535845872, -117.644110477192, 2],
        'RS4': [33.897340150833756, -117.48842212774255, 33.894628991533885, -117.50064506537053, 2],
        'RS_iDL': [33.93341044928401, -117.45850404694579, 33.93341044928401, -117.45850404694579, 0],
        'SB_iDL': [34.082150272643005, -117.56158799481022, 34.082150272643005, -117.56158799481022, 0],
        'LA_iDL': [34.03829254642072, -118.28026863444143, 34.03829254642072, -118.28026863444143, 0]}

def generate(ndrones, city, slot, charge):
    # seed1 = random.randrange(sys.maxsize)
    # random.seed(seed1)
    seed1 = 3377
    slots = slot
    # i_times = itimes
    locations, visit_frequency, families, len_DL = city2arc(city)
    distances = arc2distance(locations)
    monitoring = [arc_data(arcs[i])[1] for i in locations]
    monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
    t_matrix = np.zeros((len(monitor_times), len(monitor_times)))
    penalty = 10000
    for i in range(len(families)):
        for j in range(len(families)):
            i_mem = families[i]
            j_mem = families[j]
            for ii in i_mem:
                for jj in j_mem:
                    if i_mem == j_mem and i != len(families)-1:
                        t_matrix[ii - 1, jj - 1] = penalty
                    elif j_mem == i_mem and j == len(families)-1:
                        t_matrix[ii - 1, jj - 1] = round(distances[(i, j)], 2)
                    else:
                        t_matrix[ii - 1, jj - 1] = round(distances[(i, j)], 2)
    bura = len(visit_frequency) - np.max(np.nonzero(visit_frequency)) - 1
    for i in range(families[-bura][0]-1, families[-1][0]-1):
        t_matrix[i][i] = 0
    num_slots = 8
    time_slots = {}
    for i in range(num_slots):
        time_slots[i] = [i*(1440/num_slots), (i+1)*(1440/num_slots)]
    i_times = []
    due_date = [[1440]]  # 1440 minutes means 24 hours
    due_date2 = [[0]]
    fs_slot = []
    for i in families:
        if len(i) > 1:
            selected_slots = random.sample(list(range(num_slots)), len(i))
            selected_slots.sort()
            fs_slot.append(selected_slots)
            due_date.append([time_slots[j][1] for j in selected_slots])
            due_date2.append([time_slots[j][0] for j in selected_slots])
            # i_times.append(time_slots[max(selected_slots)][1]-time_slots[min(selected_slots)][0] + 60)
            i_times.append(time_slots[selected_slots[-1]][1]-time_slots[selected_slots[0]][0] + 10)

    for i in range(bura):
        due_date.append([1440])
        due_date2.append([0])

    due_date = reduce(operator.concat, due_date)
    due_date2 = reduce(operator.concat, due_date2)
    charges = np.ones(ndrones) * charge
    membership = []
    f = [i[:-1] for i in families[1:-bura]]
    return t_matrix, due_date, monitor_times, slots, charges, i_times, membership, families, f, due_date2, len_DL, fs_slot, seed1

def arc_data(arc):
    coord1 = (arc[0], arc[1])
    coord2 = (arc[2], arc[3])
    arc_length = geopy.distance.geodesic(coord1, coord2).kilometers  # distance in kilometers
    monitoring = round((arc_length/(drone_speed*0.5))*60, 2)  # visit times in minutes, drone speed become half during the scan or visit
    if arc == arcs['DP']:
        monitoring = 60
    return arc_length, monitoring

def city2arc(city):
    SB = ['SB1', 'SB2', 'SB3', 'SB4']
    LA = ['LA1', 'LA2', 'LA3', 'LA4', 'LA5', 'LA6', 'LA7', 'LA8']
    RS = ['RS1', 'RS2', 'RS3', 'RS4']
    visit_frequency = []
    cities = city.split('_')
    DL = []
    for i in cities:
        DL.append(str(i)+'_iDL')
    locations = ['DP']
    for i in cities:
        locations.extend([i for i in eval(i)])
    locations += DL
    # locations = loc_up(locations)
    for i in locations:
        visit_frequency.append(arcs[i][4])
    fam = [[1]]
    s = 2
    for i in visit_frequency[1:-len(DL)]:
        f = list(range(s, s+i))
        fam.append(f)
        s = f[-1]+1
    for i in range(len(DL)):
        fam.append([s])
        s += 1
    return locations, visit_frequency, fam, len(DL)

def arc2distance(locations):
    dist_mat = np.zeros((len(locations), len(locations)))
    for i, j in itertools.combinations(locations, 2):
        travel_1_2 = [arcs[i][2], arcs[i][3], arcs[j][0], arcs[j][1]]
        ij = arc_data(travel_1_2)[1]
        travel_2_1 = [arcs[j][2], arcs[j][3], arcs[i][0], arcs[i][1]]
        ji = arc_data(travel_2_1)[1]
        dist_mat[locations.index(i), locations.index(j)] = ij
        dist_mat[locations.index(j), locations.index(i)] = ji
    return dist_mat

def centerz(lis):
    length = 2*len(lis)
    lis = [arcs[i][0:4] for i in lis]
    lis = list(chain.from_iterable(lis))
    sum_x = np.sum(lis[0::2])
    sum_y = np.sum(lis[1::2])
    return sum_x/length, sum_y/length, sum_x/length, sum_y/length, 0

# def loc_up(locations):
#     latitudes = np.mean([(arcs[coord][0] + arcs[coord][2])/2 for coord in locations[1:]])
#     longitudes = np.mean([(arcs[coord][1] + arcs[coord][3])/2 for coord in locations[1:]])
#     arcs['DP'] = [latitudes, longitudes, latitudes, longitudes, 0]
#     return locations
