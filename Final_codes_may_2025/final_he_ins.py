import random
import numpy as np
import geopy.distance
import operator
from functools import reduce
from itertools import chain


drone_speed = 60  # kilometers per hour
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

def generate1(ndrones, city, slot, charge, hexa_data_required, seed):
    random.seed(seed)
    slots = slot
    # i_times = itimes
    locations, visit_frequency, families, len_DL = city2arc(city)
    if hexa_data_required:
        depots = ndrones*2
        locations, visit_frequency, families = hexa_modification(locations, visit_frequency, families, len_DL, depots, slots)
    distances = distance_1(locations)
    monitoring = [arc_data(arcs[i])[1] for i in locations]
    monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
    bura = len(visit_frequency) - np.max(np.nonzero(visit_frequency)) - 1
    t_matrix = np.zeros((len(monitor_times), len(monitor_times)))
    penalty = 100000
    for i in range(len(families)):
        for j in range(len(families)):
            i_mem = families[i]
            j_mem = families[j]
            for ii in i_mem:
                for jj in j_mem:
                    if i_mem == j_mem and locations[i_mem.index(ii)] == locations[i_mem.index(jj)]:
                        t_matrix[ii - 1, jj - 1] = penalty
                    else:
                        t_matrix[ii - 1, jj - 1] = round(distances[(i, j)], 2)

    day_slots = 8
    time_slots = {}
    for i in range(day_slots):
        time_slots[i] = [i*(1440/day_slots), (i+1)*(1440/day_slots)]
    i_times = []
    due_date = []  # 1440 minutes means 24 hours
    due_date2 = []
    fs_slot = []
    for i in families:
        if len(i) > 1:
            ss = []
            selected_slots = random.sample(list(range(day_slots)), len(i))
            selected_slots.sort()
            fs_slot.append(selected_slots)
            due_date.append([time_slots[j][1] for j in selected_slots])
            due_date2.append([time_slots[j][0] for j in selected_slots])
            for ii in range(len(selected_slots) - 1):
                ss.append(time_slots[selected_slots[ii + 1]][0] - time_slots[selected_slots[ii]][1])
            i_times.append(max(ss) + 30)
        else:
            due_date.append([1440])
            due_date2.append([0])
    due_date = reduce(operator.concat, due_date)
    due_date2 = reduce(operator.concat, due_date2)
    charges = np.ones(depots) * charge
    membership = []
    f = [i[:-1] for i in families[1:-bura]]
    # he_data = hexa_data(t_matrix, due_date, due_date2, monitor_times, slots, charges, ndrones, families, i_times)
    return t_matrix, due_date, monitor_times, slots, charges, i_times, membership, families, f, due_date2, len_DL, fs_slot #, he_data

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


def distance_1(locations):
    dist_mat = np.zeros((len(locations), len(locations)))
    for i in range(len(locations)):
        coord1_1 = (arcs[locations[i]][0], arcs[locations[i]][1])
        coord1_2 = (arcs[locations[i]][2], arcs[locations[i]][3])
        for j in range(len(locations)):
            coord2_1 = (arcs[locations[j]][0], arcs[locations[j]][1])
            coord2_2 = (arcs[locations[j]][2], arcs[locations[j]][3])
            dist_mat[i, j] = geopy.distance.geodesic(coord1_2, coord2_1).kilometers
            dist_mat[j, i] = geopy.distance.geodesic(coord2_2, coord1_1).kilometers
    return dist_mat

def centerz(lis):
    length = 2*len(lis)
    lis = [arcs[i][0:4] for i in lis]
    lis = list(chain.from_iterable(lis))
    sum_x = np.sum(lis[0::2])
    sum_y = np.sum(lis[1::2])
    return sum_x/length, sum_y/length, sum_x/length, sum_y/length, 0

def hexa_modification(locations, visit_frequency, families, len_DL, depots, slots):
    total_real = sum(visit_frequency)
    total_slots = depots*slots
    total_depots = depots
    remaining_for_idle = total_slots - total_real - total_depots
    total_idle = []
    if remaining_for_idle % len_DL == 0:
        for i in range(len_DL):
            total_idle.append(remaining_for_idle/len_DL)
    else:
        for i in range(len_DL):
            total_idle.append(remaining_for_idle//len_DL)
        total_idle[-1] += remaining_for_idle % len_DL
    locations_depots = [locations[0]]* total_depots
    locations_real = locations[1:-len_DL]
    locations_idles = [[locations[-s]]* int(total_idle[-s]) for s in range(len_DL, 0, -1)]
    locations_idles = [item for sublist in locations_idles for item in sublist]
    locations = locations_depots + locations_real + locations_idles
    visit_frequency = [visit_frequency[0]]*total_depots + visit_frequency[1:-len_DL] + [visit_frequency[-1]]*int(sum(total_idle))
    families_depots = [[i] for i in range(1, total_depots + 1)]
    families_real = [list(range(i[0] + (total_depots - 1), i[-1] + (total_depots - 1) + 1)) for i in families[1:-len_DL]]
    families_idle = [[i] for i in range(families_real[-1][-1]+1, total_slots+1)]
    families = families_depots + families_real + families_idle
    return locations, visit_frequency, families
