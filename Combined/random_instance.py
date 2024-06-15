from pyomo.environ import value
import numpy as np


def generate(ndrones, condition, slot, charge, itimes):
    t_matrix = []
    due_date = []
    monitor_times = []
    slots = slot
    charge_ = charge
    i_times = itimes
    membership = []
    families = []
    f =[]
    print('-------------------------------------------------------------------------------------------------')
    if condition == 'mini_fixed':
        f = [[2], [4, 5]]
        families = [[1], [2, 3], [4, 5, 6], [7]]
        monitor_times = np.array([3, 2, 2, 1, 1, 1, 0.01])
        t_matrix = np.array([[0, 3, 3, 4, 4, 4,  0.1],
                             [3, 0, 0, 1, 1, 1,  0.1],
                             [3, 0, 0, 1, 1, 1,  0.1],
                             [4, 1, 1, 0, 0, 0,  0.1],
                             [4, 1, 1, 0, 0, 0,  0.1],
                             [4, 1, 1, 0, 0, 0,  0.1],
                             [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0]])  # Sjk

        due_date = np.array([0, 10, 20, 8, 16, 24, 30])  # dj
        charges = np.ones(ndrones)*5
        i_times = 3 #max intervisit time
        slots = 4
        membership = [1, 2, 2, 3, 3, 3, 4]
    if condition == 'fixed':
        f = [[2, 3], [5, 6], [8]]
        families = [[1], [2, 3, 4], [5, 6, 7], [8, 9], [10]]
        monitor_times = np.array([5, 2, 2, 2, 1, 1, 1, 1, 1, 0.01])  # Pj [2, 1, 2, 3, 4, 5, 5]
        distances = np.array([[0, 3, 4, 1, 0.1],
                              [3, 0, 1, 2, 0.1],
                              [4, 1, 0, 3, 0.1],
                              [1, 2, 3, 0, 0.1],
                              [0.1, 0.1, 0.1, 0.1, 0]]) # Sjk
        m = len(monitor_times)
        penalty = 1000
        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        if i_mem == j_mem and i != len(families) - 1:
                            t_matrix[ii - 1, jj - 1] = penalty
                        elif j_mem == i_mem and j == len(families) - 1:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
                        else:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]

        due_date = np.array([50, 10, 20, 24, 8, 16, 24, 5, 10, 50])  # dj
        # due_date = np.array([i * 0.2 for i in due_date])
        charges = np.ones(ndrones)*charge_
        membership = [1, 2, 2, 2, 3, 3, 3, 4, 4, 5]
    if condition == 'SB':
        f = [[2], [4, 5], [7], [9, 10]]
        monitoring = [0.09, 0.03, 0.01, 0.02, 0.05, 0.0002]
        families = []
        last_ = 0
        for fam in f:
            families.append(fam + [fam[-1] + 1])
            last_ = fam[-1] + 2
        list1 = [[1]]
        for i in families:
            list1.append(i)
        families = list1
        families.append([last_])
        monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
        m = len(monitor_times)
        non_symmetric_dist = np.array([[0.0000, 0.0632, 0.0117, 0.0225, 0.1443, 0.0080],
                              [0.0352, 0.0000, 0.0376, 0.0392, 0.1304, 0.0080],
                              [0.0212, 0.0673, 0.0000, 0.0437, 0.1268, 0.0080],
                              [0.0458, 0.0713, 0.0574, 0.0000, 0.1818, 0.0080],
                              [0.1300, 0.1343, 0.1186, 0.1517, 0.0000, 0.0080],
                              [0.0080, 0.0080, 0.0080, 0.0080, 0.0080, 0.0000]]) # Sjk

        symmetric_dist = (non_symmetric_dist + non_symmetric_dist.T) / 2
        distances = non_symmetric_dist
        penalty = 1000
        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        if i_mem == j_mem and i != len(families)-1:
                            t_matrix[ii - 1, jj - 1] = penalty
                        elif j_mem == i_mem and j == len(families)-1:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
                        else:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
        due = np.array([50, 0.1, 0.23, 0.03, 0.15, 0.3, 0.46, 0.8, 0.7, 0.9, 1.1, 50])
        # due = np.array([i*0.2 for i in due])
        membership = []
        for i in range(len(families)):
            for j in range(len(families[i])):
                due_date.append(due[i])
                membership.append(i + 1)
        due_date = due
        charges = np.ones(ndrones) * charge_
        # i_times = 4  # max intervisit time
        # slots = 7
    if condition == 'SB_RS':
        f = [[2], [4, 5], [7], [9, 10], [12, 13], [15], [17], [19]]
        monitoring = [0.04, 0.03, 0.01, 0.02, 0.05, 0.04, 0.03, 0.02, 0.02, 0.002]
        families = []
        for fam in f:
            families.append(fam + [fam[-1] + 1])
        list1 = [[1]]
        for i in families:
            list1.append(i)
        families = list1
        families.append([21])
        monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
        m = len(monitor_times)
        non_symmetric_dist = np.array([[0, 0.0632, 0.0117, 0.0225, 0.1443, 0.4696, 0.1890, 0.3619, 0.3260, 0.01],
                              [0.0352, 0, 0.0376, 0.0392, 0.1304, 0.5006, 0.2196, 0.3508, 0.3373, 0.01],
                              [0.0212, 0.0673, 0, 0.0437, 0.1268, 0.4798, 0.2004, 0.3820, 0.3464, 0.01],
                              [0.0458, 0.0713, 0.0574, 0, 0.1818, 0.4567, 0.1772, 0.3170, 0.2856, 0.01],
                              [0.13, 0.1343, 0.1186, 0.1517, 0, 0.5609, 0.2922, 0.4783, 0.4557, 0.01],
                              [0.5067, 0.5638, 0.5117, 0.4995, 0.6329, 0, 0.3187, 0.5893, 0.3716, 0.01],
                              [0.2026, 0.2591, 0.2084, 0.1948, 0.3359, 0.2670, 0, 0.3838, 0.2279, 0.01],
                              [0.3734, 0.3526, 0.3844, 0.3518, 0.4731, 0.5726, 0.3947, 0, 0.2415, 0.01],
                              [0.3265, 0.3456, 0.3378, 0.3053, 0.4661, 0.3521, 0.2375, 0.1992, 0, 0.01],
                              [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0]])
        symmetric_dist = (non_symmetric_dist + non_symmetric_dist.T) / 2
        distances = symmetric_dist
        penalty = 1000
        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        if i_mem == j_mem and i != len(families)-1:
                            t_matrix[ii - 1, jj - 1] = penalty
                        elif j_mem == i_mem and j == len(families)-1:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
                        else:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
        due = np.array([30, 0.3, 0.69, 0.29, 2.15, 0.9, 0.48, 1.48, 2.1, 30])
        due = np.array([i * 0.2 for i in due])
        due_date = []
        membership = []
        for i in range(len(families)):
            for j in range(len(families[i])):
                due_date.append(due[i] * (j + 1))
                membership.append(i+1)

        charges = np.ones(ndrones) * charge_
        # i_times = 3 # max intervisit time
        # slots = 8
        max_need_charge = max([distances[j][i] for i in range(len(distances[j])) for j in range(len(distances[1]))]) * 2
    if condition == 'SB_RS_LA':
        f = [[2], [4, 5], [7], [9, 10], [12, 13], [15], [17], [19], [21, 22, 23, 24],[26, 27, 28], [30, 31, 32, 33],[35, 36, 37], [39, 40, 41, 42], [44, 45, 46], [48, 49, 50], [52, 53, 54]]
        monitoring = [1, 0.03, 0.01, 0.02, 0.05, 0.04, 0.03, 0.02, 0.02, 0.03, 0.03, 0.10, 0.05, 0.05, 0.03, 0.05, 0.06, 0.001]
        families = []
        for fam in f:
            families.append(fam + [fam[-1] + 1])
        list1 = [[1]]
        for i in families:
            list1.append(i)
        families = list1
        families.append([56])
        # print([len(i) + 1 for i in families])
        monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
        m = len(monitor_times)
        distances = np.array([[0.0000, 0.0632, 0.0117, 0.0225, 0.1443, 0.4696, 0.1890, 0.3619, 0.3260, 1.0635, 1.1391, 1.4392, 1.3451, 1.3214, 1.0312, 0.9372, 0.8488, 0.0800],
                              [0.0352, 0.0000, 0.0376, 0.0392, 0.1304, 0.5006, 0.2196, 0.3508, 0.3373, 1.0283, 1.1040, 1.4043, 1.3100, 1.2864, 0.9959, 0.9023, 0.8144, 0.0800],
                              [0.0212, 0.0673, 0.0000, 0.0437, 0.1268, 0.4798, 0.2004, 0.3820, 0.3464, 1.0643, 1.1421, 1.4368, 1.3475, 1.3246, 1.0323, 0.9409, 0.8543, 0.0800],
                              [0.0458, 0.0713, 0.0574, 0.0000, 0.1818, 0.4567, 0.1772, 0.3170, 0.2856, 1.0566, 1.1272, 1.4387, 1.3343, 1.3090, 1.0237, 0.9243, 0.8322, 0.0800],
                              [0.1300, 0.1343, 0.1186, 0.1517, 0.0000, 0.5609, 0.2922, 0.4783, 0.4557, 1.0485, 1.1376, 1.4033, 1.3389, 1.3200, 1.0181, 0.9410, 0.8646, 0.0800],
                              [0.5067, 0.5638, 0.5117, 0.4995, 0.6329, 0.0000, 0.3187, 0.5893, 0.3716, 1.5324, 1.5875, 1.9262, 1.7964, 1.7656, 1.4983, 1.3838, 1.2802, 0.0800],
                              [0.2026, 0.2591, 0.2084, 0.1948, 0.3359, 0.2670, 0.0000, 0.3838, 0.2279, 1.2426, 1.3074, 1.6289, 1.5156, 1.4881, 1.2092, 1.1036, 1.0064, 0.0800],
                              [0.3734, 0.3526, 0.3844, 0.3518, 0.4731, 0.5726, 0.3947, 0.0000, 0.2415, 0.9749, 1.0058, 1.3877, 1.2129, 1.1771, 0.9396, 0.8069, 0.6949, 0.0800],
                              [0.3265, 0.3456, 0.3378, 0.3053, 0.4661, 0.3521, 0.2375, 0.1992, 0.0000, 1.1781, 1.2187, 1.5851, 1.4269, 1.3929, 1.1431, 1.0171, 0.9082, 0.0800],
                              [1.0462, 0.9829, 1.0465, 1.0441, 0.9788, 1.4780, 1.2130, 0.9868, 1.1841, 0.0000, 0.1697, 0.4217, 0.3225, 0.3193, 0.0393, 0.1908, 0.3024, 0.0800],
                              [1.1314, 1.0683, 1.1329, 1.1270, 1.0767, 1.5449, 1.2888, 1.0310, 1.2379, 0.1096, 0.0000, 0.4202, 0.2137, 0.1956, 0.1223, 0.2033, 0.3157, 0.0800],
                              [1.4609, 1.3992, 1.4590, 1.4632, 1.3675, 1.9158, 1.6406, 1.4539, 1.6411, 0.4802, 0.5275, 0.0000, 0.4311, 0.4812, 0.5139, 0.6660, 0.7809, 0.0800],
                              [1.3740, 1.3108, 1.3751, 1.3704, 1.3137, 1.7908, 1.5338, 1.2748, 1.4831, 0.3156, 0.2474, 0.2785, 0.0000, 0.1077, 0.3451, 0.4487, 0.5583, 0.0800],
                              [1.2895, 1.2269, 1.2917, 1.2837, 1.2426, 1.6878, 1.4399, 1.1594, 1.3722, 0.2806, 0.1557, 0.4189, 0.0979, 0.0000, 0.2978, 0.3523, 0.4486, 0.0800],
                              [1.0371, 0.9738, 1.0375, 1.0348, 0.9709, 1.4676, 1.2031, 0.9749, 1.1728, 0.0293, 0.1664, 0.4339, 0.3269, 0.3216, 0.0000, 0.1777, 0.2893, 0.0800],
                              [0.9110, 0.8489, 0.9137, 0.9044, 0.8734, 1.3076, 1.0586, 0.7866, 0.9956, 0.2373, 0.2408, 0.6492, 0.4490, 0.4171, 0.2049, 0.0000, 0.0703, 0.0800],
                              [0.8873, 0.8266, 0.8911, 0.8786, 0.8636, 1.2633, 1.0240, 0.7307, 0.9443, 0.3242, 0.3054, 0.7308, 0.5072, 0.4691, 0.2929, 0.1405, 0.0000, 0.0800],
                              [0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0800, 0.0000]])
        penalty = 1000
        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        if i_mem == j_mem and i != len(families) - 1:
                            t_matrix[ii - 1, jj - 1] = penalty
                        elif j_mem == i_mem and j == len(families) - 1:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
                        else:
                            t_matrix[ii - 1, jj - 1] = distances[(i, j)]
        due = np.array([10, 0.3, 0.69, 0.29, 2.4, 0.9, 0.48, 5.4, 2.1, 0.3, 0.7, 0.9, 1.2, 1.9, 2.4, 2.8, 3.1, 10])
        due_date = []
        membership = []
        for i in range(len(families)):
            for j in range(len(families[i])):
                due_date.append(due[i] * (j + 1))
                membership.append(i + 1)

        charges = np.ones(ndrones) * charge_
        # i_times = 5  # max intervisit time
        # slots = 20
        max_need_charge = max([distances[j][i] for i in range(len(distances[j])) for j in range(len(distances[1]))]) * 2
    return t_matrix, due_date, monitor_times, slots, charges, i_times, membership, families, f

def mprint(m, solution, datam):
    print('***** Solver Message *****')
    print(solution['Solver'].message)
    print('Current objective value is:', value(m.obj_func))
    assign_list = np.zeros((len(datam[4]), datam[3]), dtype=int)
    assign_families = np.zeros((len(datam[4]), datam[3]), dtype=int)
    assign_dues = np.zeros((len(datam[4]), datam[3]), dtype=int)
    for ind in m.x.index_set():
        if value(m.x[ind]) == 1:
            assign_list[ind[2]-1, ind[1]-1] = ind[0]
            assign_dues[ind[2]-1, ind[1]-1] = datam[1][ind[0]-1]
            assign_families[ind[2]-1, ind[1]-1] = datam[6][ind[0]-1] + 1
    for i in range(0, assign_list.shape[0]):
        print(*assign_list[i], sep=' --> ')
    # print("\nOr in terms of due_dates:")
    # print(assign_dues)
    # print("\nOr in terms of families:")
    # print(assign_families)
    list_c = sorted(list(m.c.index_set()), key=lambda x: x[1])
    c_values = []
    s_values = []
    for ind in list_c:
        c_values.append(value(m.c[ind]))
        s_values.append(value(m.s[ind]))
    c_values = np.reshape(c_values, (len(datam[4]), datam[3]))
    s_values = np.reshape(s_values, (len(datam[4]), datam[3]))
    print("The s times:")
    print(s_values)
    print("The c times:")
    print(c_values,'\n')
    # for ind in m.w.index_set():
    #     if value(m.w[ind]):
    #         print('w', ind, '=', value(m.w[ind]))

def route_battery(m, solution, datam):
    assign_list = np.zeros((len(datam[4]), datam[3]), dtype=int)
    assign_families = np.zeros((len(datam[4]), datam[3]), dtype=int)
    assign_dues = np.zeros((len(datam[4]), datam[3]), dtype=int)
    for ind in m.x.index_set():
        if value(m.x[ind]) == 1:
            assign_list[ind[2] - 1, ind[1] - 1] = ind[0]
            assign_dues[ind[2] - 1, ind[1] - 1] = datam[1][ind[0] - 1]
            assign_families[ind[2] - 1, ind[1] - 1] = datam[6][ind[0] - 1] + 1
    battery_usage = {}
    for i in range(0, assign_list.shape[0]):
        clist = assign_list[i]
        battery_usage[i] = [datam[4][i] - datam[0][0][clist[0]-1]+datam[2][clist[0]-1]]
        for j in range(1, len(clist)):
            battery_usage[i].append(battery_usage[i][-1] - datam[0][clist[j-1]-1][clist[j]-1] - datam[2][clist[j]-1] + datam[4][i]*int(clist[j]==1))
    print("Battery Usage:")
    # for i in range(0, len(battery_usage)):
    #     print(battery_usage[i])
    for i in m.t_index_1.data():
        print([value(m.t[j,i]) for j in m.t_index_0.data()])

