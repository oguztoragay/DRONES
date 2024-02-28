from pyomo.environ import value
import numpy as np
from itertools import combinations, chain, repeat
import random


def generate(m, n, condition):
    print('-------------------------------------------------------------------------------------------------')
    if condition == 'random':
        f = [[2, 3], [5, 6], [8]]
        families = []
        for fam in f:
            families.append(fam + [fam[-1]+1])
        list1 = [[1]]
        for i in families:
            list1.append(i)
        families = list1
        pairs = list(combinations(range(len(f)+1), 2))
        distances = {key: random.randint(1, 5) for key in pairs}
        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(i+1, len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        t_matrix[ii-1, jj-1] = distances[(i, j)]
        matrix_u = np.triu(t_matrix)
        matrix_l = matrix_u.T
        t_matrix = matrix_u + matrix_l
        np.fill_diagonal(t_matrix, 0)
        t_matrix = t_matrix.reshape((m, m))

        slots = np.random.randint(6, 8)
        charges = np.random.randint(13, 15, size=n)
        i_times = 2
        due = random.sample(range(4, 9), len(families))
        due[0] = 0
        due_date = []
        membership = []
        for i in range(len(families)):
            for j in range(len(families[i])):
                due_date.append(due[i]*(j+1))
                membership.append(i)
        due = due_date
        monitor_times = np.array(np.random.randint(1, 3, size=(1, m)).ravel())

    if condition == 'fixed':
        m = 9
        n = 2
        f = [[2, 3], [5, 6], [8]]
        families = [[1], [2, 3, 4], [5, 6, 7], [8, 9], [10]]
        monitor_times = np.array([3, 2, 2, 2, 1, 1, 1, 1, 1, 0.01])  # Pj [2, 1, 2, 3, 4, 5, 5]
        t_matrix = np.array([[0, 3, 3, 3, 4, 4, 4, 1, 1, 0, 0.1],
                             [3, 0, 0, 0, 1, 1, 1, 2, 2, 3, 0.1],
                             [3, 0, 0, 0, 1, 1, 1, 2, 2, 3, 0.1],
                             [3, 0, 0, 0, 1, 1, 1, 2, 2, 3, 0.1],
                             [4, 1, 1, 1, 0, 0, 0, 3, 3, 3, 0.1],
                             [4, 1, 1, 1, 0, 0, 0, 3, 3, 3, 0.1],
                             [4, 1, 1, 1, 0, 0, 0, 3, 3, 3, 0.1],
                             [1, 2, 2, 2, 3, 3, 3, 0, 0, 3, 0.1],
                             [1, 2, 2, 2, 3, 3, 3, 0, 0, 3, 0.1],
                             [0, 3, 3, 3, 3, 3, 3, 3, 3, 0, 0.1],
                             [0.1,0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0]])  # Sjk

        due = np.array([0, 10, 20, 24, 8, 16, 24, 5, 10, 24])  # dj
        # due = np.array([0, 24, 24, 24, 24, 24, 24, 24, 24, 24])
        charges = np.ones(n)*7
        i_times = 3  #max intervisit time
        slots = 8
        membership = [0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 5]

    if condition == 'SB':
        f = [[2], [4, 5], [7], [9, 10]]
        monitoring = [0.04, 0.03, 0.01, 0.02, 0.5, 0.002]
        families = []
        for fam in f:
            families.append(fam + [fam[-1] + 1])
        list1 = [[1]]
        for i in families:
            list1.append(i)
        families = list1
        families.append([12])
        # print([len(i) + 1 for i in families])
        monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
        m = len(monitor_times)
        distances = np.array([[0, 0.0632, 0.0117, 0.0225, 0.1443, 0.08],
                              [0.0352, 0, 0.0376, 0.0392, 0.1304, 0.08],
                              [0.0212, 0.0673, 0, 0.0437, 0.1268, 0.08],
                              [0.0458, 0.0713, 0.0574, 0, 0.1818, 0.08],
                              [0.13, 0.1343, 0.1186, 0.1517, 0, 0.08],
                              [0.08, 0.08, 0.08, 0.08, 0.08, 0]]) # Sjk

        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        t_matrix[ii - 1, jj - 1] = distances[(i, j)]
        # due = np.array([0, 0.05, 0.10, 0.05, 0.10, 0.15, 0.06, 0.12, 0.10, 0.14, 0.23, 0.24])
        due = np.array([0, 0.1, 0.23, 0.03, 0.8, 0.3, 0.16, 1.8, 0.7, 1.5, 1.7, 0])
        due_date = []
        membership = []
        for i in range(len(families)):
            for j in range(len(families[i])):
                due_date.append(due[i] * (j + 1))
                membership.append(i)

        # due = np.array([0, 0.05, 0.10, 0.05, 0.10, 0.15, 0.06, 0.12, 0.10, 0.14, 0.23, 0.24])
        # due = np.array([0, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24])# dj
        charges = np.ones(n) * 0.4
        i_times = 1  # max intervisit time
        slots = 8

    if condition == 'SB_RS':
        f = [[2], [4, 5], [7], [9, 10], [12, 13], [15], [17], [19]]
        monitoring = [0.04, 0.03, 0.01, 0.02, 0.5, 0.04, 0.03, 0.02, 0.02, 0.002]
        families = []
        for fam in f:
            families.append(fam + [fam[-1] + 1])
        list1 = [[1]]
        for i in families:
            list1.append(i)
        families = list1
        families.append([21])
        # print([len(i) + 1 for i in families])
        monitor_times = list(np.repeat(monitoring, [len(i) for i in families]))
        m = len(monitor_times)
        distances = np.array([[0, 0.0632, 0.0117, 0.0225, 0.1443, 0.4696, 0.1890, 0.3619, 0.3260, 0.08],
                              [0.0352, 0, 0.0376, 0.0392, 0.1304, 0.5006, 0.2196, 0.3508, 0.3373, 0.08],
                              [0.0212, 0.0673, 0, 0.0437, 0.1268, 0.4798, 0.2004, 0.3820, 0.3464, 0.08],
                              [0.0458, 0.0713, 0.0574, 0, 0.1818, 0.4567, 0.1772, 0.3170, 0.2856, 0.08],
                              [0.13, 0.1343, 0.1186, 0.1517, 0, 0.5609, 0.2922, 0.4783, 0.4557, 0.08],
                              [0.5067, 0.5638, 0.5117, 0.4995, 0.6329, 0, 0.3187, 0.5893, 0.3716, 0.08],
                              [0.2026, 0.2591, 0.2084, 0.1948, 0.3359, 0.2670, 0, 0.3838, 0.2279, 0.08],
                              [0.3734, 0.3526, 0.3844, 0.3518, 0.4731, 0.5726, 0.3947, 0, 0.2415, 0.08],
                              [0.3265, 0.3456, 0.3378, 0.3053, 0.4661, 0.3521, 0.2375, 0.1992, 0, 0.08],
                              [0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0]])  # Sjk

        t_matrix = np.zeros((m, m))
        for i in range(len(families)):
            for j in range(len(families)):
                i_mem = families[i]
                j_mem = families[j]
                for ii in i_mem:
                    for jj in j_mem:
                        t_matrix[ii - 1, jj - 1] = distances[(i, j)]
        # due = np.array([0, 0.05, 0.10, 0.05, 0.10, 0.15, 0.06, 0.12, 0.10, 0.14, 0.23, 0.24])
        # due = np.array([0, 0.1, 0.23, 0.03, 0.8, 0.3, 0.16, 1.8, 0.7, 1.5, 1.7, 2.0, 2.2, 2.5, 2.7, 0])
        due = np.array([0, 0.3, 0.69, 0.09, 2.4, 0.9, 0.48, 5.4, 2.1, 4.5, 5.1, 6.0, 6.6, 7.5, 8.1, 0])
        due_date = []
        membership = []
        for i in range(len(families)):
            for j in range(len(families[i])):
                due_date.append(due[i] * (j + 1))
                membership.append(i)

        # due = np.array([0, 0.05, 0.10, 0.05, 0.10, 0.15, 0.06, 0.12, 0.10, 0.14, 0.23, 0.24])
        # due = np.array([0, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24])# dj
        charges = np.ones(n) * 0.8
        i_times = 3  # max intervisit time
        slots = 15

    return t_matrix, due_date, monitor_times, slots, charges, i_times, membership, families, f


def mprint(m, solution, datam):
    print('\n\n\n------------------------------------------------------')
    print('---------------- MODEL DATA & RESULTS ----------------')
    print('------------------------------------------------------\n')
    print('The travel matrix is:\n', datam[0])
    print('Due dates are:\n', datam[1])
    print('Monitoring times are:\n', datam[2])
    print('\n***** Solver Message *****')
    print(solution['Solver'].message)
    print('Current objective value is:', value(m.obj_func))
    print('\nThe visiting assignments are as follow:')
    assign_list = np.zeros((len(datam[4]), datam[3]))
    assign_families = np.zeros((len(datam[4]), datam[3]))
    for ind in m.x.index_set():
        if value(m.x[ind]) == 1:
            assign_list[ind[2]-1, ind[1]-1] = ind[0]
            assign_families[ind[2]-1, ind[1]-1] = datam[6][ind[0]-1] + 1
    print(assign_list)
    print("\nOr in terms of families:")
    print(assign_families)
    list_c = sorted(list(m.c.index_set()), key=lambda x: x[1])
    c_values = []
    s_values = []
    for ind in list_c:
        c_values.append(value(m.c[ind]))
        # s_values.append(value(m.c[ind])-datam[2][ind[0]])
        s_values.append(value(m.s[ind]))
        # print('c', ind, '=', value(m.c[ind]))
    c_values = np.reshape(c_values, (len(datam[4]), datam[3]))
    s_values = np.reshape(s_values, (len(datam[4]), datam[3]))
    print("\nThe c values are as follow:")
    print(c_values)
    print("\nThe s values are as follow:")
    print(s_values)
    for ind in m.w.index_set():
        if value(m.w[ind]):
            print('w', ind, '=', value(m.w[ind]))





































