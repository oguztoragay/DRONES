from pyomo.environ import value
import numpy as np
from itertools import combinations, product
import random


def generate(m, n, f, condition):
    print('-------------------------------------------------------------------------------------------------')
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
        monitor_times = np.array([3, 2, 2, 2, 1, 1, 1, 1, 1])  # Pj [2, 1, 2, 3, 4, 5, 5]
        t_matrix = np.array([[0, 3, 3, 3, 4, 4, 4, 1, 1],
                             [3, 0, 0, 0, 1, 1, 1, 2, 2],
                             [3, 0, 0, 0, 1, 1, 1, 2, 2],
                             [3, 0, 0, 0, 1, 1, 1, 2, 2],
                             [4, 1, 1, 1, 0, 0, 0, 3, 3],
                             [4, 1, 1, 1, 0, 0, 0, 3, 3],
                             [4, 1, 1, 1, 0, 0, 0, 3, 3],
                             [1, 2, 2, 2, 3, 3, 3, 0, 0],
                             [1, 2, 2, 2, 3, 3, 3, 0, 0]])  # Sjk

        due = np.array([0, 10, 20, 24, 8, 16, 24, 5, 10])  # dj
        charges = np.ones(n)*7
        i_times = 10  #max intervisit time
        slots = 12

    return t_matrix, due, monitor_times, slots, charges, i_times, membership


def mprint(m, solution, datam):
    print('\n\n\n------------------------------------------------------')
    print('---------------- MODEL DATA & RESULTS ----------------')
    print('------------------------------------------------------\n')
    print(solution['Solver'].message, '\n')
    print('The travel matrix is:\n', datam[0])
    print('\nDue dates are:\n', datam[1])
    print('\nMonitoring times are:\n', datam[2])
    print('\nOptimal objective value is:', value(m.obj_func))
    print('\nThe visiting assignments are as follow:')
    assign_list = np.zeros((len(datam[4]), datam[3]))
    assign_families = np.zeros((len(datam[4]), datam[3]))
    for ind in m.x.index_set():
        if value(m.x[ind]) == 1:
            assign_list[ind[2]-1, ind[1]-1] = ind[0]
            assign_families[ind[2] - 1, ind[1] - 1] = datam[6][ind[0]-1] + 1
    print(assign_list)
    print("\nOr in terms of families:")
    print(assign_families)
    list_c = sorted(list(m.c.index_set()), key=lambda x: x[1])
    c_values = []
    for ind in list_c:
        c_values.append(value(m.c[ind]))
        # print('c', ind, '=', value(m.c[ind]))
    c_values = np.reshape(c_values, (len(datam[4]), datam[3]))
    print("\nThe c values are as follow:")
    print(c_values)
    for ind in m.w.index_set():
        if value(m.w[ind]):
            print('w', ind, '=', value(m.w[ind]))


