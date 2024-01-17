from pyomo.environ import ConcreteModel, Var, Constraint, ConstraintList, NonNegativeReals, Binary, Integers, NonNegativeIntegers, Param, Objective, minimize, SolverFactory, value, maximize
import numpy as np


def generate(m, n, condition):
    print('-------------------------------------------------------------------------------------------------')
    t_matrix = np.random.randint(1, 4, size=(m, m))
    matrix_u = np.triu(t_matrix)
    matrix_l = matrix_u.T
    t_matrix = matrix_u + matrix_l
    np.fill_diagonal(t_matrix, 0)
    t_matrix = t_matrix.reshape((m, m))

    slots = 7 #np.random.randint(7, 8)
    charges = np.random.randint(13, 15, size=n)
    i_times = 2
    due = np.random.randint(15, 24, size=m+1)
    due[0] = 0
    # due[1] = 0
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
        i_times = 5  #max intervisit time
        slots = 12

    return t_matrix, due, monitor_times, slots, charges, i_times


def mprint(m, solution, datam):
    print('\n\n\n------------------------------------------------------')
    print('---------------- MODEL DATA & RESULTS ----------------')
    print('------------------------------------------------------\n')
    print(solution['Solver'].message, '\n')
    print('The travel matrix is:\n', datam[0])
    print('\nDue dates are:\n', datam[1])
    print('\nNumber of visits are:\n', datam[2])
    print('\nOptimal objective value is:', value(m.obj_func))
    print('\nThe visiting assignments are as follow:')
    assign_list = np.zeros((len(datam[4]), datam[3]))
    for ind in m.x.index_set():
        if value(m.x[ind]) == 1:
            assign_list[ind[2]-1, ind[1]-1] = ind[0]
    print(assign_list)
    for ind in m.c.index_set():
        print('c', ind, '=', value(m.c[ind]))
    # for ind in m.y.index_set():
    #     if value(m.y[ind]) == 1:
    #         print('y', ind, '=', value(m.y[ind]))
    #

