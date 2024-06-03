# Cleaned on 03/25/2024 (oguz)
# Multi-drone capable

from pyomo.environ import ConcreteModel, Var, Constraint, ConstraintList, NonNegativeReals, Binary, Integers, NonNegativeIntegers, Param, Objective, minimize, SolverFactory, value, maximize
from itertools import combinations, product
from random_instance import generate
from random_instance import mprint
import random

def nl_pyo(data, ws, ws_x, ws_y, ws_z):
    datam = data  #generate(n_drones, 'SB')
    t_matrix, due_dates, m_time, n_slot, drone_Charge, i_times, membership, families, f = datam
    demand_set = set(range(1, len(due_dates) + 1))  # use index j for N locations
    drones_set = set(range(1, len(data[4]) + 1))  # use index i for M drones
    slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
    demand_set_combin = list(combinations(demand_set, 2))
    demand_set_combin2 = [[i, j] for (i, j) in product(demand_set, demand_set) if i!=j]
    families = f
    idle = len(demand_set)

    B = 10000
    UB = 10000

    # Pyomo Nonlinear (quadratic constrained) model for the problem-----------------
    m = ConcreteModel(name="Multiple drones QP model")
    m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)
    m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.z = Var(slot_set, drones_set, domain=Binary, initialize=0)
    m.v = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.e = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.lmax = Var(initialize=0, domain=NonNegativeReals, bounds=(0, 5))
    m.obj_func = Objective(expr=m.lmax, sense=minimize)
    # if ws is not None:
    #     print('Hexaly results!------------------------------------')
    #     for i in ws:
    #         print(*i, sep=' --> ')
    # for i in m.x.index_set():
    #     if random.random() < 1:
    #         m.x[i] = ws_x[i]

    # Constraint 1:-------------------------------------------------------------------------- (1) in new model
    m.cons1 = ConstraintList()
    for j in demand_set-{1, idle}:
        m.cons1.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)

    # constraint 2:-------------------------------------------------------------------------- (2) in new model
    m.cons2 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons2.add(sum(m.x[j, r, i] for j in demand_set) == 1)

    # constraint 3:-------------------------------------------------------------------------- (3) in new model
    for i in drones_set:
        m.x[1, 1, i].fix(0)

    # constraint 4:-------------------------------------------------------------------------- (4) in new model
    m.cons4 = ConstraintList()
    for i in drones_set:
        m.cons4.add(m.c[1, i] == sum((t_matrix[0][j-1] + m_time[j-1]) * m.x[j, 1, i] for j in demand_set)) # -{1, idle}

    # constraint 5:-------------------------------------------------------------------------- (5) in new model
    m.cons5 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons5.add(m.c[r, i] == m.c[r - 1, i] + sum(t_matrix[k - 1, j - 1] * m.x[j, r, i] * m.x[k, r-1, i] for j in demand_set for k in demand_set) + sum(m_time[j - 1] * m.x[j, r, i] for j in demand_set))

    # constraint 6:-------------------------------------------------------------------------- (6) in new model
    for i in drones_set:
        m.s[1, i].fix(0)

    # constraint 7:-------------------------------------------------------------------------- (7) in new model
    m.cons7 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons7.add(m.s[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.x[j, r, i]*m.x[k, r-1, i] for j in demand_set for k in demand_set))

    # constraint 10:-------------------------------------------------------------------------- (10) in new model
    m.cons10 = ConstraintList()
    for r in slot_set:
        for i in drones_set:
            m.cons10.add(m.lmax >= m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set))# - B * (1 - sum(m.x[j, r, i] for j in demand_set)))
            # second part is not necessary anymore because we added idle and none of the slots will remain empty.

    # constraint 11 and 12:-------------------------------------------------------------------------- (11) & (12) in new model
    m.cons11 = ConstraintList()
    m.cons12 = ConstraintList()
    for i in drones_set:
        m.cons11.add(m.c[1, i] - drone_Charge[i - 1] <= B * m.z[2, i])
        m.cons12.add(m.c[1, i] - drone_Charge[i - 1] >= -B * (1 - m.z[2, i]))

    # constraint 13 and 14:-------------------------------------------------------------------------- (13) & (14) in new model
    m.cons13 = ConstraintList()
    m.cons14 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1, n_slot}:
            m.cons13.add(m.c[r, i] - sum(m.c[b, i] * m.z[b+1, i] for b in range(1, r)) - drone_Charge[i - 1] <= B * m.z[r + 1, i])
            m.cons14.add(m.c[r, i] - sum(m.c[b, i] * m.z[b+1, i] for b in range(1, r)) - drone_Charge[i - 1] >= -B * (1 - m.z[r + 1, i]))

    # constraint 19:-------------------------------------------------------------------------- (19) in new model
    m.cons19 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons19.add(m.x[1, r, i] == m.z[r, i])

    # constraint 20:-------------------------------------------------------------------------- (20) in new model
    m.cons20 = ConstraintList()
    for f in families:
        for j in f:
            m.cons20.add(sum(m.s[r, i]*m.x[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) <= i_times)

    # constraint 21:-------------------------------------------------------------------------- (21) in new model
    m.cons21 = ConstraintList()
    for f in families:
        for j in f:
            m.cons21.add(sum(m.s[r, i]*m.x[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) >= 0)

    # constraint 30:-------------------------------------------------------------------------- (30) in new model
    m.cons30 = ConstraintList()
    for f in families:
        for j in f:
            for r in slot_set - {n_slot}:
                for i in drones_set:
                    m.cons30.add(sum(m.x[jj, r + 1, i] for jj in range(j+1, len(demand_set))) <= B*(1-m.x[j, r, i]))

    # constraint 31:-------------------------------------------------------------------------- (31) in new model
    m.cons31 = ConstraintList()
    for r in slot_set:
        for i in drones_set:
            m.cons31.add(m.x[len(due_dates)-1, r, i] == 1 - sum(m.x[j, r, i] for j in demand_set-{len(due_dates)-1}))

    # m.pprint()
    num_of_cons = {}
    total_cons = 0
    for c in m.component_objects(Constraint):
        num_of_cons[c.name] = len(c)
        total_cons += len(c)

    num_of_var = {}
    total_var = 0
    for c in m.component_objects(Var):
        num_of_var[c.name] = len(c)
        total_var += len(c)
    # print('***** Total number of variables:%8d' %total_var)
    # print('***** Total number of constraints:%8d' %total_cons)
    # print('***** Variables =',num_of_var)
    # print('***** Constraints =',num_of_cons)
    msolver = SolverFactory('gurobi')
    # msolver = SolverFactory('scip', solver_io='nl', executable='C:/Users/oguzt/Desktop/solvers/scip/scip.exe', tee=True)
    msolver.options['Threads'] = 20
    msolver.options['FeasibilityTol'] = 1e-7
    msolver.options['MIPFocus'] = 2
    msolver.options['Cuts'] = 3
    msolver.options['Heuristics'] = 1
    msolver.options['RINS'] = 5
    msolver.options['SubMIPNodes'] = 100000
    solution = msolver.solve(m, warmstart= True, tee=True)
    print('\nNONLINEAR!------------------------------------')
    mprint(m, solution, datam)
    if ws is not None:
        print('Hexaly results!------------------------------------')
        for i in ws:
            print(*i, sep=' --> ')
    return None
