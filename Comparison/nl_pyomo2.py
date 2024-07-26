# Cleaned on 03/25/2024 (oguz)
# Cleaned on 07/25/2024 (oguz)

from pyomo.environ import (ConcreteModel, Var, Constraint, ConstraintList, NonNegativeReals,
                           Binary, Integers, NonNegativeIntegers, Param, Objective, minimize,
                           SolverFactory, value, maximize)
import pickle

def nl_pyo(data, verbose):
    datam = data
    t_matrix, due_dates, m_time, n_slot, drone_charge, i_times, membership, families, f = datam
    demand_set = set(range(1, len(due_dates) + 1))  # use index j for N locations
    drones_set = set(range(1, len(data[4]) + 1))  # use index i for M drones
    slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
    families = f
    idle = len(demand_set)

    full_charge = drone_charge[0]

    B = 10000

    # Pyomo Nonlinear (quadratic constrained) model for the problem-----------------
    m = ConcreteModel(name="Multiple drones QP model")
    m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)
    m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.t = Var(slot_set, drones_set,  domain=NonNegativeReals, initialize=0, bounds=(0, full_charge))  # remaining charge AFTER visit completion
    m.lmax = Var(initialize=0) #domain=NonNegativeReals,
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
        m.cons4.add(m.c[1, i] == sum((t_matrix[0][j-1] + m_time[j-1]) * m.x[j, 1, i] for j in demand_set))

    # constraint 5:-------------------------------------------------------------------------- (5) in new model
    m.cons5 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons5.add(m.c[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.x[k, r-1, i]*m.x[j, r, i] for j in demand_set for k in demand_set) + sum(m_time[j - 1] * m.x[j, r, i] for j in demand_set))

    # constraint 6:-------------------------------------------------------------------------- (6) in new model
    for i in drones_set:
        m.s[1, i].fix(0)

    # constraint 7:-------------------------------------------------------------------------- (7) in new model
    m.cons7 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            # m.cons7.add(m.s[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.x[k, r-1, i]*m.x[j, r, i] for j in demand_set for k in demand_set))
            m.cons7.add(m.s[r, i] == m.c[r, i] - sum(m_time[j - 1] * m.x[j, r, i] for j in demand_set))

    # constraint 10:-------------------------------------------------------------------------- (10) in new model
    m.cons10 = ConstraintList()
    for r in slot_set:
        for i in drones_set:
            # m.cons10.add(m.lmax >= m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set))
            m.cons10.add(m.lmax >= sum((m.c[r, i] - due_dates[j - 1]) * m.x[j, r, i] for j in demand_set))

    # constraint 11 and 12:-------------------------------------------------------------------------- (11) & (12) in new model
    m.cons11 = ConstraintList()
    for i in drones_set:
        m.cons11.add(m.t[1, i] == full_charge - m.c[1, i])

    m.cons13 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            # m.cons13.add(m.t[r, i] == (full_charge - m.c[r,i] + m.c[r-1, i]) * m.x[1, r-1, i] + (m.t[r-1, i] - m.c[r,i] + m.c[r-1, i]) * (1-m.x[1, r-1, i]))
            # m.cons13.add(m.t[r, i] == (full_charge - m.c[r,i]) * m.x[1, r-1, i] + (m.t[r-1, i] - m.c[r,i] + m.c[r-1, i]) * (1-m.x[1, r-1, i]))
            m.cons13.add(m.t[r, i] == (full_charge * m.x[1, r, i]) + ((m.t[r - 1, i] - m.c[r, i] + m.c[r - 1, i]) * (1 - m.x[1, r, i])))

    # constraint 20:-------------------------------------------------------------------------- (20) in new model
    m.cons20 = ConstraintList()
    for f in families:
        for j in f:
            m.cons20.add(sum(m.s[r, i]*m.x[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.c[r1, i1]*m.x[j, r1, i1] for r1 in slot_set for i1 in drones_set) <= i_times)

    # constraint 21:-------------------------------------------------------------------------- (21) in new model
    m.cons21 = ConstraintList()
    for f in families:
        for j in f:
            m.cons21.add(sum(m.s[r, i]*m.x[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) >= 0)

    # # constraint 30:-------------------------------------------------------------------------- (30) in new model
    # m.cons30 = ConstraintList()
    # for f in families:
    #     for j in f:
    #         for r in slot_set - {n_slot}:
    #             for i in drones_set:
    #                 m.cons30.add(sum(m.x[jj, r + 1, i] for jj in range(j+1, len(demand_set))) <= B*(1-m.x[j, r, i]))
    #
    # # constraint 31:-------------------------------------------------------------------------- (31) in new model
    # m.cons31 = ConstraintList()
    # for r in slot_set:
    #     for i in drones_set:
    #         m.cons31.add(m.x[len(due_dates)-1, r, i] == 1 - sum(m.x[j, r, i] for j in demand_set-{len(due_dates)-1}))

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
    msolver.options['Threads'] = 20
    msolver.options['FeasibilityTol'] = 1e-7
    msolver.options['MIPFocus'] = 2
    msolver.options['Cuts'] = 3
    msolver.options['Heuristics'] = 1
    msolver.options['RINS'] = 5
    msolver.options['SubMIPNodes'] = 1000
    msolver.options['PreQLinearize'] = 0
    msolver.options['BarCorrectors'] = 100
    msolver.options['PreMIQCPForm'] = 1
    # # msolver.options['Cutoff'] = 1
    # # msolver.options['SolutionLimit'] = 6

    solution = msolver.solve(m, warmstart=False, tee=verbose)


    pickle_out = open('nlp.pickle', "wb")
    pickle.dump([m, solution, datam], pickle_out)
    pickle_out.close()
    print('~~~~~~~~~~ NLP has been finalized ~~~~~~~~~~ -->',  solution.Solver.Termination_condition)
    return None



