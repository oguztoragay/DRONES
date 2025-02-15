# Cleaned on 03/25/2024 (oguz)
# Cleaned on 07/25/2024 (oguz)
# Major update on 02/06/2025 (Oguz, Nasrin)
# Major update on 02/14/2025 (Oguz, Nasrin)
import pickle
from pyomo.environ import (ConcreteModel, Var, ConstraintList, NonNegativeReals, Binary,
                           Integers, NonNegativeIntegers, Objective, minimize, SolverFactory, Constraint)
from itertools import product
# from pyomo.util.infeasible import find_infeasible_constraints

def lp_pyo(data, verbose):
    datam = data
    t_matrix, due_dates, m_time, n_slot, drone_charge, i_times, membership, families, f, due2, len_dl, fs_slot = data
    demand_set = set(range(1, len(due_dates) + 1))  # use index j for N locations
    drones_set = set(range(1, len(data[4]) + 1))  # use index i for M drones
    slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
    demand_set_combin2 = [[i, j] for (i, j) in product(demand_set, demand_set)]
    families = f
    # idle = len(demand_set)
    idle = set([i for i in range(len(demand_set)-len_dl+1, len(demand_set)+1)])
    indexed_families = list(enumerate(families))
    UB = 1000000
    LB = 0
    full_charge = drone_charge[0]

    # Pyomo Linear model for the problem------------------------------------------------
    m = ConcreteModel(name="Multiple drones LP model")
    m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)
    m.y = Var(demand_set, demand_set, slot_set, drones_set, domain=Binary, initialize=0)
    m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))  # start time of a slot
    m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))  # completion time of a slot
    m.t = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0, bounds=(0, full_charge))  # remaining charge AFTER visit completion H_{r,i}

    m.u1 = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)  # t*x
    m.u2 = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)  # c*x
    m.u3 = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)  # c_{-1}*x
    m.z = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0, bounds=(0, 1440))  # start time of a node
    m.w = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0, bounds=(0, 1440))  # start time of a node
    m.a = Var(idle, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.g = Var(idle, slot_set, drones_set, domain=NonNegativeReals, initialize=0)

    m.lmax = Var(domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))
    m.lmax2 = Var(domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))

    m.obj_func = Objective(expr=m.lmax + m.lmax2, sense=minimize)

    # constraint: ++++++++++++++++++++++++++++++ (1__ & 2__)
    m.cons1_2 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons1_2.add(m.lmax >= (m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set)))
            m.cons1_2.add(m.lmax2 >= sum(due2[j - 1] * m.x[j, r, i] for j in demand_set) - m.s[r, i])

    # constraint: ++++++++++++++++++++++++++++++  (3__)
    m.cons3 = ConstraintList()
    for j in demand_set-{1}-idle:
        m.cons3.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)

    # constraint: ++++++++++++++++++++++++++++++  (4__)
    m.cons4 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons4.add(sum(m.x[j, r, i] for j in demand_set) == 1)

    # constraint: ++++++++++++++++++++++++++++++  (5__)
    m.cons5 = ConstraintList()
    for i in drones_set:
        m.cons5.add(m.c[1, i] == sum((t_matrix[0][j - 1] + m_time[j-1]) * m.x[j, 1, i] for j in demand_set-idle) + sum(m.a[j, 1, i] for j in idle))
    m.cons5a = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            for j in idle:
                m.cons5a.add(m.a[j, r, i] >= m.g[j, r, i] - UB * (1 - m.x[j, r, i]))
                m.cons5a.add(m.a[j, r, i] <= m.g[j, r, i] + LB * (1 - m.x[j, r, i]))
                m.cons5a.add(m.a[j, r, i] <= UB * m.x[j, r, i])

    # constraint: ++++++++++++++++++++++++++++++  (6a__)
    m.cons6a = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons6a.add(m.c[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.y[j, k, r, i] for j in demand_set for k in demand_set) + sum(m_time[j-1]*m.x[j, r, i] for j in demand_set-idle) + sum(m.a[j, r, i] for j in idle))

    # constraint: ++++++++++++++++++++++++++++++  (6b__ & 6c__)
    m.cons6b = ConstraintList()
    m.cons6c = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            for jk in demand_set_combin2:
                j = jk[0]
                k = jk[1]
                m.cons6b.add(m.x[j, r, i] + m.x[k, r-1, i] - 1 <= m.y[j, k, r, i])
                m.cons6c.add(m.y[j, k, r, i] <= 0.5 * (m.x[j, r, i] + m.x[k, r - 1, i]))

    # constraint: ++++++++++++++++++++++++++++++  (7__)
    for i in drones_set:
        m.s[1, i].fix(0)

    # constraint: ++++++++++++++++++++++++++++++  (8__)
    m.cons8 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons8.add(m.s[r, i] == m.c[r, i] - sum(m_time[j-1]*m.x[j, r, i] for j in demand_set-idle) - sum(m.a[j, r, i] for j in idle))

    # constraint: ++++++++++++++++++++++++++++++  (9__)
    m.cons9 = ConstraintList()
    for i in drones_set:
        m.cons9.add(m.t[1, i] == full_charge - sum(t_matrix[0][j-1] * m.x[j, 1, i] for j in demand_set-idle) - sum(m_time[j-1] * m.x[j, 1, i] for j in demand_set-idle))

    # constraint: ++++++++++++++++++++++++++++++  (10a__)
    m.cons10a = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10a.add(m.t[r, i] == (full_charge * m.x[1, r, i]) + m.u1[r, i] - m.u2[r, i] - m.u3[r, i])

    # constraint: ++++++++++++++++++++++++++++++  (10b__ & 10c__ & 10d__ & 10e__)
    m.cons10b = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10b.add(m.u1[r, i] <= (m.t[r-1, i] + m.c[r-1, i]) + (UB * (m.x[1, r, i])))
            m.cons10b.add(m.u1[r, i] >= (m.t[r-1, i] + m.c[r-1, i]) - (UB * (m.x[1, r, i])))
            m.cons10b.add(m.u1[r, i] <= UB * (1 - m.x[1, r, i]))
            m.cons10b.add(m.u1[r, i] >= -UB * (1 - m.x[1, r, i]))

    # constraint: ++++++++++++++++++++++++++++++  (10f__ & 10g__ & 10h__ & 10i__)
    m.cons10c = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10c.add(m.u2[r, i] <= m.c[r, i] + (UB * (1 - sum(m.x[j, r, i] for j in demand_set-idle-{1}))))
            m.cons10c.add(m.u2[r, i] >= m.c[r, i] - (UB * (1 - sum(m.x[j, r, i] for j in demand_set-idle-{1}))))
            m.cons10c.add(m.u2[r, i] <= UB * sum(m.x[j, r, i] for j in demand_set-idle-{1}))
            m.cons10c.add(m.u2[r, i] >= -UB * sum(m.x[j, r, i] for j in demand_set-idle-{1}))

    # # constraint: ++++++++++++++++++++++++++++++  (10f__ & 10g__ & 10h__ & 10i__)
    m.cons10d = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10d.add(m.u3[r, i] <= m.s[r, i] + (UB * (1 - sum(m.x[j, r, i] for j in idle))))
            m.cons10d.add(m.u3[r, i] >= m.s[r, i] - (UB * (1 - sum(m.x[j, r, i] for j in idle))))
            m.cons10d.add(m.u3[r, i] <= UB * sum(m.x[j, r, i] for j in idle))
            m.cons10d.add(m.u3[r, i] >= -UB * sum(m.x[j, r, i] for j in idle))

    # constraint: ++++++++++++++++++++++++++++++  (11a__)
    m.cons11a = ConstraintList()
    for ind_, f in indexed_families:
        for j in f:
            m.cons11a.add(sum(m.z[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.w[j, r, i] for r in slot_set for i in drones_set) <= i_times[ind_])

    # constraint: ++++++++++++++++++++++++++++++  (12a__)
    m.cons12a = ConstraintList()
    for ind_, f in indexed_families:
        for j in f:
            m.cons12a.add(sum(m.z[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.w[j, r, i] for r in slot_set for i in drones_set) >= 0)

    # constraint: ++++++++++++++++++++++++++++++  (11b__ & 11c__ & 11d__ & 11e)
    m.cons11b = ConstraintList()
    m.cons11c = ConstraintList()
    m.cons11d = ConstraintList()
    m.cons11e = ConstraintList()
    for j in demand_set-idle:
        for r in slot_set:
            for i in drones_set:
                m.cons11b.add(m.z[j, r, i] >= m.s[r, i] - UB * (1 - m.x[j, r, i]))
                m.cons11c.add(m.z[j, r, i] <= m.s[r, i] + LB * (1 - m.x[j, r, i]))
                m.cons11d.add(m.z[j, r, i] <= UB * m.x[j, r, i])
                m.cons11e.add(m.z[j, r, i] >= LB * m.x[j, r, i])

    # constraint: ++++++++++++++++++++++++++++++  (11f__ & 11g__ & 11h__ & 11i__)
    m.cons11f = ConstraintList()
    m.cons11g = ConstraintList()
    m.cons11h = ConstraintList()
    m.cons11i = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            for j in demand_set:
                m.cons11f.add(m.w[j, r, i] <= m.c[r, i] + UB * (1 - m.x[j, r, i]))
                m.cons11g.add(m.w[j, r, i] >= m.c[r, i] - UB * (1 - m.x[j, r, i]))
                m.cons11h.add(m.w[j, r, i] <= UB * (m.x[j, r, i]))
                m.cons11i.add(m.w[j, r, i] >= -UB * (m.x[j, r, i]))


    # Info about the model:------------------------------------------
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
    # print('***** Total number of variables:%8d' % total_var)
    # print('***** Total number of constraints:%8d' % total_cons)
    # print('***** Variables =', num_of_var)
    # print('***** Constraints =', num_of_cons)

    msolver = SolverFactory('gurobi')
    msolver.options['Threads'] = 24
    msolver.options['FeasibilityTol'] = 1e-6
    msolver.options['OptimalityTol'] = 1e-5
    msolver.options['MIPFocus'] = 2
    msolver.options['Cuts'] = 3
    msolver.options['Heuristics'] = 1
    msolver.options['RINS'] = 5
    msolver.options['TimeLimit'] = 3600
    msolver.options['VarBranch'] = 3
    msolver.options['Presolve'] = 2
    # msolver.options['PoolSolutions'] = 5
    # msolver.options['SubMIPCuts'] = 2
    # msolver.options['SubMIPNodes'] = 500
    # msolver.options['PreQLinearize'] = 0
    # msolver.options['BarCorrectors'] = 100
    # msolver.options['PreMIQCPForm'] = 1
    # msolver.options['Cutoff'] = 1500

    solution = msolver.solve(m, tee=verbose)
    # for constr, body_value, infeasible in find_infeasible_constraints(m):
    #     print(f"Constraint {constr.name} is infeasible: {constr.expr}")
    # for i in m.u1.index_set():
    #     print(i,':', [value(m.u1[i]), value(m.u2[i]), value(m.u3[i]), value(m.t[i])])



    pickle_out = open('lp.pickle', "wb")
    pickle.dump([m, solution, datam, total_var, total_cons], pickle_out)
    pickle_out.close()
    print('~~~~~~~~~~  LP has been finalized ~~~~~~~~~~ -->', solution.Solver.Termination_condition)
    return None


