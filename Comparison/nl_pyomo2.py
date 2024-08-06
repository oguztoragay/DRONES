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

    # Pyomo Nonlinear (quadratic constrained) model for the problem-----------------
    m = ConcreteModel(name="Multiple drones QP model")
    m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)
    m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
    m.t = Var(slot_set, drones_set,  domain=NonNegativeReals, initialize=0, bounds=(0, full_charge))  # remaining charge AFTER visit completion
    m.lmax = Var(domain=NonNegativeReals, initialize=0) #
    m.obj_func = Objective(expr=m.lmax, sense=minimize)

    # Constraint:----------------------------- (1*)
    m.cons1 = ConstraintList()
    for j in demand_set-{1, idle}:
        m.cons1.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)

    # constraint:----------------------------- (2*)
    m.cons2 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons2.add(sum(m.x[j, r, i] for j in demand_set) == 1)

    # constraint:----------------------------- (3*)
    for i in drones_set:
        m.x[1, 1, i].fix(0)

    # constraint:----------------------------- (4*)
    m.cons4 = ConstraintList()
    for i in drones_set:
        m.cons4.add(m.c[1, i] == sum((t_matrix[0][j-1] + m_time[j-1]) * m.x[j, 1, i] for j in demand_set))

    # constraint:----------------------------- (5*)
    m.cons5 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons5.add(m.c[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.x[k, r-1, i]*m.x[j, r, i] for j in demand_set for k in demand_set) + sum(m_time[j - 1] * m.x[j, r, i] for j in demand_set))

    # constraint:----------------------------- (6*)
    for i in drones_set:
        m.s[1, i].fix(0)

    # constraint:----------------------------- (7*)
    m.cons7 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons7.add(m.s[r, i] == m.c[r, i] - sum(m_time[j - 1] * m.x[j, r, i] for j in demand_set))

    # constraint:----------------------------- (8*)
    m.cons8 = ConstraintList()
    for r in slot_set:
        for i in drones_set:
            m.cons8.add(m.lmax >= m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set))

    # constraint:----------------------------- (9*)
    m.cons9 = ConstraintList()
    for i in drones_set:
        m.cons9.add(m.t[1, i] == full_charge - m.c[1, i])

    # constraint:----------------------------- (10*)
    m.cons10 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10.add(m.t[r, i] == (full_charge * m.x[1, r, i]) + ((m.t[r - 1, i] - m.c[r, i] + m.c[r - 1, i]) * (1 - m.x[1, r, i])))

    # constraint:----------------------------- (11*)
    m.cons20 = ConstraintList()
    for f in families:
        for j in f:
            m.cons20.add(sum(m.s[r0, i0]*m.x[j+1, r0, i0] for r0 in slot_set for i0 in drones_set) - sum(m.c[r1, i1]*m.x[j, r1, i1] for r1 in slot_set for i1 in drones_set) <= i_times)

    # constraint:----------------------------- (12*)
    m.cons21 = ConstraintList()
    for f in families:
        for j in f:
            m.cons21.add(sum(m.s[r3, i3]*m.x[j+1, r3, i3] for r3 in slot_set for i3 in drones_set) - sum(m.c[r2, i2]*m.x[j, r2, i2] for r2 in slot_set for i2 in drones_set) >= 0)

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

    # msolver = SolverFactory('gurobi_persistent')
    # msolver.set_instance(m)
    # msolver.set_gurobi_param('FuncNonlinear', 1)
    # msolver.set_gurobi_param('LazyConstraints', 1)

    msolver = SolverFactory('gurobi')
    msolver.options['Threads'] = 20
    msolver.options['FeasibilityTol'] = 1e-7
    msolver.options['MIPFocus'] = 2
    msolver.options['Cuts'] = 3
    msolver.options['Heuristics'] = 1
    msolver.options['RINS'] = 5
    # msolver.options['SubMIPNodes'] = 1000
    msolver.options['PreQLinearize'] = 0
    msolver.options['BarCorrectors'] = 100
    msolver.options['PreMIQCPForm'] = 1

    solution = msolver.solve(m, tee=verbose)


    pickle_out = open('nlp.pickle', "wb")
    pickle.dump([m, solution, datam], pickle_out)
    pickle_out.close()
    print('~~~~~~~~~~ NLP has been finalized ~~~~~~~~~~ -->',  solution.Solver.Termination_condition)
    return None



