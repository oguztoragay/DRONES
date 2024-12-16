# Cleaned on 03/25/2024 (oguz)
# Cleaned on 07/25/2024 (oguz)
# Cleaned on 08/07/2024 (oguz)
# Cleaned on 09/22/2024 (oguz)
# Major update: Idles does not have fixed m_time that means drones can stay on idle points for as long as they need to without losing the remaining charges. 12/11/2024 (oguz)

from pyomo.environ import (ConcreteModel, Var, ConstraintList, NonNegativeReals,
                           Binary, Integers, NonNegativeIntegers, Objective, minimize,
                           SolverFactory, Constraint)
import pickle

def nl_pyo(data, verbose):
    datam = data
    t_matrix, due_dates, m_time, n_slot, drone_charge, i_times, membership, families, f, due2, len_dl, fs_slot = datam
    demand_set = set(range(1, len(due_dates) + 1))  # use index j for N locations
    drones_set = set(range(1, len(data[4]) + 1))  # use index i for M drones
    slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
    families = f
    # idle = len(demand_set)
    idle = set([i for i in range(len(demand_set)-len_dl+1, len(demand_set)+1)])
    indexed_families = list(enumerate(families))
    full_charge = drone_charge[0]

    # Pyomo Nonlinear (quadratic constrained) model for the problem-----------------
    m = ConcreteModel(name="Multiple drones QP model")
    m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)
    m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0, bounds=(0, 1440))
    m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))
    m.t = Var(slot_set, drones_set,  domain=NonNegativeReals, initialize=full_charge, bounds=(0, full_charge))  # remaining charge AFTER visit completion H_{r,i}
    m.d = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0, bounds=(0, 1440))  # duration of the visit which is fixed for demand nodes and variable for idle nodes
    m.lmax = Var(domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))
    m.lmax2 = Var(domain=NonNegativeReals, initialize=1440, bounds=(0, 1440))

    m.obj_func = Objective(expr=m.lmax + m.lmax2, sense=minimize)

    # constraint:----------------------------- (1__ & 2__)
    m.cons1_2 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons1_2.add(m.lmax >= (m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set)))
            m.cons1_2.add(m.lmax2 >= sum(due2[j - 1] * m.x[j, r, i] for j in demand_set) - m.s[r, i])

    # constraint:----------------------------- (3__)
    m.cons3 = ConstraintList()
    for j in demand_set-{1}-idle:
        m.cons3.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)

    # constraint:----------------------------- (4__)
    m.cons4 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons4.add(sum(m.x[j, r, i] for j in demand_set) == 1)

    # constraint:----------------------------- (5__)
    m.cons5 = ConstraintList()
    for i in drones_set:
        m.cons5.add(m.c[1, i] == sum((t_matrix[0][j-1] + m.d[j, 1, i]) * m.x[j, 1, i] for j in demand_set))

    # constraint:----------------------------- (6__)
    m.cons6 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons6.add(m.c[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.x[k, r-1, i]*m.x[j, r, i] for j in demand_set for k in demand_set) + sum(m.d[j, r, i] * m.x[j, r, i] for j in demand_set))

    # constraint:----------------------------- (7__)
    for i in drones_set:
        m.s[1, i].fix(0)

    # constraint:----------------------------- (8__)
    m.cons8 = ConstraintList()
    for i in drones_set:
        for r in slot_set-{1}:
            m.cons8.add(m.s[r, i] == m.c[r, i] - sum(m.d[j, r, i] * m.x[j, r, i] for j in demand_set))

    # constraint:----------------------------- (9__)
    m.cons9 = ConstraintList()
    for i in drones_set:
        m.cons9.add(m.t[1, i] == full_charge - sum((t_matrix[0][j-1] + m.d[j, 1, i]) * m.x[j, 1, i] for j in demand_set-idle))

    # constraint:----------------------------- (10__)
    m.cons10 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10.add(m.t[r, i] == (full_charge * m.x[1, r, i]) + ((m.t[r-1, i] - m.c[r, i] + m.c[r - 1, i]) * (sum(m.x[j, r, i] for j in demand_set-{1}-idle))) + ((m.t[r-1, i]-(m.s[r, i] - m.c[r-1, i])) * sum(m.x[id_, r, i] for id_ in idle)))

    # constraint:----------------------------- (11__)
    m.cons11 = ConstraintList()
    for ind_, f in indexed_families:
        for j in f:
            m.cons11.add(sum(m.s[r, i]*m.x[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) <= i_times[ind_])
            m.cons11.add(sum(m.s[r, i]*m.x[j+1, r, i] for r in slot_set for i in drones_set) - sum(m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) >= 0)

    # constraint:----------------------------- (New fixed variables)
    for i in drones_set:
        for r in slot_set:
            for j in demand_set-idle:
                m.d[j, r, i].fix(m_time[j-1])

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

    # msolver = SolverFactory('gurobi_persistent')
    # msolver.set_instance(m)
    # msolver.set_gurobi_param('FuncNonlinear', 1)
    # msolver.set_gurobi_param('LazyConstraints', 1)

    msolver = SolverFactory('gurobi')
    msolver.options['Threads'] = 24
    msolver.options['FeasibilityTol'] = 1e-6
    msolver.options['OptimalityTol'] = 1e-5
    msolver.options['MIPFocus'] = 2
    msolver.options['Cuts'] = 3
    msolver.options['Heuristics'] = 1
    msolver.options['RINS'] = 5
    msolver.options['PreQLinearize'] = 0
    msolver.options['BarCorrectors'] = 3
    msolver.options['PreMIQCPForm'] = 2
    msolver.options['Presolve'] = 2
    msolver.options['TimeLimit'] = 3600
    # msolver.options['PoolSolutions'] = 5
    solution = msolver.solve(m, tee=verbose)


    pickle_out = open('nlp.pickle', "wb")
    pickle.dump([m, solution, datam, total_var, total_cons], pickle_out)
    pickle_out.close()
    print('~~~~~~~~~~ NLP has been finalized ~~~~~~~~~~ -->',  solution.Solver.Termination_condition)
    return None

