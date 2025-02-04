import pickle
from pyomo.environ import (ConcreteModel, Var, ConstraintList, NonNegativeReals, Binary,
                           Objective, minimize, SolverFactory, Constraint)
from itertools import product

def lp_pyo(data, verbose):
    datam = data
    t_matrix, due_dates, m_time, n_slot, drone_charge, i_times, membership, families, f, due2, len_dl, fs_slot = data
    demand_set = set(range(1, len(due_dates) + 1))  
    drones_set = set(range(1, len(data[4]) + 1))  
    slot_set = set(range(1, n_slot + 1))  
    families = f
    idle = set([i for i in range(len(demand_set) - len_dl + 1, len(demand_set) + 1)])
    indexed_families = list(enumerate(families))

    # Big-M Values
    M = 10000
    UB = 10000
    full_charge = drone_charge[0]

    # === Pyomo Linear Model ===
    m = ConcreteModel(name="Multiple Drones LP Model")

    # === Decision Variables ===
    m.x = Var(demand_set, slot_set, drones_set, domain=Binary)
    m.y = Var(demand_set, demand_set, slot_set, drones_set, domain=Binary)
    m.s = Var(slot_set, drones_set, domain=NonNegativeReals, bounds=(0, 1440))
    m.c = Var(slot_set, drones_set, domain=NonNegativeReals, bounds=(0, 1440))
    m.t = Var(slot_set, drones_set, domain=NonNegativeReals, bounds=(0, full_charge))
    m.lmax = Var(domain=NonNegativeReals, bounds=(0, 1440))
    m.lmax2 = Var(domain=NonNegativeReals, bounds=(0, 1440))

    # Auxiliary Variables for Linearization
    m.z = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, bounds=(0, 1440))
    m.w = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, bounds=(0, 1440))
    m.v = Var(slot_set, drones_set, domain=NonNegativeReals)

    # === Objective Function ===
    m.obj_func = Objective(expr=m.lmax + m.lmax2, sense=minimize)

    # === Constraints ===

    # (1) & (2) Max Lateness and Earliness
    m.cons1_2 = ConstraintList()
    for j in demand_set - {1} - idle:
        m.cons1_2.add(m.lmax >= sum(m.w[j, r, i] for r in slot_set for i in drones_set) - due_dates[j - 1])
        m.cons1_2.add(m.lmax2 >= due2[j - 1] - sum(m.z[j, r, i] for r in slot_set for i in drones_set))

    # (3) Each Visit Happens Once
    m.cons3 = ConstraintList()
    for j in demand_set - {1} - idle:
        m.cons3.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)

    # (4) Each Slot Assigned Once
    m.cons4 = ConstraintList()
    for i in drones_set:
        for r in slot_set:
            m.cons4.add(sum(m.x[j, r, i] for j in demand_set) == 1)

    # (5) Initial Completion Time
    m.cons5 = ConstraintList()
    for i in drones_set:
        m.cons5.add(m.c[1, i] == sum((t_matrix[0][j - 1] + m_time[j - 1]) * m.x[j, 1, i] for j in demand_set - idle))

    # (6) Completion Time Linearization
    m.cons6 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons6.add(m.c[r, i] == m.c[r - 1, i] + sum(t_matrix[k - 1, j - 1] * m.y[j, k, r, i] for j in demand_set for k in demand_set))

    # (6a) Binary Product Linearization
    m.cons6a = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            for j, k in product(demand_set, repeat=2):
                m.cons6a.add(m.y[j, k, r, i] <= m.x[j, r, i])
                m.cons6a.add(m.y[j, k, r, i] <= m.x[k, r - 1, i])
                m.cons6a.add(m.y[j, k, r, i] >= m.x[j, r, i] + m.x[k, r - 1, i] - 1)

    # (8) Start Time Linearization
    m.cons8 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons8.add(m.s[r, i] == m.c[r, i] - sum(m.z[j, r, i] for j in demand_set))

    # (9) Battery Consumption
    m.cons9 = ConstraintList()
    for i in drones_set:
        m.cons9.add(m.t[1, i] == full_charge - sum((t_matrix[0][j - 1] + m_time[j - 1]) * m.x[j, 1, i] for j in demand_set - idle))

    # (10) Battery Update
    m.cons10 = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10.add(m.t[r, i] == full_charge * m.x[1, r, i] + m.t[r - 1, i] - m.c[r, i] + m.v[r, i])

    # (10a) Binary-Continuous Product Linearization
    m.cons10a = ConstraintList()
    for i in drones_set:
        for r in slot_set - {1}:
            m.cons10a.add(m.v[r, i] <= M * m.x[1, r, i])
            m.cons10a.add(m.v[r, i] >= -M * m.x[1, r, i])
            m.cons10a.add(m.v[r, i] <= m.c[r - 1, i])
            m.cons10a.add(m.v[r, i] >= m.c[r - 1, i] - M * (1 - m.x[1, r, i]))

    # === Solve the Model ===
    solver = SolverFactory("gurobi")
    solver.options["Threads"] = 24
    solver.options["FeasibilityTol"] = 1e-6
    solver.options["OptimalityTol"] = 1e-5
    solver.options["MIPFocus"] = 3
    solver.solve(m, tee=verbose)

    # Save Solution
    with open("lp.pickle", "wb") as f:
        pickle.dump([m], f)

    return m
