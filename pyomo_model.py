from pyomo.environ import ConcreteModel, Var, Constraint, ConstraintList, NonNegativeReals, Binary, Integers, NonNegativeIntegers, Param, Objective, minimize, SolverFactory, value, maximize
import numpy as np
from itertools import combinations
from pyomo.util.infeasible import find_infeasible_constraints
n_demandnode = 9  # plus depot = 6  Max visit numbers for each location:
n_drones = 2
n_slot = 7
demand_set = set(range(1, n_demandnode + 1))  # use index j for N locations
drones_set = set(range(1, n_drones + 1))  # use index i for M drones
slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
demand_set_combin = set(combinations(demand_set, 2))
monitor_time_matrix = np.array([3, 2, 2, 2, 1, 1, 1, 1, 1])  # Pj [2, 1, 2, 3, 4, 5, 5]
t_matrix = np.array([[0, 3, 3, 3, 4, 4, 4, 1, 1],
                     [3, 0, 0, 0, 1, 1, 1, 2, 2],
                     [3, 0, 0, 0, 1, 1, 1, 2, 2],
                     [3, 0, 0, 0, 1, 1, 1, 2, 2],
                     [4, 1, 1, 1, 0, 0, 0, 3, 3],
                     [4, 1, 1, 1, 0, 0, 0, 3, 3],
                     [4, 1, 1, 1, 0, 0, 0, 3, 3],
                     [1, 2, 2, 2, 3, 3, 3, 0, 0],
                     [1, 2, 2, 2, 3, 3, 3, 0, 0]])  # Sjk

B = 10000
due_dates = np.array([0, 24, 24, 24, 24, 24, 24, 5, 10])  # dj
Drone_Charge = 7  # 8 before
UB = 1000  # upper bound of C
i_matrix = 50  # 5 before MAX

m = ConcreteModel(name="Parallel Machines")
m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)  # allocation 1
m.y = Var(demand_set, demand_set, slot_set, drones_set, domain=Binary, initialize=0)  # allocation 2
m.c = Var(slot_set, drones_set, domain=Integers, initialize=0)  # allocation 3
m.z = Var(slot_set, drones_set, domain=Binary, initialize=0)  # allocation 4
m.w = Var(slot_set, drones_set, domain=Integers, initialize=0)  # allocation 5
m.v = Var(demand_set, slot_set, drones_set, domain=NonNegativeIntegers, initialize=0)  # allocation 6
m.lmax = Var(initialize=0, bounds=(0, 1000))  # We should discuss it
# Lmax= LpVariable ("Lmax", cat="Integer", lowBound= 0)  # when calculating Lmax do not include depot!!!

m.obj_func = Objective(expr=m.lmax, sense=minimize)

# Constraint 2:-------------------------------------------------------------------------- (3) in the model
m.cons2 = ConstraintList()
for j in demand_set - {1}:
    m.cons2.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) <= 1)
m.cons2.pprint()

# constraint 3:-------------------------------------------------------------------------- (4) in the model
m.cons3 = ConstraintList()
for r in slot_set:
    for i in drones_set - {1, 2}:
        m.cons3.add(sum(m.x[j, r, i] for j in demand_set) <= 1)
# m.cons3.pprint()

# constraint 4:--------------------------------------------------- No need to have this constraint  (5) in the model
# m.cons4 = ConstraintList()
# for j in demand_set:
#     for r in slot_set:
#         for i in drones_set:
#             m.cons4.add(m.x[j, r, i] <= 1)
# m.cons4.pprint()

# constraint 5: Depot cannot happen in the first slot of each drone ------------ (6) in the model
# Can be handled without a constraint... Pyomo magic
for i in drones_set:
    m.x[1, 1, i].fix(0)

# constraint 6:-------------------------------------------------------------------------- (7) in the model
m.cons6 = ConstraintList()
for r in slot_set - {1}:
    for i in drones_set:
        m.cons6.add(sum(m.x[j, r, i] - m.x[j, r-1, i] for j in demand_set - {1}) <= 0)
# m.cons6.pprint()

# constraint 7:-------------------------------------------------------------------------- My interpretation (...) in the model
m.cons7 = ConstraintList()
for j in demand_set:
    for r in slot_set:
        m.cons7.add(sum(m.x[j, r, i] for i in drones_set) <= 1)
# m.cons7.pprint()

# constraint 8:-------------------------------------------------------------------------- (9) in the model
m.cons8 = ConstraintList()
for i in drones_set:
    m.cons8.add(m.c[1, i] == sum((t_matrix[0][j-1] + monitor_time_matrix[j-1]) * m.x[j, 1, i] for j in demand_set - {1}))
# m.cons8.pprint()

# constraint 9:-------------------------------------------------------------------------- (10a) in the model
m.cons9 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1}:
        m.cons9.add(m.c[r, i] == m.c[r-1, i] + sum(t_matrix[jk[0]-1, jk[1]-1]*m.y[jk[0], jk[1], r, i] for jk in demand_set_combin) + sum(monitor_time_matrix[j-1] * m.x[j, r, i] for j in demand_set))
# m.cons9.pprint()

# constraint 10 & 11:-------------------------------------------------------------------- (10b) & (10c)in the model
m.cons10 = ConstraintList()
m.cons11 = ConstraintList()
for i in drones_set:
    for jk in demand_set_combin:
        j = jk[0]
        k = jk[1]
        for r in slot_set - {1}:
            m.cons10.add(m.x[j, r, i] + m.x[k, r-1, i] - 1 <= m.y[j, k, r, i])
            m.cons11.add(0.5*(m.x[j, r, i] + m.x[k, r-1, i]) >= m.y[j, k, r, i])
# m.cons10.pprint()
# m.cons11.pprint()

# constraint 12:-------------------------------------------------------------------------- (2) in the model
m.cons12 = ConstraintList()
for r in slot_set:
    for i in drones_set:
        m.cons12.add(m.lmax >= m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set) - B * (1 - sum(m.x[j, r, i] for j in demand_set)))
# m.cons12.pprint()

# constraint 13:--------------------------------------------------------------------------
m.cons13 = ConstraintList()
for i in drones_set:
    m.cons13.add(m.c[1, i] - Drone_Charge <= B*m.z[2, i])
# m.cons13.pprint()

# constraint 15:--------------------------------------------------------------------------
m.cons15 = ConstraintList()
for i in drones_set:
    for r in slot_set - {7}:
        m.cons15.add(m.c[r, i] - sum(m.w[b, i] for b in slot_set) - Drone_Charge <= B*m.z[r+1, i])
# m.cons15.pprint()

# constraint 17 & 18 & 19:---------------------------------------------------------------
m.cons17 = ConstraintList()
m.cons18 = ConstraintList()
m.cons19 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1}:
        m.cons17.add(m.w[r-1, i] >= m.c[r-1, i] - UB*(1 - m.z[r, i]))
        m.cons18.add(m.w[r-1, i] <= m.c[r-1, i])
        m.cons19.add(m.w[r-1, i] <= UB*m.z[r, i])
# m.cons17.pprint()
# m.cons18.pprint()
# m.cons19.pprint()

# constraint 21:--------------------------------------------------------------------------
# Can be handled without a constraint... Pyomo magic
for i in drones_set:
    m.w[1, i].fix(0)

m.cons210 = ConstraintList()
for i in drones_set:
    m.cons210.add(m.c[1, i] - Drone_Charge <= B*(m.w[1, i]))
# m.cons210.pprint()

m.cons211 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1, 7}:
        m.cons211.add(m.c[r, i] - sum(m.w[b, i] for b in slot_set) - Drone_Charge >= -B*(1 - m.w[r + 1, i]))
# m.cons211.pprint()

# constraint 22:--------------------------------------------------------------------------
m.cons22 = ConstraintList()
for i in drones_set:
    for r in slot_set:
        m.cons22.add(m.x[1, r, i] == m.w[r, i])
# m.cons22.pprint()

m.cons_families = ConstraintList()
families = [[1, 2, 3], [4, 5, 6], [7, 8]]
for f in families:
    for j in f:
        m.cons_families.add(sum(m.v[j+1, r, i] - m.v[j, r, i] for r in slot_set for i in drones_set) <= i_matrix)
        m.cons_families.add(sum(m.v[j+1, r, i] - m.v[j, r, i] for r in slot_set for i in drones_set) == 1)
        for r in slot_set - {7}:
            for i in drones_set:
                m.cons_families.add(m.x[j+1, r+1, i] <= B*(1 - m.x[j, r, i]))
m.cons_families.pprint()

#Job family constraint: #Dummy 1
m.cons_dummy1 = ConstraintList()
m.cons_dummy2 = ConstraintList()
m.cons_dummy3 = ConstraintList()
for j in demand_set:
    for r in slot_set:
        for i in drones_set:
            m.cons_dummy1.add(m.v[j, r, i] <= UB * m.x[j, r, i])
            m.cons_dummy2.add(m.v[j, r, i] <= m.c[r, i])
            m.cons_dummy3.add(m.v[j, r, i] >= m.c[r, i] - UB*(1 - m.x[j, r, i]))
# m.cons_dummy1.pprint()
# m.cons_dummy2.pprint()
# m.cons_dummy3.pprint()

# m.pprint()
msolver = SolverFactory('gurobi')  # The following parameter set considered Gurobi as the solver
# msolver.options['TimeLimit'] = 7200  # Time limit is set here
# msolver.options['LogToConsole'] = 1
# msolver.options['DisplayInterval'] = 100
# msolver.options['Threads'] = 16
# msolver.options['FeasibilityTol'] = 1e-5
# msolver.options['MIPFocus'] = 2
# msolver.options['Cuts'] = 3
# msolver.options['Heuristics'] = 1
# msolver.options['RINS'] = 10
solution = msolver.solve(m, tee=False)
print(solution['Solver'].message)
print(solution)
print(value(m.obj_func))
for constr, body_value, infeasible in find_infeasible_constraints(m):
    print(f"Constraint {constr.name} is infeasible: body_value={body_value}, infeasible={infeasible}")


