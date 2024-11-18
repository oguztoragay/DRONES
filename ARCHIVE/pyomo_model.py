from pyomo.environ import ConcreteModel, Var, Constraint, ConstraintList, NonNegativeReals, Binary, Integers, NonNegativeIntegers, Param, Objective, minimize, SolverFactory, value, maximize
import numpy as np
from itertools import combinations, product
from random_instance import generate
from random_instance import mprint
from pyomo.util.infeasible import find_infeasible_constraints

# n_demandnode = 12  # +Depot and +Idle
n_drones = 2

datam = generate(n_drones, 'SB_RS_LA')

t_matrix, due_dates, m_time, n_slot, drone_Charge, i_times, membership, families, f = datam

demand_set = set(range(1, len(due_dates) + 1))  # use index j for N locations
drones_set = set(range(1, n_drones + 1))  # use index i for M drones
slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
demand_set_combin = list(combinations(demand_set, 2))
demand_set_combin2 = [[i, j] for (i, j) in product(demand_set, demand_set)]
families = f

# fixed values:
B = 10000
UB = 1000  # upper bound of C

# Pyomo model for the problem-----------------------------------------------------------
m = ConcreteModel(name="Parallel Machines")
m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=0)
m.y = Var(demand_set, demand_set, slot_set, drones_set, domain=Binary, initialize=0)
m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
m.z = Var(slot_set, drones_set, domain=Binary, initialize=0)
m.w = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
m.v = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
m.e = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
m.lmax = Var(initialize=0, bounds=(0, 10))

m.obj_func = Objective(expr=m.lmax, sense=minimize)
# m.obj_func = Objective(expr=sum(m.c[r, i] for r in slot_set for i in drones_set), sense=minimize)

# Constraint 2:-------------------------------------------------------------------------- (3) in the model
m.cons2 = ConstraintList()
for j in demand_set-{1, len(demand_set)}:
    m.cons2.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)
# m.cons2.pprint()

# constraint 3:-------------------------------------------------------------------------- (4) in the model
m.cons3 = ConstraintList()
for i in drones_set:
    for r in slot_set:
        m.cons3.add(sum(m.x[j, r, i] for j in demand_set) <= 1)
# m.cons3.pprint()

for i in drones_set:
    m.x[1, 1, i].fix(0)
# m.x.pprint()
# constraint 7:------------------------------------------------------ My interpretation (...) in the model
# m.cons7 = ConstraintList()
# for j in demand_set-{len(demand_set)}:
#     for r in slot_set:
#         for i in drones_set - {len(drones_set)}:
#             m.cons7.add(m.x[j, r, i+1] + m.x[j, r, i] <= 2)
# m.cons7.pprint()

# constraint 8:-------------------------------------------------------------------------- (9) in the model
m.cons8 = ConstraintList()
for i in drones_set:
    m.cons8.add(m.c[1, i] == sum((t_matrix[0][j-1] + m_time[j-1]) * m.x[j, 1, i] for j in demand_set - {1, len(demand_set)}))
# m.cons8.pprint()

# constraint 9:-------------------------------------------------------------------------- (10a) in the model
m.cons9 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1}:
        m.cons9.add(m.c[r, i] - m.c[r-1, i] - sum(t_matrix[k-1, j-1]*m.y[j, k, r, i] for j in demand_set for k in demand_set) - sum(m_time[j-1] * m.x[j, r, i] for j in demand_set - {1, len(demand_set)}) == 0)
# m.cons9.pprint()

for i in drones_set:
    m.s[1, i].fix(0)

# constraint 9:-------------------------------------------------------------------------- (10a) in the model
m.cons91 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1}:
        m.cons91.add(m.s[r, i] - m.c[r-1, i] == 0)
# m.cons9.pprint()

# constraint 10 & 11:-------------------------------------------------------------------- (10b) & (10c)in the model
m.cons10 = ConstraintList()
m.cons11 = ConstraintList()
for i in drones_set:
    for jk in demand_set_combin2:
        j = jk[0]
        k = jk[1]
        for r in slot_set - {1}:
            m.cons10.add(m.x[j, r, i] + m.x[k, r-1, i] - m.y[j, k, r, i] <= 1)
            m.cons11.add(0.5*(m.x[j, r, i] + m.x[k, r-1, i]) >= m.y[j, k, r, i])
# m.cons10.pprint()
# m.cons11.pprint()

# constraint 12:-------------------------------------------------------------------------- (2) in the model
m.cons12 = ConstraintList()
for r in slot_set:
    for i in drones_set:
        m.cons12.add(m.lmax >= m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set) - B * (1 - sum(m.x[j, r, i] for j in demand_set)))
# m.cons12.pprint() #OK


# constraint 13:--------------------------------------------------------------------------
m.cons13 = ConstraintList()
for i in drones_set:
    m.cons13.add(m.c[1, i] - drone_Charge[i-1] <= B*m.z[2, i])
    # TODO: goes to depot more than once 111
    # fixme
#m.cons13.pprint() #OK

# constraint 15:--------------------------------------------------------------------------
m.cons15 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1, n_slot}:
        m.cons15.add(m.c[r, i] - sum(m.w[b, i] for b in range(1, r)) - drone_Charge[i-1] <= B*m.z[r+1, i])
# m.cons15.pprint() #OK
# Nasrin's comment: I changed the number of slots from 7 to 12. And changed b range from 1 to r-1.

# constraint 17 & 18 & 19:---------------------------------------------------------------
m.cons17 = ConstraintList()
m.cons18 = ConstraintList()
m.cons19 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1, 2}:
        m.cons17.add(m.w[r-1, i] >= m.c[r-1, i] - UB*(1 - m.z[r, i]))
        m.cons18.add(m.w[r-1, i] <= m.c[r-1, i])
        m.cons19.add(m.w[r-1, i] <= UB*m.z[r, i])
#m.cons17.pprint() #OK
#m.cons18.pprint() #OK
#m.cons19.pprint() #OK

# constraint 21:--------------------------------------------------------------------------
# Can be handled without a constraint... Pyomo magic
for i in drones_set:
    m.w[1, i].fix(0)

#for i in range(n_drones):
 #   for r in range(1,n_slot+1):
        #print(allocation5[r-1][i] >= 0)
  #      model += allocation5[r-1][i] >= 0


m.cons210 = ConstraintList()
for i in drones_set:
    m.cons210.add(m.c[1, i] - drone_Charge[i-1] >= -B * (1 - m.z[2, i]))
#m.cons210.pprint() #OK

m.cons211 = ConstraintList()
for i in drones_set:
    for r in slot_set - {1, n_slot}:
        m.cons211.add(m.c[r, i] - sum(m.w[b, i] for b in range(1, r)) - drone_Charge[i-1] >= -B*(1 - m.z[r + 1, i]))
#m.cons211.pprint() #OK
# Nasrin's comment: I changed the number of slots from 7 to 12. And changed b range from 1 to r-1.

# constraint 22:--------------------------------------------------------------------------
m.cons22 = ConstraintList()
for i in drones_set:
    for r in slot_set:
        m.cons22.add(m.x[1, r, i] == m.z[r, i])
# m.cons22.pprint() OK

m.cons_families_1 = ConstraintList()
m.cons_families_2 = ConstraintList()
# m.cons_families_3 = ConstraintList()
for f in families:
    for j in f:
        m.cons_families_1.add(sum(m.v[j + 1, r, i] - m.v[j, r, i] for r in slot_set for i in drones_set) <= i_times)
        m.cons_families_2.add(sum(m.v[j + 1, r, i] - m.v[j, r, i] for r in slot_set for i in drones_set) >= 0) #@@@@@@@@@@@@@@@@@@@@@ >=1

        # TODO: 4-3, 3-2 should be 4-2
        # fixme
        # for r in slot_set - {7}:
        #     for i in drones_set:
        #         m.cons_families_3.add(m.v[j, r, i] <= B*(1 - m.x[j, r, i]))
# m.cons_families_1.pprint()
# m.cons_families_2.pprint()
# m.cons_families_3.pprint()
# Nasrin's comment: Changed >=1 instead of <=1; Families are changed from 1 to 2; 1 is the depot. Also, changed the family ranges.
m.cons28feb = ConstraintList()
for f in families:
    for j in f:
        for i in drones_set-{len(drones_set)}:
            m.cons28feb.add(sum(m.e[j + 1, r, i+1] - m.v[j, r, i] for r in slot_set) >= 0)
            m.cons28feb.add(sum(m.e[j + 1, r, i] - m.v[j, r, i+1] for r in slot_set) >= 0)
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
# m.cons_dummy1 = ConstraintList()
# m.cons_dummy2 = ConstraintList()
# m.cons_dummy3 = ConstraintList()
# for j in demand_set:
#     for r in slot_set:
#         for i in drones_set:
            m.cons_dummy1.add(m.e[j, r, i] <= UB * m.x[j, r, i])
            m.cons_dummy2.add(m.e[j, r, i] <= m.s[r, i])
            m.cons_dummy3.add(m.e[j, r, i] >= m.s[r, i] - UB*(1 - m.x[j, r, i]))



# constraint 23:-------------------------------------------------------------------------- (18) in the model
m.cons23_families_1 = ConstraintList()
# m.cons23_families_2 = ConstraintList()
# m.cons23_families_3 = ConstraintList()
for f in families:
    for j in f:
        for r in slot_set - {n_slot}:
            for i in drones_set:
                m.cons23_families_1.add(sum(m.x[jj, r + 1, i] for jj in range(j+1, len(demand_set))) <= B*(1-m.x[j, r, i]))

        # for r in slot_set - {7}:
        #     for i in drones_set:
        #         m.cons_families_3.add(m.v[j, r, i] <= B*(1 - m.x[j, r, i]))
# m.cons23_families_1.pprint()
# m.cons23_families_2.pprint()
# m.cons23_families_3.pprint()

# constraint 24:--------------------------------------------------------------
m.cons24 = ConstraintList()
for r in slot_set:
    for i in drones_set:
        m.cons24.add(m.x[len(due_dates)-1, r, i] == 1 - sum(m.x[j, r, i] for j in demand_set-{len(due_dates)-1}))
# m.cons24.pprint()

# m.cons25 = ConstraintList()
# for r in slot_set:
#     for i in drones_set:
#         m.cons25.add(m.v[2, r, i] + 0.0001 <= m.v[3, r, i])
        # m.cons25.add(m.v[4, r, i] + 0.0001 <= m.v[5, r, i])
        # m.cons25.add(m.v[5, r, i] < m.v[6, r, i])

msolver = SolverFactory('gurobi')  # The following parameter set considered Gurobi as the solver
# msolver.options['TimeLimit'] = 300 # Time limit is set here
msolver.options['LogToConsole'] = 10
# msolver.options['DisplayInterval'] = 100
msolver.options['Threads'] = 24
msolver.options['FeasibilityTol'] = 1e-7
msolver.options['MIPFocus'] = 3
msolver.options['Cuts'] = 3
msolver.options['Heuristics'] = 1
msolver.options['RINS'] = 10
solution = msolver.solve(m, tee=True)
for constr, body_value, infeasible in find_infeasible_constraints(m):
    print(f"Constraint {constr.name} is infeasible: body_value={body_value}, infeasible={infeasible}")

mprint(m, solution, datam)


