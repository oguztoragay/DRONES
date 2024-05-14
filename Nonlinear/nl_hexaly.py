import localsolver
from localsolver.modeler import *
from random_instance import generate
from random_instance import mprint
from itertools import combinations, product
import math

# Hexaly model for the problem-----------------------------------------------------------
with localsolver.LocalSolver() as ls:
    m = ls.model
    n_drones = 3
    datam = generate(n_drones, 'mini_fixed')
    t_matrix_data, due_dates_data, m_time_data, n_slot_data, drone_Charge_data, \
        i_times, membership, families, f = datam
    t_matrix_data = list(t_matrix_data)
    n_slot = datam[3]
    n_node = len(m_time_data)
    drone_Charge = m.array(drone_Charge_data)
    t_matrix = m.array()
    td_matrix = m.array(t_matrix_data[0])
    due_dates = m.array(due_dates_data)
    m_time = m.array(m_time_data)
    earliest = m.array()
    for n in range(n_node):
        t_matrix.add_operand(m.array(t_matrix_data[n]))
    for n in range(n_node):
        earliest.add_operand(1)

    # demand_set = set(range(1, len(due_dates) + 1))  # use index j for N locations
    # drones_set = set(range(1, n_drones + 1))  # use index i for M drones
    # slot_set = set(range(1, n_slot + 1))  # use index r for R slots in each drone
    # demand_set_combin = list(combinations(demand_set, 2))
    # demand_set_combin2 = [[i, j] for (i, j) in product(demand_set, demand_set) if i!=j]
    # families = f
    # idle = len(demand_set)
    # B = 10000
    # UB = 10000



    # m.x = Var(demand_set, slot_set, drones_set, domain=Binary, initialize=1)
    vis_sequences = [m.list(n_slot) for k in range(n_drones)]
    m.constraint(m.cover(vis_sequences))
    dist_routes = [None] * n_drones
    end_time = [None] * n_drones
    home_lateness = [None] * n_drones
    lateness = [None] * n_drones

    drone_used = [(m.count(vis_sequences[k]) > 0) for k in range(n_drones)]
    nb_drones_used = m.sum(drone_used)
# m.s = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
# m.c = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
# m.z = Var(slot_set, drones_set, domain=Binary, initialize=0)
# # m.w = Var(slot_set, drones_set, domain=NonNegativeReals, initialize=0)
# m.v = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
# m.e = Var(demand_set, slot_set, drones_set, domain=NonNegativeReals, initialize=0)
# m.lmax = Var(initialize=0, domain=NonNegativeReals, bounds=(0, 5))
# m.obj_func = Objective(expr=m.lmax, sense=minimize)

    for k in range(n_drones):
        sequence = vis_sequences[k]
        c = m.count(sequence)

        # Distance traveled by each drone
        dist_lambda = m.lambda_function(lambda i: m.at(t_matrix, sequence[i - 1], sequence[i]))
        dist_routes[k] = m.sum(m.range(1, c), dist_lambda) + m.iif(c > 0, td_matrix[sequence[0]] + t_matrix[sequence[c - 1]], 0)

        # The battery needed in each route must not exceed the drone's battery charge
        battery_lambda = m.lambda_function(lambda j: m.at(t_matrix, sequence[j - 1], sequence[j]))
        route_battery = m.sum(sequence, battery_lambda)
        m.constraint(route_battery <= drone_Charge[k])

        # End of each visit
        end_time_lambda = m.lambda_function(lambda i, prev:m.max( earliest[sequence[i]],
                                                                  m.iif( i == 0, td_matrix[sequence[0]], prev + m.at(t_matrix, sequence[i - 1], sequence[i])))
                                                           + m_time[sequence[i]])
        end_time[k] = m.array(m.range(0, c), end_time_lambda, 0)
    print('wait here')
    #     # Arriving home after max horizon
    #     home_lateness[k] = m.iif(drone_used[k], m.max(0, end_time[k][c - 1] + td_matrix[sequence[c - 1]] - 10000), 0)
    #
    #     # Completing visit after latest end
    #     late_lambda = m.lambda_function(lambda i: m.max(0, end_time[k][i] - due_dates[sequence[i]]))
    #     lateness[k] = home_lateness[k] + m.sum(m.range(0, c), late_lambda)
    #
    # # Total lateness
    # total_lateness = m.sum(lateness)
    #
    # # Total distance traveled
    # total_distance = m.div(m.round(100 * m.sum(dist_routes)), 100)


# m.cons1 = ConstraintList()
# for j in demand_set-{1, idle}:
#     m.cons1.add(sum(m.x[j, r, i] for r in slot_set for i in drones_set) == 1)
#
# m.cons2 = ConstraintList()
# for i in drones_set:
#     for r in slot_set:
#         m.cons2.add(sum(m.x[j, r, i] for j in demand_set) <= 1)
#
# for i in drones_set:
#     m.x[1, 1, i].fix(0)
#
# m.cons4 = ConstraintList()
# for i in drones_set:
#     m.cons4.add(m.c[1, i] == sum((t_matrix[0][j-1] + m_time[j-1]) * m.x[j, 1, i] for j in demand_set - {1, idle}))
#
# m.cons5 = ConstraintList()
# for i in drones_set:
#     for r in slot_set - {1}:
#         m.cons5.add(m.c[r, i] == m.c[r - 1, i] + sum(t_matrix[k - 1, j - 1] * m.x[j, r, i] * m.x[k, r-1, i] for j in demand_set for k in demand_set) + sum(m_time[j - 1] * m.x[j, r, i] for j in demand_set))
#
# for i in drones_set:
#     m.s[1, i].fix(0)
#
# m.cons7 = ConstraintList()
# for i in drones_set:
#     for r in slot_set - {1}:
#         m.cons7.add(m.s[r, i] == m.c[r-1, i] + sum(t_matrix[k-1, j-1]*m.x[j, r, i]*m.x[k, r-1, i] for j in demand_set for k in demand_set))
#
# m.cons10 = ConstraintList()
# for r in slot_set:
#     for i in drones_set:
#         m.cons10.add(m.lmax >= m.c[r, i] - sum(due_dates[j-1] * m.x[j, r, i] for j in demand_set) - B * (1 - sum(m.x[j, r, i] for j in demand_set)))
#
# m.cons11 = ConstraintList()
# m.cons12 = ConstraintList()
# for i in drones_set:
#     m.cons11.add(m.c[1, i] - drone_Charge[i - 1] <= B * m.z[2, i])
#     m.cons12.add(m.c[1, i] - drone_Charge[i - 1] >= -B * (1 - m.z[2, i]))
#
# m.cons13 = ConstraintList()
# m.cons14 = ConstraintList()
# for i in drones_set:
#     for r in slot_set - {1, n_slot}:
#         m.cons13.add(m.c[r, i] - sum(m.c[b, i] * m.z[b+1, i] for b in range(1, r)) - drone_Charge[i - 1] <= B * m.z[r + 1, i])
#         m.cons14.add(m.c[r, i] - sum(m.c[b, i] * m.z[b+1, i] for b in range(1, r)) - drone_Charge[i - 1] >= -B * (1 - m.z[r + 1, i]))
#
# m.cons19 = ConstraintList()
# for i in drones_set:
#     for r in slot_set:
#         m.cons19.add(m.x[1, r, i] == m.z[r, i])
#
# m.cons20 = ConstraintList()
# for f in families:
#     for j in f:
#         m.cons20.add(sum(m.c[r, i]*m.x[j+1, r, i] - m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) <= i_times)
#
# m.cons21 = ConstraintList()
# for f in families:
#     for j in f:
#         m.cons21.add(sum(m.c[r, i]*m.x[j+1, r, i] - m.c[r, i]*m.x[j, r, i] for r in slot_set for i in drones_set) >= 0)
#
# m.cons22 = ConstraintList()
# for f in families:
#     for j in f:
#         m.cons22.add(sum(m.e[j+1, r, i] for r in slot_set for i in drones_set) >= sum(m.c[r, i] * m.x[j, r, i]  for r in slot_set for i in drones_set))
#
# m.cons24 = ConstraintList()
# m.cons25 = ConstraintList()
# m.cons26 = ConstraintList()
# for j in demand_set:
#     for r in slot_set:
#         for i in drones_set:
#             m.cons24.add(m.v[j, r, i] <= UB * m.x[j, r, i])
#             m.cons25.add(m.v[j, r, i] <= m.c[r, i])
#             m.cons26.add(m.v[j, r, i] >= m.c[r, i] - UB*(1 - m.x[j, r, i]))
#
# m.cons27 = ConstraintList()
# m.cons28 = ConstraintList()
# m.cons29 = ConstraintList()
# for j in demand_set:
#     for r in slot_set:
#         for i in drones_set:
#             m.cons27.add(m.e[j, r, i] <= UB * m.x[j, r, i])
#             m.cons28.add(m.e[j, r, i] <= m.s[r, i])
#             m.cons29.add(m.e[j, r, i] >= m.s[r, i] - UB*(1 - m.x[j, r, i]))
#
# m.cons30 = ConstraintList()
# for f in families:
#     for j in f:
#         for r in slot_set - {n_slot}:
#             for i in drones_set:
#                 m.cons30.add(sum(m.x[jj, r + 1, i] for jj in range(j+1, len(demand_set))) <= B*(1-m.x[j, r, i]))
#
# m.cons31 = ConstraintList()
# for r in slot_set:
#     for i in drones_set:
#         m.cons31.add(m.x[len(due_dates)-1, r, i] == 1 - sum(m.x[j, r, i] for j in demand_set-{len(due_dates)-1}))
