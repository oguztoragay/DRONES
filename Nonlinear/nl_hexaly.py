import localsolver
from localsolver.modeler import *
from localsolver import LSInterval
from localsolver import LSError
from random_instance import generate
from operator import itemgetter, indexOf

# Hexaly model for the problem-----------------------------------------------------------
with localsolver.LocalSolver() as ls:
    m = ls.model
    n_drones = 2
    datam = generate(n_drones, 'mini_fixed_modi')
    t_matrix_data, due_dates_data, m_time_data, n_slot_data, drone_Charge_data, \
        i_times, membership, families_data, f = datam
    t_matrix_data = list(t_matrix_data)
    n_slot = datam[3]
    n_node = len(m_time_data)
    n_drones = len(drone_Charge_data)
    drone_Charge = m.array(drone_Charge_data)
    t_matrix = m.array(t_matrix_data)
    td_matrix = m.array(t_matrix_data[0])
    due_dates = m.array(due_dates_data)
    m_time = m.array(m_time_data)
    successors = [[] for i in range(n_node)]
    for fam in families_data:
        for i in fam:
            successors[i-1] = fam[indexOf(fam, i)+1::]
    vis_sequences = [m.list(n_node) for k in range(n_drones)]
    m.constraint(m.partition(vis_sequences))
    end_time = [None] * n_drones
    lateness = [None] * n_drones
    for i in range(1, 7):
        m.constraint((m.contains(vis_sequences[n_drones-1], i)) == False)

    for k in range(n_drones):
        sequence = vis_sequences[k]
        c = m.count(sequence)
        m.constraint(c <= n_slot)
        # battery_lambda = m.lambda_function(lambda j, prev: m.at(t_matrix, sequence[j - 1], sequence[j]))
        # route_battery = m.sum(m.range(0, c), battery_lambda)
        # m.constraint(m.or_(route_battery <= drone_Charge[k], 0))
        # m.constraint(route_battery <= drone_Charge[k]+100)
        end_time_lambda = m.lambda_function(lambda i, prev: m.iif(i != 0, prev + m.at(t_matrix, sequence[i-1], sequence[i]) + m_time[sequence[i]], m_time[sequence[i]]))
        end_time[k] = m.array(m.range(0, c), end_time_lambda)
        late_lambda = m.lambda_function(lambda i: end_time[k][i] - due_dates[sequence[i]])
        lateness[k] = m.max(m.range(0, c), late_lambda)
    max_lateness = m.max(lateness[0:n_drones-1])
    m.minimize(max_lateness)
    print(m.__str__())
    m.close()
    ls.param.time_limit = int(30)
    # ls.param.seed = 1
    ls.solve()
    for i in range(n_drones):
        print('sequence:',vis_sequences[i].value)
        print('complete:',end_time[i].value)
        print('lateness:', lateness[i].value)
