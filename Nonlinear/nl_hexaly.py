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
    real_node_data = [1, 2, 3, 4, 5]
    real_node = m.range(1, 6)
    n_drones = len(drone_Charge_data)
    drone_Charge = m.array(drone_Charge_data)
    t_matrix = m.array(t_matrix_data)
    td_matrix = m.array(t_matrix_data[0])
    due_dates = m.array(due_dates_data)
    m_time = m.array(m_time_data)
    successors_data = [[] for i in range(n_node)]
    for fam in families_data:
        for i in fam:
            if i == 0:
                successors_data[i] = real_node_data
            else:
                successors_data[i] = fam[indexOf(fam, i)+1::]
    successors = m.array(successors_data)
    vis_sequences = [m.list(n_node) for k in range(n_drones)]
    m.constraint(m.cover(vis_sequences))
    end_time = [None] * n_drones
    str_time = [None] * n_drones
    lateness = [None] * n_drones
    seq_ = [None] * n_drones
    route_battery = [None] * n_drones
    loc_ = [None] * len(real_node_data)
    for i in real_node_data:
        m.constraint((m.contains(vis_sequences[n_drones-1], i)) == False)

    for k in range(n_drones):
        sequence = vis_sequences[k]
        c = m.count(sequence)
        m.constraint(c <= n_slot)
        battery_lambda = m.lambda_function(lambda i, prev: m.iif(i == 0, drone_Charge[k]-m.at(td_matrix, sequence[i])-m_time[sequence[i]], prev - m.at(t_matrix, sequence[i - 1], sequence[i]) - m_time[sequence[i]]))
        route_battery[k] = m.array(m.range(0, c), battery_lambda, -1000)
        quantity_lambda = m.lambda_function(lambda i: route_battery[k][i] >= 0)
        m.constraint(m.and_(m.range(0, c), quantity_lambda))
        # str_time_lambda = m.lambda_function(lambda i, prev: m.iif(i == 0, 0, end_time[k] + m.at(t_matrix, sequence[i-1], sequence[i])))
        end_time_lambda = m.lambda_function(lambda i, prev: m.iif(i != 0, prev + m.at(t_matrix, sequence[i-1], sequence[i]) + m_time[sequence[i]], m_time[sequence[i]]))
        # str_time[k] = m.array(m.range(0, c), str_time_lambda)
        end_time[k] = m.array(m.range(0, c), end_time_lambda)
        # sequence_lambda = m.lambda_function(lambda i: m.iif(m.contains(real_node, sequence[i]), m.and_(successors[i], end_time[k][i] < i_times + end_time[k][successors[i]]), 1))
        # seq_[k] = m.array(m.range(0, c),sequence_lambda)
        # m.constraint(m.sum(seq_[k]) == c)
        late_lambda = m.lambda_function(lambda i: end_time[k][i] - due_dates[sequence[i]])
        lateness[k] = m.max(m.range(0, c), late_lambda)
    # location_lambda = m.lambda_function(lambda i, k: m.contains(vis_sequences[k],i))
    loc_lambda = m.lambda_function(lambda i, k: m.contains(k, i), 0)
    ss = []
    for i in range(1, len(real_node_data)+1):
        for k in range(n_drones-1):
            sequence = vis_sequences[k]
            loc_[i-1] = m.sum(m.array(i, sequence, loc_lambda))
            ss.append(m.sum(m.array(i, sequence, loc_lambda)))

    max_lateness = m.max(lateness[0:n_drones-1])
    m.minimize(max_lateness)
    print(m.__str__())
    m.close()
    ls.param.time_limit = int(5)
    ls.param.seed = 1
    ls.solve()
    for i in range(n_drones):
        print('Drone Number', i+1, '-------------------------')
        print('sequence:', vis_sequences[i].value)
        # print('   start:', str_time[i].value)
        print('complete:', end_time[i].value)
        print('lateness:', lateness[i].value)
        print(' battery:',  route_battery[i].value)
    print(loc_[0])
