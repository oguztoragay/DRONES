import localsolver
from random_instance import generate
from operator import indexOf

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
    route_battery = [None] * n_drones
    for i in real_node_data:
        m.constraint((m.contains(vis_sequences[n_drones-1], i)) == False)

    for k in range(n_drones):
        sequence = vis_sequences[k]
        c = m.count(sequence)
        m.constraint(c <= n_slot)
        battery_lambda = m.lambda_function(lambda i, prev: m.iif(i == 0, drone_Charge[k]-m.at(td_matrix, sequence[i])-m_time[sequence[i]],
                                                                 prev - m.at(t_matrix, sequence[i - 1], sequence[i]) - m_time[sequence[i]]))
        route_battery[k] = m.array(m.range(0, c), battery_lambda, -1000)
        quantity_lambda = m.lambda_function(lambda i: route_battery[k][i] >= 0)
        m.constraint(m.and_(m.range(0, c), quantity_lambda))
        end_time_lambda = m.lambda_function(lambda i, prev: m.iif(i != 0, prev + m.at(t_matrix, sequence[i-1], sequence[i]) + m_time[sequence[i]], m_time[sequence[i]]))
        end_time[k] = m.array(m.range(0, c), end_time_lambda)
        # str_time_lambda = m.lambda_function(lambda i: m.iif(i == 0, 0,
        #                                                     m.iif(m.contains(m.at(successors,sequence[i-1]), sequence[i]) == True,
        #                                                           end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i]) + i_times,
        #                                                           end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i]))))
        str_time_lambda = m.lambda_function(lambda i: m.iif(i == 0, 0, end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i])))
        str_time[k] = m.array(m.range(0, c), str_time_lambda)
        late_lambda = m.lambda_function(lambda i: end_time[k][i] - due_dates[sequence[i]])
        lateness[k] = m.max(m.range(0, c), late_lambda)

    vis_sequences_array = m.array(vis_sequences)
    end_times = m.array(end_time)
    str_times = m.array(str_time)
    for i in range(1, 5):
        if successors_data[i]:
            i_index = m.find(vis_sequences_array, i)
            i_list = m.at(vis_sequences_array, i_index)
            ic_time = m.at(m.at(end_times, i_index), i)
            for j in successors_data[i]:
                j_index = m.find(vis_sequences_array, j)
                j_list = m.at(vis_sequences_array, j_index)
                jb_time = m.at(m.at(str_times, j_index), j)
                # m.constraint(pick_up_list_index == delivery_list_index)
                # ic_time = m.array(end_time[i_index])[i_ind]
                # jb_time = str_time[j_index][j_ind]
                # m.constraint(ic_time + i_times <= jb_time )
                # m.constraint(i_ind <= j_ind)
                m.constraint(m.index(i_list, i) + 1 <= m.index(j_list, j))

    max_lateness = m.max(lateness[0:n_drones-1])
    m.minimize(max_lateness)
    # m.minimize(c)
    print(m.__str__())
    m.close()
    ls.param.time_limit = int(10)
    ls.param.seed = 1
    ls.solve()
    for i in range(n_drones):
        print('Drone Number', i+1, '-------------------------')
        print('sequence:', vis_sequences[i].value)
        print('   start:', str_time[i].value)
        print('complete:', end_time[i].value)
        print('lateness:', lateness[i].value)
        print(' battery:',  route_battery[i].value)
