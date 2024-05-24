import localsolver
from operator import indexOf

def hexa(data):
    with localsolver.LocalSolver() as ls:
        m = ls.model
        n_drones = data[0]
        t_matrix_data = list(data[2])
        n_slot = data[1]
        n_node = len(data[5])
        real_node_data = list(range(1, n_node))
        drone_charge = m.array(data[3])
        t_matrix = m.array(t_matrix_data)
        td_matrix = m.array(t_matrix_data[0])
        due_dates = m.array(data[6])
        m_time = m.array(data[5])
        successors_data = [[] for i in range(n_node)]
        families = list(map(lambda list: [item-1 for item in list], data[7]))
        for fam in families:
            for i in fam:
                if i == 1:
                    successors_data[i-1] = []  #real_node_data
                else:
                    successors_data[i-1] = fam[indexOf(fam, i)+1::]
        # successors = m.array(successors_data)
        # The only variable in the model is a list with length of all visits for each drone.
        vis_sequences = [m.list(n_node) for k in range(n_drones)]
        m.constraint(m.cover(vis_sequences))
        end_time = [None] * n_drones
        str_time = [None] * n_drones
        lateness = [None] * n_drones
        route_battery = [None] * n_drones
        for i in real_node_data:
            m.constraint((m.contains(vis_sequences[n_drones-1], i)) == False)
        # for k in range(n_drones-1):
        #     m.constraint(m.contains(vis_sequences[k], 1) == True)

        for k in range(n_drones):
            sequence = vis_sequences[k]
            c = m.count(sequence)
            m.constraint(c <= n_slot)
            battery_lambda = m.lambda_function(lambda i, prev:
                                               m.iif(i == 0, drone_charge[k] - m.at(td_matrix, sequence[i]) - m_time[sequence[i]],
                                                     m.iif(sequence[i] == 1, drone_charge[k],
                                                     prev - m.at(t_matrix, sequence[i - 1], sequence[i]) - m_time[sequence[i]])))
            route_battery[k] = m.array(m.range(0, c), battery_lambda)
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
        ls.param.time_limit = int(5)
        # ls.param.seed = 1
        ls.solve()
        for i in range(n_drones):
            print('Drone Number', i+1, '-------------------------')
            print('sequence:', vis_sequences[i].value)
            print('   start:', str_time[i].value)
            print('complete:', end_time[i].value)
            print('lateness:', lateness[i].value)
            print(' battery:',  route_battery[i].value)
    return None
