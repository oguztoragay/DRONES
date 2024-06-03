# Working hexaly model that generates feasible solutions for all instances
# Cleaned on 05/26/2024
import localsolver
from operator import indexOf

def hexa(data, gen_seq, gen_st, gen_ct, av_time):
    with localsolver.LocalSolver() as ls:
        m = ls.model
        n_drones = data[0]
        t_matrix_data = list(data[2])
        n_slot = data[1]
        n_node = len(data[5])
        real_node_data = list(range(1, n_node-1))
        drone_charge = m.array(data[3])
        t_matrix = m.array(t_matrix_data)
        td_matrix = m.array(t_matrix_data[0])
        due_dates = m.array(data[6])
        m_time = m.array(data[5])
        successors_data = [[] for i in range(n_node)]
        families = list(map(lambda list: [item-1 for item in list], data[7]))
        for fam in families:
            for i in fam:
                successors_data[i] = fam[indexOf(fam, i)+1::]
        vis_sequences = [m.list(n_node) for _ in range(n_drones)]
        m.constraint(m.cover(vis_sequences))
        for i in real_node_data:
            m.constraint(m.sum(m.contains(vis_sequences[k], i) for k in range(n_drones)) == 1)
        end_time = [None] * n_drones
        str_time = [None] * n_drones
        lateness = [None] * n_drones
        route_battery = [None] * n_drones

        for k in range(n_drones):
            sequence = vis_sequences[k]
            seq = m.array(sequence)
            c = m.count(sequence)
            m.constraint(c <= n_slot)
            battery_lambda = m.lambda_function(lambda i, prev:
                                               m.iif(i == 0, drone_charge[k] - m.at(td_matrix, sequence[i]) - m_time[sequence[i]],
                                                     m.iif(sequence[i] == 0, drone_charge[k],
                                                     prev - m.at(t_matrix, sequence[i - 1], sequence[i]) - m_time[sequence[i]])))
            route_battery[k] = m.array(m.range(0, c), battery_lambda)
            quantity_lambda = m.lambda_function(lambda i: route_battery[k][i] >= 0)
            m.constraint(m.and_(m.range(0, c), quantity_lambda))

            end_time_lambda = m.lambda_function(lambda i, prev: m.iif(i != 0, prev + m.at(t_matrix, sequence[i-1], sequence[i]) + m_time[sequence[i]], m_time[sequence[i]]))
            end_time[k] = m.array(m.range(0, c), end_time_lambda)
            str_time_lambda = m.lambda_function(lambda i: m.iif(i == 0, 0, end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i])))
            str_time[k] = m.array(m.range(0, c), str_time_lambda)
            late_lambda = m.lambda_function(lambda i: m.iif(sequence[i] == 0, 0, end_time[k][i] - due_dates[sequence[i]]))
            lateness[k] = m.max(m.range(0, c), late_lambda)

        vis_sequences_array = m.array(vis_sequences)
        end_times = m.array(end_time)
        str_times = m.array(str_time)
        for i in real_node_data:
            if successors_data[i]:
                i_index = m.find(vis_sequences_array, i)
                i_list = m.at(vis_sequences_array, i_index)
                loc_i = m.index(i_list, i)
                ic_time = end_times[i_index][loc_i]
                j = successors_data[i][0]
                j_index = m.find(vis_sequences_array, j)
                j_list = m.at(vis_sequences_array, j_index)
                loc_j = m.index(j_list, j)
                jb_time = str_times[j_index][loc_j]
                jc_time = end_times[j_index][loc_j]
                # m.constraint(m.index(i_list, i) + 1 < m.index(j_list, j))
                m.constraint(jb_time - ic_time <= data[4])
                m.constraint(jb_time - ic_time >= 0)

        max_lateness = m.max(lateness[0:n_drones])
        m.minimize(max_lateness)
        m.close()
        ls.param.time_limit = int(av_time)
        ls.solve()
        for k in range(n_drones):
            gen_seq.append([])
            gen_st.append([])
            gen_ct.append([])
            for i1 in vis_sequences[k].value:
                gen_seq[-1].append(i1+1)
            for i2 in end_time[k].value:
                gen_ct[-1].append(round(i2, 4))
            for i3 in str_time[k].value:
                gen_st[-1].append(round(i3, 4))
        # for i in range(n_drones):
        #     print('Drone Number', i+1, '-------------------------')
        #     print('Sequence:', vis_sequences[i].value)
        #     print('   start:', str_time[i].value)
        #     print('complete:', end_time[i].value)
        #     print('lateness:', lateness[i].value)
        #     print(' battery:', route_battery[i].value)
    return gen_seq, gen_st, gen_ct
