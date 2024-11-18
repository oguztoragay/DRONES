# Working hexaly model that generates feasible solutions for all instances
# Cleaned on 05/26/2024
import localsolver
from operator import indexOf
import pickle

def hexa(data, gen_seq, gen_st, gen_ct, av_time, b_res, verbose):
    with localsolver.LocalSolver() as ls:
        m = ls.model
        n_drones = data[0]
        t_matrix_data = list(data[2])
        n_slot = data[1]
        n_node = len(data[5])
        real_node_data = list(range(1, n_node-1))
        drone_charge = m.array(data[3])
        t_matrix = m.array(t_matrix_data)
        # td_matrix = m.array(t_matrix_data[0])
        due_dates = m.array(data[6])
        m_time = m.array(data[5])
        successors_data = [[] for i in range(n_node)]
        families = list(map(lambda list: [item-1 for item in list], data[7]))
        for fam in families:
            for i in fam:
                successors_data[i] = fam[indexOf(fam, i)+1::]
        vis_sequences = [m.list(n_slot) for _ in range(n_drones)]

        end_time = [None] * n_drones
        str_time = [None] * n_drones
        lateness = [None] * n_drones
        route_battery = [None] * n_drones

        for k in range(n_drones):
            sequence = vis_sequences[k]
            c = n_slot-1
            m.constraint(sequence[0] != 0)
            battery_lambda = m.lambda_function(lambda i, prev:
                                               m.iif(i == 0, drone_charge[k] - m_time[sequence[i]] - m.at(t_matrix, 0, sequence[i]),
                                                     m.iif(sequence[i] == 0, drone_charge[k],
                                                     prev - m.at(t_matrix, sequence[i-1], sequence[i]) - m_time[sequence[i]])))
            route_battery[k] = m.array(m.range(0, c), battery_lambda,0)
            # quantity_lambda = m.lambda_function(lambda i: m.iif(route_battery[k][i] >= 0, True, sequence[i] == 0))
            quantity_lambda = m.lambda_function(lambda i: route_battery[k][i] >= 0)
            m.constraint(m.and_(m.range(0, c-1), quantity_lambda))
            # m.constraint(m.and_([m.range(0, c), m.iif(route_battery[k][i] >= 0, sequence[i] >=1, True)]))


            end_time_lambda = m.lambda_function(lambda i, prev: m.iif(i != 0, prev + m.at(t_matrix, sequence[i-1], sequence[i]) + m_time[sequence[i]], m.at(t_matrix, 0, sequence[i]) + m_time[sequence[i]]))
            end_time[k] = m.array(m.range(0, c), end_time_lambda,0)
            str_time_lambda = m.lambda_function(lambda i: m.iif(i == 0, 0, end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i])))
            str_time[k] = m.array(m.range(0, c), str_time_lambda)
            late_lambda = m.lambda_function(lambda i: m.iif(m.or_(i == 0, i == n_node), 0, end_time[k][i] - due_dates[sequence[i]])) # m.iif(m.or_(i == 0, i == n_node), -1000,
            lateness[k] = m.max(m.range(0, c), late_lambda)


            seq_rule = m.lambda_function(lambda i: m.iif(i == 0, 1, str_time[k][i] >= end_time[k][i-1]))
            m.constraint(m.and_(m.range(0, c), seq_rule))

        vis_sequences_array = m.array(vis_sequences)
        end_times = m.array(end_time)
        str_times = m.array(str_time)

        for i in real_node_data:
            # m.constraint(m.and_(m.range(0, n_drones), m.bool(m.contains(vis_sequences[k], i)))==False)
            # m.constraint(m.and_(m.range(0, n_drones), m.contains(vis_sequences[k], i)) == False)
            m.constraint(m.contains(vis_sequences[0], i) + m.contains(vis_sequences[1], i) == 1)
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
                # m.constraint(jb_time - ic_time <= data[4])
                # m.constraint(jb_time - ic_time >= 0)

        max_lateness = m.max(m.array(lateness))
        m.minimize(max_lateness)
        m.close()
        ls.param.time_limit = int(av_time)
        ls.param.verbosity = int(verbose)
        ls.solve()
        iis = ls.compute_inconsistency()
        print(iis)
        for k in range(n_drones):
            gen_seq.append([])
            gen_st.append([])
            gen_ct.append([])
            b_res.append([])
            for i1 in vis_sequences[k].value:
                gen_seq[-1].append(i1+1)
            for i2 in end_time[k].value:
                gen_ct[-1].append(round(i2, 4))
            for i3 in str_time[k].value:
                gen_st[-1].append(round(i3, 4))
            for i4 in route_battery[k].value:
                b_res[-1].append(round(i4, 4))
        # print(data[6])
        pickle_out = open('hxl.pickle', "wb")
        pickle.dump([gen_seq, gen_st, gen_ct, b_res, max_lateness.value], pickle_out)
        pickle_out.close()
        print('~~~~~~~~~~ HXL has been finalized ~~~~~~~~~~ -->', ls.solution.status.name)
    return gen_seq, gen_st, gen_ct, b_res

