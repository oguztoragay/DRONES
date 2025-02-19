import hexaly.optimizer
from operator import indexOf
import pickle
import math

from hexaly.optimizer import HxModel





def expand_list(lst, n, m):
    # return [lst[0]] * n + lst + [lst[-1]] * m
    return lst[1: -1] + list(range(len(lst)-1, len(lst)+n+m-1))

def hexa(data, gen_seq, gen_st, gen_ct, av_time, b_res, verbose):
    with hexaly.optimizer.HexalyOptimizer() as ls:
        m = ls.model
        n_drones = data[0]
        t_matrix_data = list(data[2])
        n_slot = data[1]
        n_node = len(data[5])
        idle = n_slot * n_drones
        node_list = list(range(0, n_node))
        node_list = expand_list(node_list, n_drones, idle)
        n_drone_modi = math.ceil(len(node_list) / n_slot)

        real_nodes = node_list[0:n_node-2]
        depos = node_list[n_node-2: n_node-2+n_drones]
        idles = node_list[n_node+n_drones-2:]

        real_nodes_array = m.array(real_nodes)
        idles_array = m.array(idles)
        depos_array = m.array(depos)

        def map(it, s_depo=depos, s_real=real_nodes, s_idle=idle):
            if it in s_depo:
                map_ = 0
            if it in s_idle:
                map_ = n_slot[-1]
            if it in s_real:
                map_ = it
            return map_
        drone_charge = m.array([data[3][0]]*n_drone_modi)
        t_matrix = m.array(t_matrix_data)
        earliest = m.array(data[8])
        latest = m.array(data[6])
        m_time = m.array(data[5])
        # successors_data = [[] for i in range(n_node)]
        # families = list(map(lambda list: [item-1 for item in list], data[7]))
        # for fam in families:
        #     for i in fam:
        #         successors_data[i] = fam[indexOf(fam, i)+1::]
        vis_sequences = [m.list(node_list[-1]) for k in range(n_drone_modi)]
        for i in real_nodes:
            m.constraint(((m.contains(vis_sequences[d], i) for d in range(n_drones, n_drone_modi)) == False))
        # for i in real_node_data:
        #     m.constraint(m.sum(m.contains(vis_sequences[k], i) for k in range(n_drones)) == 1)
        m.constraint(m.partition(vis_sequences))

        end_time = [None] * n_drone_modi
        str_time = [None] * n_drone_modi
        lateness = [None] * n_drone_modi
        earlness = [None] * n_drone_modi
        route_battery = [None] * n_drone_modi
        cost_ = [None] * n_drones

        for k in range(n_drones):
            sequence = vis_sequences[k]
            c = m.count(sequence)
            m.constraint(c <= n_slot)
            for i in depos:
                m.constraint(sequence[0] != i)
            battery_lambda = m.lambda_function(lambda i, prev:
                                               m.iif(m.contains(real_nodes_array, sequence[i]),
                                                     prev - m_time[sequence[i]] - m.at(t_matrix, sequence[i-1], sequence[i]),
                                                     m.iif(m.contains(idles_array, sequence[i]),
                                                           prev - m.at(t_matrix, sequence[i-1], sequence[i]), drone_charge[k])))
            route_battery[k] = m.array(m.range(0, c), battery_lambda)

            quantity_lambda = m.lambda_function(lambda i: m.or_(route_battery[k][i] >= 0, sequence[i] == 0))
            # quantity_lambda = m.lambda_function(lambda i: route_battery[k][i] >= 0)
            m.constraint(m.and_(m.range(0, c), quantity_lambda))
            # m.constraint(m.and_([m.range(0, c), m.iif(route_battery[k][i] >= 0, sequence[i] >=1, True)]))


            end_time_lambda = m.lambda_function(lambda i, prev: m.iif(i != 0, prev + m.at(t_matrix, sequence[i-1], sequence[i]) + m_time[sequence[i]], m.at(t_matrix, 0, sequence[i]) + m_time[sequence[i]]))
            end_time[k] = m.array(m.range(0, c), end_time_lambda)

            str_time_lambda = m.lambda_function(lambda i: m.iif(i == 0, 0, end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i])))
            str_time[k] = m.array(m.range(0, c), str_time_lambda)

            late_lambda = m.lambda_function(lambda i: m.iif(m.or_(i == 0, i == n_node-1), -5000, end_time[k][i] - latest[sequence[i]])) # m.iif(m.or_(i == 0, i == n_node), -1000,
            earl_lambda = m.lambda_function(lambda i: m.iif(m.or_(i == 0, i == n_node-1), -5000, earliest[sequence[i]] - str_time[k][i]))
            lateness[k] = m.max(m.range(0, c), late_lambda)
            earlness[k] = m.max(m.range(0, c), earl_lambda)
            cost_[k] = m.sum(lateness[k], earlness[k])
            # lateness[k] = end_time[k][c]


            seq_rule = m.lambda_function(lambda i: m.iif(i == 0, 1, str_time[k][i] >= end_time[k][i-1]))
            m.constraint(m.and_(m.range(0, c), seq_rule))

        # vis_sequences_array = m.array(vis_sequences)
        # end_times = m.array(end_time)
        # str_times = m.array(str_time)
        # for i in real_node_data:
        #     if successors_data[i]:
        #         i_index = m.find(vis_sequences_array, i)
        #         i_list = m.at(vis_sequences_array, i_index)
        #         loc_i = m.index(i_list, i)
        #         ic_time = end_times[i_index][loc_i]
        #         j = successors_data[i][0]
        #         j_index = m.find(vis_sequences_array, j)
        #         j_list = m.at(vis_sequences_array, j_index)
        #         loc_j = m.index(j_list, j)
        #         jb_time = str_times[j_index][loc_j]
        #         jc_time = end_times[j_index][loc_j]
        #         # m.constraint(m.index(i_list, i) + 1 < m.index(j_list, j))
        #         m.constraint(jb_time - ic_time <= 1000)
        #         m.constraint(jb_time - ic_time >= 0)

        max_la = m.max(m.array(cost_))
        m.minimize(max_la)
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
        pickle.dump([gen_seq, gen_st, gen_ct, b_res, max_la.value], pickle_out)
        pickle_out.close()
        print('~~~~~~~~~~ HXL has been finalized ~~~~~~~~~~ -->', ls.solution.status.name)
    return gen_seq, gen_st, gen_ct, b_res
