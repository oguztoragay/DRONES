from itertools import count
from operator import indexOf
import hexaly.optimizer
import random
import pickle


def map_sequence(sequence, depos, real_nodes, idles):
    depot_set = set(depos)
    idle_set = set(idles)
    idle_replacement = len(real_nodes) + 2
    mapped_sequence = []
    for i in range(len(sequence)):
        mapped_sequence.append([])
        seq = sequence[i]
        for node in seq:
            if node in depot_set:
                mapped_sequence[-1].append(1)  # Replace depot nodes with 1
            elif node in idle_set:
                mapped_sequence[-1].append(idle_replacement)  # Replace idle nodes with max_real_node + 1
            else:
                mapped_sequence[-1].append(node - len(depos) + 2)  # Keep real nodes as is
    return mapped_sequence

def hexa(data, gen_seq, gen_st, gen_ct, av_time, b_res, verbose):
    with hexaly.optimizer.HexalyOptimizer() as hex:
        # random.seed(seed1)
        m = hex.model
        n_drones = len(data[4])
        t_matrix_data = list(data[0])
        n_slot = data[3]
        n_node = len(data[0])

        real_data = [i for i in range(len(data[2])) if (data[2][i] != 0 and data[2][i] != 60)]
        real_set = m.set(n_node)
        depot_data = [x - 1 for x in range(1, real_data[0] + 1)]  # Convert depot indices to 0-based
        depot_set = m.set(n_node)
        idle_data = [x for x in range(real_data[-1] + 1, len(data[2]))]  # Convert idle node indices to 0-based
        idle_set = m.set(n_node)

        for i in range(n_node):
            if i in real_data:
                m.constraint(m.contains(real_set, i))
            else:
                m.constraint(m.not_(m.contains(real_set, i)))
            if i in idle_data:
                m.constraint(m.contains(idle_set, i))
            else:
                m.constraint(m.not_(m.contains(idle_set, i)))
            if i in depot_data:
                m.constraint(m.contains(depot_set, i))
            else:
                m.constraint(m.not_(m.contains(depot_set, i)))

        drone_charge = m.array(data[4])
        t_matrix = m.array(t_matrix_data)
        i_times = m.array(data[5])
        due_dates = m.array(data[1])
        due2_dates = m.array(data[9])
        m_time = data[2][:len(real_data)+len(depot_data)]

        families = list(map(lambda lst: [x - 1 for x in lst], data[7]))
        successors_data = {i: [] for i in range(n_node)}
        for fam in families:
            for i, node in enumerate(fam[:-1]):
                successors_data[node] = fam[i + 1:]

        vis_sequences = [m.list(n_node) for _ in range(n_drones)]
        m.constraint(m.partition(vis_sequences))
        for d in range(int(n_drones / 2), n_drones):
            for i in real_data:
                m.constraint(m.not_(m.contains(vis_sequences[d], i)))
        end_time = [None] * n_drones
        str_time = [None] * n_drones
        lateness = [None] * n_drones
        earliness = [None] * n_drones
        route_battery = [None] * n_drones
        nm_time = m_time + [m.float(0, 1440) for _ in range(len(idle_data))]
        nm_times = m.array(*nm_time)
        for k in range(n_drones):
            sequence = vis_sequences[k]
            c = m.count(sequence)
            m.constraint(c == n_slot)
            m.constraint(m.not_(m.contains(depot_set, sequence[0])))
            # battery_lambda = m.lambda_function(lambda i, prev:
            #                                    m.iif(i == 0,
            #                                          drone_charge[k] - m_time[sequence[i]] - m.at(t_matrix, 0, sequence[i]),
            #                                          m.iif(m.contains(depot_set, sequence[i]), drone_charge[k],
            #                                                prev - m.at(t_matrix, sequence[i-1], sequence[i]) -
            #                                                m.iif(m.contains(real_set, sequence[i]), m_time[sequence[i]], 0))))

            battery_lambda = m.lambda_function(lambda i, prev:
                                               m.iif(i == 0,
                                                     drone_charge[k] - m.iif(
                                                         m.contains(real_set, sequence[i]),
                                                         nm_times[sequence[i]] + t_matrix[0][sequence[i]], 0),
                                                     m.iif(m.contains(depot_set, sequence[i]), drone_charge[k],
                                                           prev - t_matrix[sequence[i-1]][sequence[i]] -
                                                           m.iif(m.contains(real_set, sequence[i]),
                                                                 nm_times[sequence[i]], 0))))

            route_battery[k] = m.array(m.range(0, c), battery_lambda, 0)
            # quantity_lambda = m.lambda_function(lambda i: m.iif(route_battery[k][i] >= 0, True, sequence[i] == 0))
            quantity_lambda = m.lambda_function(lambda i: route_battery[k][i] >= 0)
            m.constraint(m.and_(m.range(0, c-1), quantity_lambda))
            # m.constraint(m.and_([m.range(0, c), m.iif(route_battery[k][i] >= 0, sequence[i] >=1, True)]))


            end_time_lambda = m.lambda_function(lambda i, prev:
                                                m.iif(i != 0,
                                                      prev + m.at(t_matrix, sequence[i-1], sequence[i]) + nm_times[sequence[i]],
                                                      m.at(t_matrix, 0, sequence[i]) + nm_times[sequence[i]]))
            end_time[k] = m.array(m.range(0, c), end_time_lambda, 0)

            str_time_lambda = m.lambda_function(lambda i:
                                                m.iif(i == 0, 0, end_time[k][i - 1] + m.at(t_matrix, sequence[i - 1], sequence[i])))
            str_time[k] = m.array(m.range(0, c), str_time_lambda)

            late_lambda = m.lambda_function(lambda i: m.max(end_time[k][i] - due_dates[sequence[i]], 0))
            lateness[k] = m.max(m.range(0, c), late_lambda)

            earli_lambda = m.lambda_function(lambda i: m.max(due2_dates[sequence[i]] - str_time[k][i], 0))  # m.iif(m.or_(i == 0, i == n_node), -1000,
            earliness[k] = m.max(m.range(0, c), earli_lambda)

            # seq_rule = m.lambda_function(lambda i: m.iif(i == 0, 1, str_time[k][i] >= end_time[k][i-1]))
            # m.constraint(m.and_(m.range(0, c), seq_rule))

        # for i in range(n_node):
        #     if i in depot_data + real_data:
        #         m.constraint(nm_times[i] == data[2][i])
        #     # else:
        #     #     m.constraint(nm_times[i] >= m_time[i])





        vis_sequences_array = m.array(vis_sequences)
        end_times = m.array(end_time)
        str_times = m.array(str_time)
        #
        for i in real_data:
            # m.constraint(m.and_(m.range(0, n_drones), m.bool(m.contains(vis_sequences[k], i)))==False)
            # m.constraint(m.and_(m.range(0, n_drones), m.contains(vis_sequences[k], i)) == False)
            # m.constraint(m.contains(vis_sequences[0], i) + m.contains(vis_sequences[1], i) == 1)
            if successors_data[i]:
                ind_ = next((ind for ind, lst in enumerate(families) if i in lst), -1) - depot_data[-1] - 1
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
                m.constraint(jb_time - ic_time <= data[5][ind_])
                m.constraint(jb_time - ic_time >= 0)

        max_lateness = m.max(m.array(lateness))
        max_earliness = m.max(m.array(earliness))
        obj = m.minimize(max_lateness + max_earliness)
        # m.minimize(max_earliness)
        m.close()
        hex.param.time_limit = int(av_time)
        hex.param.verbosity = int(verbose)
        hex.solve()
        solution_status = hex.solution.status
        print('\n=== Solution Status ===')
        print('Solution status:', solution_status.name)

        if solution_status.name in ['INCONSISTENT', 'UNKNOWN']:
            print("\n=== Constraint Analysis ===")
            print("No valid solution found. Status:", solution_status.name)
            iis = hex.compute_inconsistency()
            print("Inconsistent constraints:", iis)

            print("\n=== Time Window Violations ===")
            for i in range(n_node):
                try:
                    if hasattr(str_times[i], 'value') and hasattr(end_times[i], 'value'):
                        print(
                            f"Node {i}: start={str_times[i].value}, end={end_times[i].value}, window=[{data[9][i]}, {data[1][i]}]")
                except:
                    pass

        try:
            for k in range(n_drones):
                gen_seq.append([])
                gen_st.append([])
                gen_ct.append([])
                b_res.append([])

                seq_values = vis_sequences[k].value
                for i1 in seq_values:
                    gen_seq[-1].append(i1)  # Convert back to 1-based indices for output

                for i2 in range(len(seq_values)):
                    end_val = end_times.value[k][i2]
                    gen_ct[-1].append(round(float(end_val), 2))
                    str_val = str_times.value[k][i2]
                    gen_st[-1].append(round(float(str_val), 2))

                # Get battery values safely
                try:
                    r_bat = route_battery[k]
                    for i4 in range(len(seq_values)):
                        b_res[-1].append(round(r_bat.value[i4], 2))
                except Exception as e:
                    print(f"Error getting battery values for drone {k}: {e}")
                    continue

            print('~~~~~~~~~~ HXL has been finalized ~~~~~~~~~~ -->', solution_status.name)

        except Exception as e:
            print(f"Error processing results: {e}")

        # print("\n=== Problem Parameters ===")
        # print(f"Number of drones: {n_drones}")
        # print(f"Number of slots: {n_slot}")
        # print(f"Number of nodes: {n_node}")
        # print(f"Depot nodes (0-based): {depot_data}")
        # print(f"Real nodes (0-based): {real_data}")
        # print(f"Idle nodes (0-based): {idle_data}")
        # print("\n=== Time Matrix Analysis ===")
        # max_travel_time = max(max(row) for row in t_matrix_data)
        # print(f"Maximum travel time: {max_travel_time}")
        # print(f"Service times: {data[10]}")
        # print("\n=== Time Windows ===")
        # for i in real_data:
        #     print(f"Node {i}: earliest={data[9][i]}, latest={data[1][i]}, service_time={data[2][i]}")
        #
        # for i in range(int(n_drones / 2)):
        #     # print(map_sequence(gen_seq[i], depot_data, real_data, idle_data), '\n')
        #     print(gen_seq[i])
        #     print(gen_st[i])
        #     print(gen_ct[i])
        #     print(b_res[i])
        #     print('~~~~~~~~~~')
        gen_seq1 = map_sequence(gen_seq, depot_data, real_data, idle_data)
        solution_time = hex.statistics.running_time
        objective_value = m.objectives[0].value
        pickle_out = open('hexa.pickle', "wb")
        pickle.dump([gen_seq1, gen_st, gen_ct, b_res, objective_value, solution_time], pickle_out)
        pickle_out.close()
        return None
