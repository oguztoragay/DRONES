# Updated on 03/02/2025 Another try to make the Hexaly work with the recent updates in the mathematical model
import hexaly.optimizer
from operator import indexOf
import pickle
import math

def hexa(data, gen_seq, gen_st, gen_ct, av_time, b_res, verbose):
    with hexaly.optimizer.HexalyOptimizer() as ls:
        m = ls.model
        n_drones = data[0]
        n_slot = data[1]
        n_node = len(data[2])
        t_matrix_data = list(data[3])
        
        # Convert 1-based indices to 0-based
        depos = [x-1 for x in data[4]]  # Convert depot indices to 0-based
        real_nodes = [x-1 for x in data[5]]  # Convert real node indices to 0-based
        idles = [x-1 for x in data[6]]  # Convert idle node indices to 0-based
        drone_charge = data[7]
        
        print("Problem parameters:")
        print(f"Number of drones: {n_drones}")
        print(f"Number of slots: {n_slot}")
        print(f"Number of nodes: {n_node}")
        print(f"Depot nodes (0-based): {depos}")
        print(f"Real nodes (0-based): {real_nodes}")
        print(f"Idle nodes (0-based): {idles}")

        # Create sets with proper size initialization
        real_nodes_set = m.set(n_node)
        idles_set = m.set(n_node)
        depos_set = m.set(n_node)
        
        # Add elements to sets (now using 0-based indices)
        for node in real_nodes:
            m.constraint(m.contains(real_nodes_set, node))
        for idle in idles:
            m.constraint(m.contains(idles_set, idle))
        for depot in depos:
            m.constraint(m.contains(depos_set, depot))

        t_matrix = m.array(t_matrix_data)
        earliest = m.array(data[9])
        latest = m.array(data[8])
        m_time = m.array(data[10])
        vis_sequences = [m.list(n_node) for k in range(n_drones)]
        m.constraint(m.partition(vis_sequences))
        
        # Ensure each real node is assigned to a drone in the first half
        for i in real_nodes:
            m.constraint(m.or_([m.contains(vis_sequences[k], i) for k in range(int(n_drones/2))]))
        
        # Modify the constraint for real nodes to be less restrictive
        for d in range(int(n_drones/2)+1, n_drones):
            for i in real_nodes:
                m.constraint(m.not_(m.contains(vis_sequences[d], i)))
        
        # Create variables for time windows - using interval variables
        time_horizon = 2000  # Assuming 2000 is a reasonable upper bound
        
        # Create interval variables for time windows
        slot_time = [[m.interval(0, time_horizon) for _ in range(n_slot)] for _ in range(n_drones)]
        lateness = [None] * n_drones
        earlness = [None] * n_drones
        route_battery = [None] * n_drones
        cost_ = [None] * n_drones

        successors_data = {i: [] for i in range(n_node)}  # Use dictionary instead of list
        families = list(map(lambda lst: [x-1 for x in lst], data[11]))  # Convert to 0-based indices
        print("Processing families:", families)
        
        for fam in families:
            for i, node in enumerate(fam[:-1]):  # Exclude last node as it has no successors
                successors_data[node] = fam[i+1:]  # All subsequent nodes in family are successors
        print("Successors data:", successors_data)

        for k in range(n_drones):
            sequence = vis_sequences[k]
            c = m.count(sequence)
            m.constraint(m.count(sequence) <= n_slot)
            
            battery_lambda = m.lambda_function(lambda i, prev:
                m.iif(i == 0,  # First node case
                    m.iif(m.contains(real_nodes_set, sequence[i]),
                        drone_charge[k] - m_time[sequence[i]] - m.at(t_matrix, 0, sequence[i]),
                        drone_charge[k]),  # For first idle node, only initial charge
                    # For subsequent nodes (i > 0)
                    m.iif(m.contains(real_nodes_set, sequence[i]),
                        prev - m_time[sequence[i]] - m.at(t_matrix, sequence[i-1], sequence[i]),
                        m.iif(m.contains(idles_set, sequence[i]),
                            prev - m.at(t_matrix, sequence[i-1], sequence[i]),
                            drone_charge[k]))))
            route_battery[k] = m.array(m.range(c), battery_lambda)

            quantity_lambda = m.lambda_function(lambda i: m.or_(route_battery[k][i] >= 0, m.contains(depos_set, sequence[i])))
            m.constraint(m.and_(m.range(c), quantity_lambda))

            # Time window constraints using intervals
            for i in range(n_slot):
                if i == 0:
                    # First node in sequence
                    m.constraint(m.iif(
                        i < c,  # Only if position is within sequence
                        m.and_(
                            m.start(slot_time[k][i]) >= 0,
                            m.end(slot_time[k][i]) >= m.iif(
                                m.contains(real_nodes_set, sequence[i]),
                                m.start(slot_time[k][i]) + m_time[sequence[i]],
                                m.start(slot_time[k][i])
                            )
                        ),
                        m.and_(
                            m.start(slot_time[k][i]) == 0,
                            m.end(slot_time[k][i]) == 0
                        )
                    ))
                else:
                    # Subsequent nodes
                    m.constraint(m.iif(
                        i < c,  # Only if position is within sequence
                        m.and_(
                            m.start(slot_time[k][i]) >= m.end(slot_time[k][i-1]),
                            m.end(slot_time[k][i]) >= m.iif(
                                m.contains(real_nodes_set, sequence[i]),
                                m.start(slot_time[k][i]) + m_time[sequence[i]],
                                m.start(slot_time[k][i])
                            )
                        ),
                        m.and_(
                            m.start(slot_time[k][i]) == 0,
                            m.end(slot_time[k][i]) == 0
                        )
                    ))

                # Travel time constraints
                m.constraint(m.iif(
                    m.and_(i < c, i > 0),
                    m.start(slot_time[k][i]) >= m.end(slot_time[k][i-1]) + m.at(t_matrix, sequence[i-1], sequence[i]),
                    True
                ))

                # Time window bounds for real nodes
                m.constraint(m.iif(
                    m.and_(
                        i < c,
                        m.contains(real_nodes_set, sequence[i])
                    ),
                    m.and_(
                        m.start(slot_time[k][i]) >= m.at(earliest, sequence[i]),
                        m.end(slot_time[k][i]) <= m.at(latest, sequence[i])
                    ),
                    True
                ))

            slot_time_array = m.array(slot_time[k])
            
            late_lambda = m.lambda_function(lambda i: m.iif(m.contains(real_nodes_set, sequence[i]), m.end(m.at(slot_time_array, i)) - m.at(latest, sequence[i]), 0))
            earl_lambda = m.lambda_function(lambda i: m.iif(m.contains(real_nodes_set, sequence[i]), m.at(earliest, sequence[i]) - m.start(m.at(slot_time_array, i)), 0))
            
            lateness[k] = m.max(m.range(c), late_lambda)
            earlness[k] = m.max(m.range(c), earl_lambda)
            cost_[k] = m.sum(lateness[k], earlness[k])

            print(f'Drone {k} constraints added')

        # Add precedence constraints across all drones
        for i in range(n_node):
            if i in successors_data and successors_data[i]:
                for j in successors_data[i]:
                    # For each drone k
                    for k in range(n_drones):
                        sequence = vis_sequences[k]
                        c = m.count(sequence)
                        
                        # Find positions of i and j in the sequence
                        for pos_i in range(n_slot):
                            for pos_j in range(n_slot):
                                # If both i and j are in this sequence, ensure j comes after i
                                m.constraint(m.iif(
                                    m.and_(
                                        pos_i < c,
                                        pos_j < c,
                                        sequence[pos_i] == i,
                                        sequence[pos_j] == j
                                    ),
                                    pos_j > pos_i,
                                    True
                                ))
                    
                    # Cross-drone precedence using time windows
                    for k1 in range(n_drones):
                        seq1 = vis_sequences[k1]
                        c1 = m.count(seq1)
                        for k2 in range(n_drones):
                            seq2 = vis_sequences[k2]
                            c2 = m.count(seq2)
                            
                            for pos1 in range(n_slot):
                                for pos2 in range(n_slot):
                                    m.constraint(m.iif(
                                        m.and_(
                                            pos1 < c1,
                                            pos2 < c2,
                                            seq1[pos1] == i,
                                            seq2[pos2] == j
                                        ),
                                        m.end(slot_time[k1][pos1]) <= m.start(slot_time[k2][pos2]),
                                        True
                                    ))

        max_la = m.max(m.array(cost_))
        m.minimize(max_la)
        m.close()
        ls.param.time_limit = int(av_time)
        ls.param.verbosity = int(verbose)
        ls.solve()
        
        solution_status = ls.solution.status
        print('Solution status:', solution_status.name)
        
        # Check if solution is valid (not INCONSISTENT or UNKNOWN)
        if solution_status.name in ['INCONSISTENT', 'UNKNOWN']:
            print("No valid solution found. Status:", solution_status.name)
            iis = ls.compute_inconsistency()
            print("Inconsistent constraints:", iis)
            return gen_seq, gen_st, gen_ct, b_res
        
        try:
            for k in range(n_drones):
                gen_seq.append([])
                gen_st.append([])
                gen_ct.append([])
                b_res.append([])
                
                # Get sequence values safely
                try:
                    seq_values = vis_sequences[k].value
                    for i1 in seq_values:
                        gen_seq[-1].append(i1+1)  # Convert back to 1-based indices for output
                except Exception as e:
                    print(f"Error getting sequence values for drone {k}: {e}")
                    continue
                
                # Get completion times safely
                try:
                    for i2 in range(len(seq_values)):
                        interval = slot_time[k][i2].value
                        end_time = interval.end() if hasattr(interval, 'end') else interval
                        gen_ct[-1].append(round(float(end_time), 4))
                except Exception as e:
                    print(f"Error getting completion times for drone {k}: {e}")
                    continue
                
                # Get start times safely
                try:
                    for i3 in range(len(seq_values)):
                        interval = slot_time[k][i3].value
                        start_time = interval.start() if hasattr(interval, 'start') else interval
                        gen_st[-1].append(round(float(start_time), 4))
                except Exception as e:
                    print(f"Error getting start times for drone {k}: {e}")
                    continue
                
                # Get battery values safely
                try:
                    battery_values = route_battery[k].value
                    for i4 in battery_values:
                        b_res[-1].append(round(i4, 4))
                except Exception as e:
                    print(f"Error getting battery values for drone {k}: {e}")
                    continue

            # Only save to pickle if we have valid results
            if any(gen_seq):
                try:
                    max_la_value = max_la.value if max_la.value is not None else float('inf')
                    pickle_out = open('hxl.pickle', "wb")
                    pickle.dump([gen_seq, gen_st, gen_ct, b_res, max_la_value], pickle_out)
                    pickle_out.close()
                except Exception as e:
                    print(f"Error saving results to pickle: {e}")
            
            print('~~~~~~~~~~ HXL has been finalized ~~~~~~~~~~ -->', solution_status.name)
            
        except Exception as e:
            print(f"Error processing results: {e}")
        print('~~~~~~~~~~')
        print(gen_seq)
        print('~~~~~~~~~~')
        print(gen_st)
        print('~~~~~~~~~~')
        print(gen_ct)
        return gen_seq, gen_st, gen_ct, b_res
