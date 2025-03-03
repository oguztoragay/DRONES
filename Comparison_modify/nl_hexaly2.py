# Updated on 03/02/2025 Another try to make the Hexaly work with the recent updates in the mathematical model
import hexaly.optimizer
from operator import indexOf
import pickle
import math

def map_sequence(sequence, depos, real_nodes, idles):
    depot_set = set(depos)
    idle_set = set(idles)
    idle_replacement = max(real_nodes) + 1 -len(depos)+1
    mapped_sequence = []
    for node in sequence:
        if node in depot_set:
            mapped_sequence.append(1)  # Replace depot nodes with 1
        elif node in idle_set:
            mapped_sequence.append(idle_replacement)  # Replace idle nodes with max_real_node + 1
        else:
            mapped_sequence.append(node-len(depos)+1)  # Keep real nodes as is
    return mapped_sequence

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
        drone_charge = [x for x in data[7]]  # Scale battery capacity
        
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

        t_matrix = m.array([x for x in t_matrix_data])
        earliest = m.array([x for x in data[9]])
        latest = m.array([x for x in data[8]])
        m_time = m.array([x for x in data[10]])
        vis_sequences = [m.list(n_node) for k in range(n_drones)]
        m.constraint(m.partition(vis_sequences))
        
        # Ensure each real node is assigned to a drone in the first half
        for i in real_nodes:
            m.constraint(m.or_([m.contains(vis_sequences[k], i) for k in range(int(n_drones/2)-1)]))
        
        # Modify the constraint for real nodes to be less restrictive
        for d in range(int(n_drones/2), n_drones):
            for i in real_nodes:
                m.constraint(m.not_(m.contains(vis_sequences[d], i-1)))
        
        # Create variables for time windows - using separate start and end times
        time_horizon = 1440  # Scale the horizon for real number precision
        
        # Create start and end time variables
        start_time = [[m.float(0, time_horizon) for _ in range(n_slot)] for _ in range(n_drones)]
        end_time = [[m.float(0, time_horizon) for _ in range(n_slot)] for _ in range(n_drones)]
        
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
                                               m.iif(i == 0,
                                                     # First node in sequence
                                                     m.iif(m.contains(depos_set, sequence[i]),
                                                           # If first node is depot, start with full charge
                                                           drone_charge[k],
                                                           # If first node is not depot, consume from depot 0
                                                           drone_charge[k] - m.iif(m.contains(real_nodes_set, sequence[i]),
                                                                                 m_time[sequence[i]], 0) - m.at(t_matrix, 0, sequence[i])),
                                                     # Not first node
                                                     m.iif(m.contains(depos_set, sequence[i]),
                                                           # If current node is depot, recharge
                                                           drone_charge[k],
                                                           # If not depot, consume battery based on type
                                                           prev - m.iif(m.contains(real_nodes_set, sequence[i]),
                                                                       m_time[sequence[i]], 0) - m.at(t_matrix, sequence[i-1], sequence[i]))))

            route_battery[k] = m.array(m.range(c), battery_lambda)
            
            # Battery constraints for each position
            for pos in range(n_slot):
                # Only apply constraints to positions that are used (pos < c)
                m.constraint(m.iif(m.not_(m.contains(depos_set, sequence[pos])),
                                   route_battery[k][pos] >= 0,
                                   route_battery[k][pos] == drone_charge[k]))

                # If this isn't the last position and both positions are used,
                # ensure battery decreases properly between positions
                if pos < n_slot - 1:
                    m.constraint(m.iif(m.contains(depos_set, sequence[pos + 1]), True,
                                       route_battery[k][pos + 1] == route_battery[k][pos] -
                             m.iif(m.contains(real_nodes_set, sequence[pos + 1]),
                                   m_time[sequence[pos + 1]], 0) - 
                             m.at(t_matrix, sequence[pos], sequence[pos + 1])))

            # Time window constraints using start and end times
            for i in range(n_slot):
                if i == 0:
                    # First node starts at time 0
                    m.constraint(start_time[k][i] == 0)
                else:
                    # Subsequent nodes start after previous node ends plus travel time
                    m.constraint(start_time[k][i] == end_time[k][i-1] + m.at(t_matrix, sequence[i-1], sequence[i]))

                # End time constraints
                m.constraint(end_time[k][i] == start_time[k][i] +
                             m.iif(m.contains(real_nodes_set, sequence[i]), m_time[sequence[i]], 0))

                # Time window bounds for real nodes
                # m.constraint(m.iif(m.contains(real_nodes_set, sequence[i])),
                #     m.and_(start_time[k][i] >= m.at(earliest, sequence[i]),
                #            end_time[k][i] <= m.at(latest, sequence[i])),
                #     True))

            # Calculate lateness and earliness using position-based constraints
            late_values = []
            earl_values = []
            for pos in range(n_slot):
                # Only calculate for positions that are used and contain real nodes
                late_expr = m.iif(
                    m.and_(pos < c, m.contains(real_nodes_set, sequence[pos])),
                    m.max(end_time[k][pos] - m.at(latest, sequence[pos]), 0),
                    0
                )
                earl_expr = m.iif(
                    m.and_(pos < c, m.contains(real_nodes_set, sequence[pos])),
                    m.max(m.at(earliest, sequence[pos]) - start_time[k][pos], 0),
                    0
                )
                late_values.append(late_expr)
                earl_values.append(earl_expr)
            
            lateness[k] = m.max(m.array(late_values))
            earlness[k] = m.max(m.array(earl_values))
            cost_[k] = m.sum(lateness[k], earlness[k])

            print(f'Drone {k} constraints added')

        # Add precedence constraints across all drones
        for i in range(n_node):
            if i in successors_data and successors_data[i]:
                for j in successors_data[i]:
                    # For each pair of drones
                    for k1 in range(n_drones):
                        sequence1 = vis_sequences[k1]
                        for k2 in range(n_drones):
                            sequence2 = vis_sequences[k2]
                            
                            # If i is in sequence1 and j is in sequence2
                            m.constraint(m.iif(
                                m.and_(m.contains(sequence1, i), m.contains(sequence2, j)),
                                m.and_(
                                    # If same drone, ensure j comes after i
                                    m.iif(k1 == k2,
                                        m.and_([m.iif(
                                            m.and_(sequence1[pos1] == i, sequence1[pos2] == j),
                                            pos2 > pos1,
                                            True
                                        ) for pos1 in range(n_slot) for pos2 in range(n_slot)]),
                                        True
                                    ),
                                    # Ensure time precedence
                                    m.and_([m.iif(
                                        m.and_(sequence1[pos1] == i, sequence2[pos2] == j),
                                        end_time[k1][pos1] <= start_time[k2][pos2],
                                        True
                                    ) for pos1 in range(n_slot) for pos2 in range(n_slot)])
                                ),
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
                    # Map the sequence after creating it
                    # gen_seq[-1] = map_sequence(gen_seq[-1], data[4], data[5], data[6])
                except Exception as e:
                    print(f"Error getting sequence values for drone {k}: {e}")
                    continue
                
                # Get completion times safely
                try:
                    for i2 in range(len(seq_values)):
                        end_val = end_time[k][i2].value
                        gen_ct[-1].append(round(float(end_val), 2))
                except Exception as e:
                    print(f"Error getting completion times for drone {k}: {e}")
                    continue
                
                # Get start times safely
                try:
                    for i3 in range(len(seq_values)):
                        start_val = start_time[k][i3].value
                        gen_st[-1].append(round(float(start_val), 2))
                except Exception as e:
                    print(f"Error getting start times for drone {k}: {e}")
                    continue
                
                # Get battery values safely - scale back from
                try:
                    battery_values = route_battery[k].value
                    for i4 in battery_values:
                        b_res[-1].append(round(float(i4), 2))
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
        for i in range(int(n_drones)):
            print(map_sequence(gen_seq[i], data[4], data[5], data[6]), '\n')
            print(gen_st[i])
            print(gen_ct[i])
            print(b_res[i])
            print('~~~~~~~~~~')

        return gen_seq, gen_st, gen_ct, b_res
