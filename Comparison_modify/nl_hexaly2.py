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
        print(f"Real nodes (0-based): {real_nodes}")
        print(f"Idle nodes (0-based): {idles}")
        print(f"Depot nodes (0-based): {depos}")
        
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
        
        # Create variables for time windows - using interval variables
        time_horizon = 2000  # Assuming 2000 is a reasonable upper bound
        
        # Create interval variables for time windows
        end_time = [[m.interval(0, time_horizon) for _ in range(n_slot)] for _ in range(n_drones)]
        str_time = [[m.interval(0, time_horizon) for _ in range(n_slot)] for _ in range(n_drones)]
        lateness = [None] * n_drones
        earlness = [None] * n_drones
        route_battery = [None] * n_drones
        cost_ = [None] * n_drones

        # Modify the constraint for real nodes to be less restrictive
        for d in range(int(n_drones/2), n_drones):
            for i in real_nodes:
                m.constraint(m.not_(m.contains(vis_sequences[d], i)))
        
        # Ensure all nodes are visited appropriately across all sequences
        all_nodes = set(range(n_node))
        for node in all_nodes:
            # Create visit counting constraints for each node
            visit_counts = []
            for k in range(n_drones):
                sequence = vis_sequences[k]
                count_lambda = m.lambda_function(lambda i: m.iif(sequence[i] == node, 1, 0))
                sequence_count = m.sum(m.array(m.range(m.count(sequence)), count_lambda))
                visit_counts.append(sequence_count)
            
            total_visits = m.sum(m.array(visit_counts))
            
            if node in real_nodes or node in idles:
                m.constraint(total_visits == 1)  # Each non-depot node must be visited exactly once
            else:
                m.constraint(total_visits >= 0)  # Depot nodes can be visited multiple times or not at all

        # Process family precedence constraints first
        successors_data = {i: [] for i in range(n_node)}  # Use dictionary instead of list
        families = list(map(lambda lst: [x-1 for x in lst], data[11]))  # Convert to 0-based indices
        print("Processing families:", families)
        
        for fam in families:
            for i, node in enumerate(fam[:-1]):  # Exclude last node as it has no successors
                successors_data[node] = fam[i+1:]  # All subsequent nodes in family are successors
        
        print("Successors data:", successors_data)

        # Create node position lookup for each drone
        node_positions = []  # Will store (drone_index, position) for each node
        for k in range(n_drones):
            sequence = vis_sequences[k]
            c = m.count(sequence)
            m.constraint(m.count(sequence) <= n_slot)
            
            # Allow starting from any node for more flexibility
            if k < int(n_drones/2):  # Only apply depot constraints to first half of drones
                for i in depos:
                    m.constraint(sequence[0] != i)

            battery_lambda = m.lambda_function(lambda i, prev:
                                               m.iif(m.contains(real_nodes_set, sequence[i]),
                                                     prev - m_time[sequence[i]] - m.at(t_matrix, sequence[i-1], sequence[i]),
                                                     m.iif(m.contains(idles_set, sequence[i]),
                                                           prev - m.at(t_matrix, sequence[i-1], sequence[i]), drone_charge[k])))
            route_battery[k] = m.array(m.range(c), battery_lambda)

            quantity_lambda = m.lambda_function(lambda i: m.or_(route_battery[k][i] >= 0, m.contains(depos_set, sequence[i])))
            m.constraint(m.and_(m.range(c), quantity_lambda))

            # Time window constraints using intervals
            for i in range(n_slot):
                if i == 0:
                    # First node in sequence
                    m.constraint(str_time[k][i].start == 0)
                    m.constraint(end_time[k][i].start == m.iif(
                        m.contains(depos_set, sequence[i]),
                        m.at(t_matrix, 0, sequence[i]) + m_time[sequence[i]],
                        m.at(t_matrix, 0, sequence[i]) + m_time[sequence[i]]
                    ))
                else:
                    # Subsequent nodes
                    m.constraint(str_time[k][i].start == end_time[k][i-1].end + m.at(t_matrix, sequence[i-1], sequence[i]))
                    m.constraint(end_time[k][i].start == str_time[k][i].start + m_time[sequence[i]])

                # Ensure end time is after start time
                m.constraint(end_time[k][i].start >= str_time[k][i].start)
                
                # Time window constraints
                if not m.contains(depos_set, sequence[i]):
                    m.constraint(str_time[k][i].start >= earliest[sequence[i]])
                    m.constraint(end_time[k][i].end <= latest[sequence[i]])

            # Calculate lateness and earliness using interval bounds
            late_lambda = m.lambda_function(lambda i: 
                m.iif(m.or_(i == 0, i == n_node-1), -5000,
                      m.iif(m.contains(depos_set, sequence[i]), -5000,
                            end_time[k][i].end - latest[sequence[i]])))
            
            earl_lambda = m.lambda_function(lambda i: 
                m.iif(m.or_(i == 0, i == n_node-1), -5000,
                      m.iif(m.contains(depos_set, sequence[i]), -5000,
                            earliest[sequence[i]] - str_time[k][i].start)))
            
            lateness[k] = m.max(m.range(c), late_lambda)
            earlness[k] = m.max(m.range(c), earl_lambda)
            cost_[k] = m.sum(lateness[k], earlness[k])

            print(f'Drone {k} constraints added')

        # Add precedence constraints across all drones
        for i in range(n_node):
            if i in successors_data and successors_data[i]:  # Check if node has any successors
                # Find which drone and position contains node i
                i_drone_pos = []
                for k in range(n_drones):
                    i_pos = m.find(vis_sequences[k], i)
                    m.constraint(m.implies(i_pos != -1, m.and_([m.find(vis_sequences[d], i) == -1 for d in range(n_drones) if d != k])))
                    i_drone_pos.append((k, i_pos))
                
                for j in successors_data[i]:
                    # Find which drone and position contains node j
                    j_drone_pos = []
                    for k in range(n_drones):
                        j_pos = m.find(vis_sequences[k], j)
                        m.constraint(m.implies(j_pos != -1, m.and_([m.find(vis_sequences[d], j) == -1 for d in range(n_drones) if d != k])))
                        j_drone_pos.append((k, j_pos))
                    
                    # Add precedence constraints between i and j across all possible drone combinations
                    for i_k, i_pos in i_drone_pos:
                        for j_k, j_pos in j_drone_pos:
                            m.constraint(m.implies(
                                m.and_(i_pos != -1, j_pos != -1),
                                m.and_(
                                    str_time[j_k][j_pos].start - end_time[i_k][i_pos].end <= data[4],
                                    str_time[j_k][j_pos].start - end_time[i_k][i_pos].end >= 0
                                )
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
                        gen_ct[-1].append(round(end_time[k][i2].end.value, 4))
                except Exception as e:
                    print(f"Error getting completion times for drone {k}: {e}")
                    continue
                
                # Get start times safely
                try:
                    for i3 in range(len(seq_values)):
                        gen_st[-1].append(round(str_time[k][i3].start.value, 4))
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
        print(gen_seq, gen_st, gen_ct, b_res)
        return gen_seq, gen_st, gen_ct, b_res
