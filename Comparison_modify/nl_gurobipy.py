import gurobipy as gp
from gurobipy import GRB
import pickle

from pyomo.core import Model
# env = gp.Env(empty=True)
# env.setParam("OutputFlag", 0)  # Disables output
# env.start()

def nl_gurobipy(data, verbose):
    t_matrix, due_dates, m_time, n_slot, drone_charge, i_times, membership, families, f, due2, len_dl, fs_slot, __ = data
    demand_set = range(1, len(due_dates) + 1)  # Locations (j)
    drones_set = range(1, len(drone_charge) + 1)  # Drones (i)
    slot_set = range(1, n_slot + 1)  # Slots (r)
    idle_set = set(range(len(demand_set) - len_dl + 1, len(demand_set) + 1))
    indexed_families = list(enumerate(f))
    full_charge = drone_charge[0]

    # Create Gurobi model
    model = gp.Model("Multiple Drones QP Model")
    model.setParam('OutputFlag', verbose)
    # Define variables
    x = model.addVars(demand_set, slot_set, drones_set, vtype=GRB.BINARY, name="x")
    s = model.addVars(slot_set, drones_set, vtype=GRB.CONTINUOUS, lb=0, ub=1440, name="s")
    c = model.addVars(slot_set, drones_set, vtype=GRB.CONTINUOUS, lb=0, ub=1440, name="c")
    t = model.addVars(slot_set, drones_set, vtype=GRB.CONTINUOUS, lb=0, ub=full_charge, name="t")
    d = model.addVars(demand_set, slot_set, drones_set, vtype=GRB.CONTINUOUS, lb=0, ub=1440, name="d")
    lmax = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=1440, name="lmax")
    lmax2 = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=1440, name="lmax2")

    # Objective function
    model.setObjective(lmax + lmax2, GRB.MINIMIZE)

    # Constraints
    for i in drones_set:
        for r in slot_set:
            model.addConstr(lmax >= (c[r, i] - gp.quicksum(due_dates[j - 1] * x[j, r, i] for j in demand_set)), name=f"c1_{r}_{i}")
            model.addConstr(lmax2 >= gp.quicksum(due2[j - 1] * x[j, r, i] for j in demand_set) - s[r, i], name=f"c2_{r}_{i}")

    for j in set(demand_set) - {1} - idle_set:
        model.addConstr(gp.quicksum(x[j, r, i] for r in slot_set for i in drones_set) == 1, name=f"c3_{j}")

    for i in drones_set:
        for r in slot_set:
            model.addConstr(gp.quicksum(x[j, r, i] for j in demand_set) == 1, name=f"c4_{r}_{i}")

    for i in drones_set:
        model.addConstr(c[1, i] == gp.quicksum((t_matrix[0][j - 1] + d[j, 1, i]) * x[j, 1, i] for j in demand_set), name=f"c5_{i}")

    for i in drones_set:
        for r in set(slot_set) - {1}:
            model.addConstr(c[r, i] == c[r-1, i] + gp.quicksum(t_matrix[k-1][j-1] * x[k, r-1, i] * x[j, r, i] for j in demand_set for k in demand_set) +
                gp.quicksum(d[j, r, i] * x[j, r, i] for j in demand_set), name=f"c6_{r}_{i}")

    for i in drones_set:
        s[1, i].lb = 0  # Fix s[1,i] = 0
        s[1, i].ub = 0

    for i in drones_set:
        for r in set(slot_set) - {1}:
            model.addConstr(s[r, i] == c[r, i] - gp.quicksum(d[j, r, i] * x[j, r, i] for j in demand_set), name=f"c8_{r}_{i}")

    for i in drones_set:
        model.addConstr(t[1, i] == full_charge - gp.quicksum((t_matrix[0][j-1] + d[j, 1, i]) * x[j, 1, i] for j in set(demand_set) - idle_set), name=f"c9_{i}")

    for i in drones_set:
        for r in set(slot_set) - {1}:
            model.addConstr(
                t[r, i] == (full_charge * x[1, r, i]) +
                ((t[r-1, i] - c[r, i] + c[r-1, i]) * gp.quicksum(x[j, r, i] for j in set(demand_set) - {1} - idle_set)) +
                ((t[r-1, i] - s[r, i] + c[r-1, i]) * gp.quicksum(x[id_, r, i] for id_ in idle_set)), name=f"c10_{r}_{i}")

    for ind_, f in indexed_families:
        for j in f:
            model.addConstr(
                gp.quicksum(s[r, i] * x[j+1, r, i] for r in slot_set for i in drones_set) - gp.quicksum(c[r, i] * x[j, r, i] for r in slot_set for i in drones_set) <= i_times[ind_], name=f"c11a_{ind_}_{j}")
            model.addConstr(
                gp.quicksum(s[r, i] * x[j+1, r, i] for r in slot_set for i in drones_set) - gp.quicksum(c[r, i] * x[j, r, i] for r in slot_set for i in drones_set) >= 0, name=f"c11b_{ind_}_{j}")

    for i in drones_set:
        for r in slot_set:
            for j in set(demand_set) - idle_set:
                d[j, r, i].lb = m_time[j - 1]  # Fix d[j, r, i]
                d[j, r, i].ub = m_time[j - 1]


    # # Set Gurobi parameters
    model.Params.Threads = 24
    # model.Params.FeasibilityTol = 1e-6
    # model.Params.OptimalityTol = 1e-5
    # model.Params.MIPFocus = 2
    # model.Params.Cuts = 3
    # model.Params.Heuristics = 1
    # model.Params.RINS = 5
    # # model.Params.PreQLinearize = 0
    # model.Params.BarCorrectors = 3
    # # model.Params.PreMIQCPForm = 1
    # # model.Params.Presolve = 2
    model.Params.TimeLimit = 3600
    # model.Params.FuncNonlinear = 1

    model.Params.MIPFocus = 2
    # model.Params.NoRelHeurWork = 120
    model.Params.NoRelHeurTime = 120
    model.Params.GomoryPasses = 0

    # Solve the model
    model.optimize()
    # model.tune()

    # # Print best parameter settings found
    # if model.TuneResultCount > 0:
    #     for i in range(model.TuneResultCount):
    #         model.getTuneResult(i)  # Apply the i-th best tuning result
    #         print(f"Tuning Result {i + 1}:")
    #         for param, value in model.Params.__dict__.items():
    #             print(f"{param}: {value}")  # Print parameter values


    for v in model.getVars():
        if v.x == 1.0000:
            print(f"{v.varName}: {v.x:.4f}")
    print(f"Objective Value: {model.objVal:.4f}")

    # pickle_out = open('nlp1.pickle', "wb")
    # solution_data = {
    #     "variables": {var.varName: var.X for var in model.getVars()},
    #     "objective_value": model.ObjVal if model.status == GRB.OPTIMAL else None,
    #     "num_variables": model.NumVars,
    #     "num_constraints": model.NumConstrs}
    #
    # pickle.dump(solution_data, pickle_out)
    # pickle_out.close()
    # print('~~~~~~~~~~ NLP has been finalized ~~~~~~~~~~ -->', model.Status)
    return None
