import warnings
warnings.filterwarnings("ignore", category=UserWarning)
import numpy as np
import gurobipy as gb
from gurobipy import GRB
from shapely.geometry import LineString
import pandas as pd
import time

def GBNLPpyo():
    # --------------------------------------------------------------------------------------------------------------------------------------------------
    m = gb.Model('NewModel')
    m.setParam('NonConvex', 2)
    m.setParam('TimeLimit', 18000)  # Time limit increase from 10800 to 18000 on 11.23.2020
    m.setParam('Threads', 16)  # Input your machine's # of CPUs
    m.setParam('LogToConsole', 1)
    m.setParam('DisplayInterval', 100)
    m.setParam('InfUnbdInfo', 1)
    m.setParam('FeasibilityTol', 1e-5)
    m.setParam('Heuristics', 1)
    m.setParam('RINS', 10)
    m.setParam('Cuts', 3)
    m.setParam('MIPFocus', 1)
    m.setParam('Presolve', 2)
    m.setParam('PreQLinearize', 2)
    m.setParam('FuncNonlinear', 1)
    # --------------------------------------------------------------------------------------------------------------------------------------------------
    LN = [1, 2, 3]
    # --------------------------------------------------------------------------------------------------------------------------------------------------
    # d = m.addVars(dofs, lb=-dmax, ub=dmax, vtype=GRB.CONTINUOUS, name='Disp')
    # for i in nfree:
    #     d[i].ub = 0
    #     d[i].lb = 0
    s = m.addVars(LN, vtype=GRB.BINARY, name='Ss')
    x = m.addVars(LN, lb=1, ub=23, vtype=GRB.CONTINUOUS, name='Xs')
    y = m.addVars(LN, vtype=GRB.BINARY, name='Ys')

    obj = gb.quicksum(-1000 * x[i] * (1-y[i]) + s[i] + pow(x[i], 2) for i in LN)
    m.setObjective(obj, GRB.MINIMIZE)
    # -------------------------------------------------------------CONSTRAINTS---------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------
    # for i in LN:
    #     m.addQConstr(x[i] <= s[i] * y[i])
    #     m.addQConstr(x[i] >= s[i] * y[i])
    # -----------------------------------------------------------------------------------------
    for i in LN:
        m.addQConstr(sum(x[i] for i in [2, 3]) >= x[i] * y[i]**2, "FuncPieces=1000")
    # ----------------------------------------------------------
    m.update()
    QPstart = time.time()
    m.optimize()
    Obj1 = m.objVal
    for i in LN:
        print(x[i].x)
        print(y[i].x)
    print(Obj1)

GBNLPpyo()