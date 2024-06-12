import greedy
import os
from nl_hexaly3 import hexa
# from nl_pyomo_model import nl_pyo
from nl_pyomo2 import nl_pyo
from pyomo_model import lp_pyo
from random_instance import generate
from random_instance import mprint
import ast

def run(city, time):
    a, b, c, d, e = city
    ins = generate(ndrones=a, condition=b, slot=c, charge=d, itimes=e)
    incumbent = ins2incumbent(ins, a, b, c, d, e, time)
    warm_start = []
    ws_x = []
    ws_y = []
    ws_z = []
    # warm_start, ws_x, ws_y, ws_z = incumbent2pyomo(incumbent, c, ins[7][-1][0])

    # lp_pyo(ins, warm_start, ws_x, ws_y, ws_z)
    nl_pyo(ins, warm_start, ws_x, ws_y, ws_z)

def ins2incumbent(ins, a, b, c, d, e, time):
    hexa_data = [a, c, ins[0], ins[4], e, ins[2], ins[1], ins[7]]
    gen_seq = []
    gen_st = []
    gen_ct = []
    seq, st, ct = hexa(hexa_data, gen_seq, gen_st, gen_ct, time)
    # os.system('cls')
    print('Defined families in this instance:----------------------------')
    print(ins[7])
    print('The ws (Hexaly):-------')
    print(*seq, sep='\n')
    print('The ws start times:----')
    print(*st, sep='\n')
    print('The ws comp. times:----')
    print(*ct, sep='\n')
    return gen_seq

def incumbent2pyomo(incumbent, c, idle):
    for i in incumbent:
        if len(i) < c:
            to_add = c - len(i)
            ext = [idle] * to_add
            i.extend(ext)
    ws_x = {}
    ws_y = {}
    ws_z = {}
    for drn in range(1, len(incumbent) + 1):
        for slt in range(1, c+1):
            for vis in range(1, idle + 1):
                ws_x[vis, slt, drn] = 0
            ws_x[incumbent[drn-1][slt-1], slt, drn] = 1
    for drn in range(1, len(incumbent) + 1):
        for slt in range(1, c):
            ws_z[slt, drn] = 1
            for vis in range(1, idle + 1):
                for vis2 in range(1, idle + 1):
                    ws_y[vis, vis2, slt, drn] = 0
                    if slt>0 and incumbent[drn-1][slt] == vis and incumbent[drn-1][slt-1] == vis2:
                        ws_y[vis, vis2, slt+1, drn] = 1

    return incumbent, ws_x, ws_y, ws_z

if __name__ == '__main__':
    # instance values = [ndrones, condition, slot, charge, itimes)
    fixed = [2, 'fixed', 5, 12, 3] # 10 nodes including idle --->OK
    SB = [3, 'SB', 5, 0.3, 3] # 12 nodes including idle --->OK
    SB_RS = [4, 'SB_RS', 6, 0.8, 3] # 21 nodes including idle
    SB_RS_LA = [5, 'SB_RS_LA', 14, 2, 5] # 56 nodes including idle
    run(fixed, 20)