from datetime import date

import greedy
import os
from nl_hexaly4 import hexa
# from nl_pyomo_model import nl_pyo
from nl_pyomo2 import nl_pyo
from lp_pyomo import lp_pyo
from random_instance import generate
from random_instance import mprint
from pyomo.environ import value
import ast
import pickle
import numpy as np
import time

def run(city, time_, verbose):
    a, b, c, d, e = city
    ins = generate(ndrones=a, condition=b, slot=c, charge=d, itimes=e)
    incumbent = ins2incumbent(ins, a, b, c, d, e, time_, verbose)
    warm_start = []
    ws_x = []
    ws_y = []
    ws_z = []
    # warm_start, ws_x, ws_y, ws_z = incumbent2pyomo(incumbent, c, ins[7][-1][0])
    lp_pyo(ins, warm_start, ws_x, ws_y, ws_z, verbose)
    nl_pyo(ins, warm_start, ws_x, ws_y, ws_z, verbose)
def ins2incumbent(ins, a, b, c, d, e, time_, verbose):
    hexa_data = [a, c, ins[0], ins[4], e, ins[2], ins[1], ins[7]]
    gen_seq = []
    gen_st = []
    gen_ct = []
    bres = []
    seq, st, ct, bs = hexa(hexa_data, gen_seq, gen_st, gen_ct, time_, bres, verbose)
    # os.system('cls')
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
def compare(rep):
    nlp_pickle = open('nlp.pickle', "rb")
    nlp_ = pickle.load(nlp_pickle)
    assign_list = np.zeros((len(nlp_[2][4]), nlp_[2][3]), dtype=int)
    assign_dues = np.zeros((len(nlp_[2][4]), nlp_[2][3]), dtype=int)
    for ind in nlp_[0].x.index_set():
        if value(nlp_[0].x[ind]) == 1:
            assign_list[ind[2]-1, ind[1]-1] = ind[0]
            assign_dues[ind[2]-1, ind[1]-1] = nlp_[2][1][ind[0]-1]
    nllist_c = sorted(list(nlp_[0].c.index_set()), key=lambda x: x[1])
    nlpc_values = []
    nlps_values = []
    nlpt_values = []
    for ind in nllist_c:
        nlpc_values.append(round(value(nlp_[0].c[ind]),4))
        nlps_values.append(round(value(nlp_[0].s[ind]),4))
        nlpt_values.append(round(value(nlp_[0].t[ind]),4))
    nlpc_values = np.reshape(nlpc_values, (len(nlp_[2][4]), nlp_[2][3]))
    nlps_values = np.reshape(nlps_values, (len(nlp_[2][4]), nlp_[2][3]))
    nlpt_values = np.reshape(nlpt_values, (len(nlp_[2][4]), nlp_[2][3]))

    lp_pickle = open('lp.pickle', "rb")
    lp_ = pickle.load(lp_pickle)
    lassign_list = np.zeros((len(lp_[2][4]), lp_[2][3]), dtype=int)
    lassign_dues = np.zeros((len(lp_[2][4]), lp_[2][3]), dtype=int)
    for ind in lp_[0].x.index_set():
        if value(lp_[0].x[ind]) == 1:
            lassign_list[ind[2]-1, ind[1]-1] = ind[0]
            lassign_dues[ind[2]-1, ind[1]-1] = lp_[2][1][ind[0]-1]
    lplist_c = sorted(list(lp_[0].c.index_set()), key=lambda x: x[1])
    lpc_values = []
    lps_values = []
    lpt_values = []
    for ind in lplist_c:
        lpc_values.append(round(value(lp_[0].c[ind]),4))
        lps_values.append(round(value(lp_[0].s[ind]),4))
        lpt_values.append(round(value(lp_[0].t[ind]),4))
    lpc_values = np.reshape(lpc_values, (len(lp_[2][4]), lp_[2][3]))
    lps_values = np.reshape(lps_values, (len(lp_[2][4]), lp_[2][3]))
    lpt_values = np.reshape(lpt_values, (len(lp_[2][4]), lp_[2][3]))


    hxl_pickle = open('hxl.pickle', "rb")
    hxl_ = pickle.load(hxl_pickle)


    print('~~~~~~~~~~~~~~~~~~~~~~~~~~ Instance ~~~~~~~~~~~~~~~~~~~~~~~~~')
    print(' Families:', nlp_[2][7])
    print('Due_dates:', nlp_[2][1])
    print('Monitor_t:', nlp_[2][2])
    print('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~')
    print('~~~~~ lp_objective:', value(lp_[0].obj_func))
    for i in range(0, lassign_list.shape[0]):
        print('     Drone (' + str(i + 1) + '):', *lassign_list[i], sep=' --> ')
        print('     s_times', *lps_values[i], sep=' --> ')
        print('     c_times', *lpc_values[i], sep=' --> ')
        print('      charge', *lpt_values[i], sep=' --> ')
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('~~~~~ nlp_objective:', value(nlp_[0].obj_func))
    for i in range(0, assign_list.shape[0]):
        print('     Drone (' + str(i + 1) + '):', *assign_list[i], sep=' --> ')
        print('     s_times', *nlps_values[i], sep=' --> ')
        print('     c_times', *nlpc_values[i], sep=' --> ')
        print('      charge', *nlpt_values[i], sep=' --> ')
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('~~~~~ hxl_objective:', hxl_[4])
    for i in range(0, len(hxl_[0])):
        print('     Drone ('+str(i+1)+'):', *hxl_[0][i], sep=' --> ')
        print('     s_times', *hxl_[1][i], sep=' --> ')
        print('     c_times', *hxl_[2][i], sep=' --> ')
        print('      charge', *hxl_[3][i], sep=' --> ')
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('==========================================')
    if rep:
        folder_name = 'result_'+str(date.today())
        f_loc = os.getcwd()
        try:
            os.mkdir(f_loc + '/'+folder_name)
        except:
            None
        c_f = open(folder_name+'/'+'Output_record '+time.strftime("%Y%m%d-%H%M%S")+'.txt', 'w+')
        c_f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~ Instance ~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        c_f.write(' Families: '+ str(nlp_[2][7])+'\n')
        c_f.write('Due_dates: '+ str(nlp_[2][1])+'\n')
        c_f.write('Monitor_t: '+ str(nlp_[2][2])+'\n')
        c_f.write('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('~~~~~ lp_objective: '+ str(value(lp_[0].obj_func))+'\n')
        for i in range(0, lassign_list.shape[0]):
            c_f.write('     Drone (' + str(i + 1) + '):'+ str(lassign_list[i]) +'\n')
            c_f.write('     s_times: ' + str(lps_values[i]) + '\n')
            c_f.write('     c_times: ' + str(lpc_values[i]) + '\n')
            c_f.write('      charge: ' + str(lpt_values[i]) + '\n')
            c_f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('~~~~~ nlp_objective: ' + str(value(nlp_[0].obj_func)) + '\n')
        for i in range(0, assign_list.shape[0]):
            c_f.write('     Drone (' + str(i + 1) + '):'+ str(assign_list[i]) +'\n')
            c_f.write('     s_times: ' + str(nlps_values[i]) + '\n')
            c_f.write('     c_times: ' + str(nlpc_values[i]) + '\n')
            c_f.write('      charge: ' + str(nlpt_values[i]) + '\n')
            c_f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('~~~~~ hxl_objective:' + str(hxl_[4])+'\n')
        for i in range(0, len(hxl_[0])):
            c_f.write('     Drone (' + str(i + 1) + '):' + str(hxl_[0][i]) + '\n')
            c_f.write('     s_times' + str(hxl_[1][i]) + '\n')
            c_f.write('     c_times' + str(hxl_[2][i]) + '\n')
            c_f.write('      charge' + str(hxl_[3][i]) + '\n')
            c_f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('==========================================\n')
        c_f.close()


if __name__ == '__main__':
    # instance values = [ndrones, condition, slot, charge, itimes)
    fixed = [2, 'fixed', 6, 12, 3]  # 10 nodes including idle --->OK
    SB = [4, 'SB', 4, 0.4, 0.5]  # 12 nodes including idle
    SB_M = [3, 'SB_M', 4, 15, 15]  # 12 nodes including idle
    SB_RS = [4, 'SB_RS', 6, 1, 1]  # 21 nodes including idle
    SB_RS_LA = [5, 'SB_RS_LA', 15, 4, 5]  # 56 nodes including idle
    run(fixed, 30, verbose=True)
    compare(rep=True)

    # Options:
    # Control the verbosity of the solvers by changing the verbose=True/False
    # If you want to record the solution, change the rep=True ==> makes a .txt file to record the current comparison of the solutions