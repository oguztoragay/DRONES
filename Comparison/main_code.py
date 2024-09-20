from datetime import date

import os

import random_instance
from Comparison import instance_generator
from nl_pyomo2 import nl_pyo
from lp_pyomo import lp_pyo
from random_instance import generate
from instance_generator import generate
from pyomo.environ import value
import pickle
import numpy as np
import time

def run(city, verbose):
    a, b, c, d, e = city
    # ins = random_instance.generate(ndrones=a, city=b, slot=c, charge=d, itimes=e)
    ins = instance_generator.generate(ndrones=a, city=b, slot=c, charge=d, itimes=e)
    # lp_pyo(ins, verbose)
    nl_pyo(ins, verbose)

def compare(instance, report):
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
        nlpc_values.append(round(value(nlp_[0].c[ind]), 4))
        nlps_values.append(round(value(nlp_[0].s[ind]), 4))
        nlpt_values.append(round(value(nlp_[0].t[ind]), 4))
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


    print('~~~~~~~~~~~~~~~~~~~~~~~~~~ Instance ~~~~~~~~~~~~~~~~~~~~~~~~~')
    print(' Families:', nlp_[2][7])
    print('Due_dates:', nlp_[2][1])
    print('Monitor_t:', nlp_[2][2])
    print('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~')
    print('***** lp_objective:', value(lp_[0].obj_func))
    print('***** lp_Sol_time:', round(lp_[1].Solver.Time, 3))
    for i in range(0, lassign_list.shape[0]):
        print('     Drone (' + str(i + 1) + '):', *lassign_list[i], sep=' --> ')
        print('     s_times', *lps_values[i], sep=' --> ')
        print('     c_times', *lpc_values[i], sep=' --> ')
        print('      charge', *lpt_values[i], sep=' --> ')
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('***** nlp_objective:', value(nlp_[0].obj_func))
    # print('***** nlp_Sol_time:', round(nlp_[1].Solver[0]['Wallclock time'], 3))
    print('***** nlp_Sol_time:', round(nlp_[1].Solver.Time, 3))
    for i in range(0, assign_list.shape[0]):
        print('     Drone (' + str(i + 1) + '):', *assign_list[i], sep=' --> ')
        print('     s_times', *nlps_values[i], sep=' --> ')
        print('     c_times', *nlpc_values[i], sep=' --> ')
        print('      charge', *nlpt_values[i], sep=' --> ')
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('==========================================')
    if report:
        folder_name = 'result_'+str(date.today())
        f_loc = os.getcwd()
        try:
            os.mkdir(f_loc + '/'+folder_name)
        except:
            print('The named folder exists in the given path.')
        c_f = open(folder_name+'/'+'Output_record '+time.strftime("%Y%m%d-%H%M%S")+'.txt', 'w+')
        c_f.write('~~~~~~~~~~~~~~~~~~~ ' + str(instance) + ' ~~~~~~~~~~~~~~~~~~~\n')
        c_f.write(' Families: ' + str(nlp_[2][7])+'\n')
        c_f.write('Due_dates: ' + str(nlp_[2][1])+'\n')
        c_f.write('Monitor_t: ' + str(nlp_[2][2])+'\n')
        c_f.write('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('***** lp_objective: ' + str(value(lp_[0].obj_func))+'\n')
        c_f.write('***** lp_Sol_time: ' + str(round(lp_[1].Solver.Time, 3))+'\n')
        for i in range(0, lassign_list.shape[0]):
            c_f.write('Drone(' + str(i + 1) + '): ' + ' --> '.join(map(str, lassign_list[i])) + '\n')
            c_f.write(' s_times: ' + ' --> '.join(map(str, lps_values[i])) + '\n')
            c_f.write(' c_times: ' + ' --> '.join(map(str, lpc_values[i])) + '\n')
            c_f.write('  charge: ' + ' --> '.join(map(str, lpt_values[i])) + '\n')
            c_f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('***** nlp_objective: ' + str(value(nlp_[0].obj_func)) + '\n')
        # c_f.write('***** nlp_Sol_time:' + str(round(nlp_[1].Solver[0]['Wallclock time'], 3)) + '\n')
        c_f.write('***** nlp_Sol_time:' + str(round(nlp_[1].Solver.Time, 3)) + '\n')
        for i in range(0, assign_list.shape[0]):
            c_f.write('Drone(' + str(i + 1) + '): ' + ' --> '.join(map(str, assign_list[i])) + '\n')
            c_f.write(' s_times: ' + ' --> '.join(map(str, nlps_values[i])) + '\n')
            c_f.write(' c_times: ' + ' --> '.join(map(str, nlpc_values[i])) + '\n')
            c_f.write('  charge: ' + ' --> '.join(map(str, nlpt_values[i])) + '\n')
            c_f.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        c_f.write('==========================================\n')
        c_f.close()


if __name__ == '__main__':
    # instance values = [ndrones, condition, slot, charge, itimes)
    fixed = [2, 'fixed', 6, 10, 2]  # 10 nodes including idle --->OK
    SB = [3, 'SB', 5, 400, 100]  # 12 nodes including idle
    SB_RS = [3, 'SB_RS', 10, 300, 100]  # 21 nodes including idle
    SB_RS_LA = [5, 'SB_RS_LA', 20, 4, 5]  # 56 nodes including idle
    run(SB_RS, verbose=True)
    compare(SB_RS, report=False)

    # Options:
    # Control the verbosity of the solvers by changing the verbose=True/False
    # If you want to record the solution, change the report=True ==>
    # makes a .txt file to record the current comparison of the solutions
