from datetime import date
import os
import random
import sys
from Comparison import instance_gen4
from nl_pyomo2 import nl_pyo
from lp_pyomo2 import lp_pyo
from pyomo.environ import value
import pickle
import numpy as np
import time

def run(city, verbose):
    a, b, c, d = city
    # ins = random_instance.generate(ndrones=a, city=b, slot=c, charge=d, itimes=e)
    ins = instance_gen4.generate(ndrones=a, city=b, slot=c, charge=d)
    lp_pyo(ins, verbose)
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
    nlptr_values = np.zeros([len(nlp_[2][4]), nlp_[2][3]])
    for ind in nllist_c:
        nlpc_values.append(round(value(nlp_[0].c[ind]), 4))
        nlps_values.append(round(value(nlp_[0].s[ind]), 4))
        nlpt_values.append(round(value(nlp_[0].t[ind]), 4))
    for i in range(assign_list.shape[0]):
        for j in range(assign_list.shape[1]-1):
            nlptr_values[i][j] = nlp_[2][0][assign_list[i][j]-1][assign_list[i][j+1]-1]
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
    lptr_values = np.zeros([len(lp_[2][4]), lp_[2][3]])
    for ind in lplist_c:
        lpc_values.append(round(value(lp_[0].c[ind]),4))
        lps_values.append(round(value(lp_[0].s[ind]),4))
        lpt_values.append(round(value(lp_[0].t[ind]),4))
    for i in range(lassign_list.shape[0]):
        for j in range(lassign_list.shape[1]-1):
            lptr_values[i][j] = lp_[2][0][lassign_list[i][j]-1][lassign_list[i][j+1]-1]
    lpc_values = np.reshape(lpc_values, (len(lp_[2][4]), lp_[2][3]))
    lps_values = np.reshape(lps_values, (len(lp_[2][4]), lp_[2][3]))
    lpt_values = np.reshape(lpt_values, (len(lp_[2][4]), lp_[2][3]))


    print('~~~~~~~~~~~~~~~~~~~~~~~~~~ Instance ~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('     seed:', seed1)  # nlp_[2][12])
    print(' Families:', nlp_[2][7])
    print('  i_times:', nlp_[2][5])
    print('Fam slots:', nlp_[2][11])
    print('Ear_dates:', nlp_[2][9])
    print('Due_dates:', nlp_[2][1])
    print('Monitor_t:', nlp_[2][2])

    print('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~')
    print('***** lp_objective:', value(lp_[0].obj_func))
    print('***** lp_Sol_time:', round(lp_[1].Solver.Time, 3))
    col_widths = 10
    for i in range(0, lassign_list.shape[0]):
        print('     Drone (' + str(i + 1) + '):')
        print(''.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lassign_list[i]))
        print('s_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lps_values[i]))
        print('c_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lpc_values[i]))
        print('travels'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lptr_values[i]))
        print('charges'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lpt_values[i]))
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

    print('***** nlp_objective:', value(nlp_[0].obj_func))
    print('***** nlp_Sol_time:', round(nlp_[1].Solver.Time, 3))

    for i in range(0, assign_list.shape[0]):
        print('     Drone (' + str(i + 1) + '):')
        print(''.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in assign_list[i]))
        print('s_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlps_values[i]))
        print('c_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlpc_values[i]))
        print('travels'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlptr_values[i]))
        print('charges'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlpt_values[i]))
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    print('==========================================')

    if report:
        folder_name = 'res_'+str(date.today())+'_equality'
        f_loc = os.getcwd()
        directory_path = os.path.join(f_loc, 'Comparison', 'Results', folder_name)
        os.makedirs(directory_path, exist_ok=True)
        # file name: City + number of drones + number of slots + max charge
        with open(directory_path+'/'+instance[1]+'_'+str(instance[0])+'_'+str(instance[2])+'_'+str(instance[3])+'_'+time.strftime("%H%M%S")+'.txt', 'w+') as file:
            file.write('~~~~~~~~~~~~~~~~~~~ ' + str(instance) + ' ~~~~~~~~~~~~~~~~~~~\n')
            file.write('     seed: ' + str(seed1)+'\n')  #  str(nlp_[2][12])
            file.write(' Families: ' + str(nlp_[2][7])+'\n')
            file.write('  i_times: ' + str(nlp_[2][5])+'\n')
            file.write('Fam slots: ' + str(nlp_[2][11])+'\n')
            file.write('Ear_dates: ' + str(nlp_[2][9])+'\n')
            file.write('Due_dates: ' + str(nlp_[2][1])+'\n')
            file.write('Monitor_t: ' + str(nlp_[2][2])+'\n')
            file.write('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~\n')
            file.write(f'***** lp_Variables: {lp_[3]}\n')
            file.write(f'***** lp_Constraints: {lp_[4]}\n')
            file.write(f'***** lp_objective: {value(lp_[0].obj_func)}\n')
            file.write(f'***** lp_Sol_time: {round(lp_[1].Solver.Time, 3)}\n')
            col_widths = 10
            for i in range(0, lassign_list.shape[0]):
                file.write(f'     Drone ({i + 1}):\n')
                file.write(
                    ''.ljust(col_widths) + " | ".join(str(item).ljust(col_widths) for item in lassign_list[i]) + '\n')
                file.write('s_times'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in lps_values[i]) + '\n')
                file.write('c_times'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in lpc_values[i]) + '\n')
                file.write('travels'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in lptr_values[i]) + '\n')
                file.write('charges'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in lpt_values[i]) + '\n')
                file.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
            file.write(f'***** nlp_Variables: {nlp_[3]}\n')
            file.write(f'***** nlp_Constraints: {nlp_[4]}\n')
            file.write(f'***** nlp_objective: {value(nlp_[0].obj_func)}\n')
            file.write(f'***** nlp_Sol_time: {round(nlp_[1].Solver.Time, 3)}\n')
            for i in range(0, assign_list.shape[0]):
                file.write(f'     Drone ({i + 1}):\n')
                file.write(
                    ''.ljust(col_widths) + " | ".join(str(item).ljust(col_widths) for item in assign_list[i]) + '\n')
                file.write('s_times'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in nlps_values[i]) + '\n')
                file.write('c_times'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in nlpc_values[i]) + '\n')
                file.write('travels'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in nlptr_values[i]) + '\n')
                file.write('charges'.ljust(col_widths) + " | ".join(
                    str(item).ljust(col_widths) for item in nlpt_values[i]) + '\n')
                file.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
        #     file.write('==========================================\n')
        # file.write('==========================================\n')
        # file.close()


if __name__ == '__main__':
    # instance values = [ndrones, condition, slot, charge)
    SB = [3, 'SB', 5, 360]  # 12 nodes including iDL and DP
    RS = [3, 'RS', 4, 360]  # 11 nodes including iDL and DP
    LA = [3, 'LA', 13, 360]  # 37 nodes including iDL and DP
    SB_RS = [3, 'SB_RS', 8, 360]  # 22 nodes including iDLs and DP
    SB_LA = [3, 'SB_LA', 15, 720]  # 48 nodes including iDLs and DP
    RS_LA = [3, 'RS_LA', 15, 720]  # 47 nodes including iDLs and DP
    SB_RS_LA = [5, 'SB_RS_LA', 11, 720]  # 58 nodes including iDLs and DP (now 50)
    for i in range(10):
        seed1 = random.randrange(sys.maxsize)
        random.seed(seed1)
        print(i, ': seed === ', seed1)
        run(SB, verbose=True)
        compare(SB, report=True)

    # Options:
    # Control the verbosity of the solvers by changing the verbose=True/False
    # If you want to record the solution, change the report=True ==>
    # makes a .txt file to record the current comparison of the solutions
