from datetime import date
import os
import sys
import random
from final_lp_ins import generate
from final_he_ins import generate1
from final_nl import nl_pyo
from final_lp import lp_pyo
from final_hexa import hexa
from pyomo.environ import value
import pickle
import numpy as np
import time
import pandas as pd

def run(city, verbose, seed, sol_time):
    a, b, c, d = city
    hexa_ins = generate1(ndrones=a, city=b, slot=c, charge=d, hexa_data_required=1, seed=seed)
    gen_seq = []
    gen_st = []
    gen_ct = []
    bres = []
    hexa(hexa_ins, gen_seq, gen_st, gen_ct, sol_time, bres, verbose)
    ins = generate(ndrones=a, city=b, slot=c, charge=d, seed=seed)
    lp_pyo(ins, verbose, sol_time)
    nl_pyo(ins, verbose, sol_time)

def compare(instance, report, collective_report):
    col_widths = 10
    nlp_ = []
    try:
        nlp_pickle = open('nlp.pickle', 'rb')
        nlp_ = pickle.load(nlp_pickle)
        assign_list = np.zeros((len(nlp_[2][4]), nlp_[2][3]), dtype=int)
        assign_dues = np.zeros((len(nlp_[2][4]), nlp_[2][3]), dtype=int)
        for ind in nlp_[0].x.index_set():
            if value(nlp_[0].x[ind]) > 0.9999:
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
    except:
        print('No pickle file')
    lp_ = []
    try:
        lp_pickle = open('lp.pickle', 'rb')
        lp_ = pickle.load(lp_pickle)
        lassign_list = np.zeros((len(lp_[2][4]), lp_[2][3]), dtype=int)
        lassign_dues = np.zeros((len(lp_[2][4]), lp_[2][3]), dtype=int)
        for ind in lp_[0].x.index_set():
            if value(lp_[0].x[ind]) > 0.9999:
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
    except:
        print('No pickle file')

    try:
        hexa_pickle = open('hexa.pickle', 'rb')
        hx_ = pickle.load(hexa_pickle)
        hassign_list = hx_[0]
        hc_values = hx_[2]
        hs_values = hx_[1]
        ht_values = hx_[3]
        h_obj_value = hx_[4]
        h_sol_time = hx_[5]
        i_maxx = hx_[6]
    except:
        print('No pickle file')

    # print('~~~~~~~~~~~~~~~~~~~~~~~~~~ Instance ~~~~~~~~~~~~~~~~~~~~~~~~~')
    # print('     seed:', seed1)  # nlp_[2][12])
    # print(' Families:', nlp_[2][7])
    # print('  i_times:', nlp_[2][5])
    # print('Fam slots:', nlp_[2][11])
    # print('Ear_dates:', nlp_[2][9])
    # print('Due_dates:', nlp_[2][1])
    # print('Monitor_t:', nlp_[2][2])

    # print('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~')
    # print('***** lp_objective:', value(lp_[0].obj_func))
    # # print('***** lp_Sol_time:', round(lp_[1].Solver.Wall_time, 3))

    # for i in range(0, lassign_list.shape[0]):
    #     print('     Drone (' + str(i + 1) + '):')
    #     print(''.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lassign_list[i]))
    #     print('s_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lps_values[i]))
    #     print('c_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lpc_values[i]))
    #     print('travels'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lptr_values[i]))
    #     print('charges'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in lpt_values[i]))
    #     print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

    # print('***** nlp_objective:', value(nlp_[0].obj_func))
    # # print('***** nlp_Sol_time:', round(nlp_[1].Solver.Wall_time, 3))
    #
    # for i in range(0, assign_list.shape[0]):
    #     print('     Drone (' + str(i + 1) + '):')
    #     print(''.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in assign_list[i]))
    #     print('s_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlps_values[i]))
    #     print('c_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlpc_values[i]))
    #     print('travels'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlptr_values[i]))
    #     print('charges'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlpt_values[i]))
    #     print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

    print('***** hx_objective:', h_obj_value)
    print('***** hx_Sol_time:', h_sol_time)

    for h in range(0, int(len(hassign_list)/2)):
        print('     Drone (' + str(h + 1) + '):')
        print(''.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in hassign_list[h]))
        print('s_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in hs_values[h]))
        print('c_times'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in hc_values[h]))
        # print('travels'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in nlptr_values[h]))
        print('charges'.ljust(col_widths), " | ".join(str(item).ljust(col_widths) for item in ht_values[h]))
        print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')


    # print('==========================================')

    # if report:
    #     folder_name = 'res_'+str(date.today())+'_Major'
    #     f_loc = os.getcwd()
    #     directory_path = os.path.join(f_loc, 'Comparison_modify', 'Results', folder_name)
    #     os.makedirs(directory_path, exist_ok=True)
    #     # file name: City + number of drones + number of slots + max charge
    #     with open(directory_path+'/'+instance[1]+'_'+str(instance[0])+'_'+str(instance[2])+'_'+str(instance[3])+'_'+time.strftime("%H%M%S")+'.txt', 'w+') as file:
    #         file.write('~~~~~~~~~~~~~~~~~~~ ' + str(instance) + ' ~~~~~~~~~~~~~~~~~~~\n')
    #         file.write('     seed: ' + str(seed1)+'\n')  #  str(nlp_[2][12])
    #         file.write(' Families: ' + str(nlp_[2][7])+'\n')
    #         file.write('  i_times: ' + str(nlp_[2][5])+'\n')
    #         file.write('Fam slots: ' + str(nlp_[2][11])+'\n')
    #         file.write('Ear_dates: ' + str(nlp_[2][9])+'\n')
    #         file.write('Due_dates: ' + str(nlp_[2][1])+'\n')
    #         file.write('Monitor_t: ' + str(nlp_[2][2])+'\n')
    #         file.write('~~~~~~~~~~~~~~~~~~~ Comparing the results ~~~~~~~~~~~~~~~~~~~\n')
    #         file.write(f'***** lp_Variables: {lp_[3]}\n')
    #         file.write(f'***** lp_Constraints: {lp_[4]}\n')
    #         file.write(f'***** lp_objective: {value(lp_[0].obj_func)}\n')
    #         # file.write(f'***** lp_Sol_time: {round(lp_[1].Solver.Time, 3)}\n')
    #         col_widths = 10
    #         for i in range(0, lassign_list.shape[0]):
    #             file.write(f'     Drone ({i + 1}):\n')
    #             file.write(
    #                 ''.ljust(col_widths) + " | ".join(str(item).ljust(col_widths) for item in lassign_list[i]) + '\n')
    #             file.write('s_times'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in lps_values[i]) + '\n')
    #             file.write('c_times'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in lpc_values[i]) + '\n')
    #             file.write('travels'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in lptr_values[i]) + '\n')
    #             file.write('charges'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in lpt_values[i]) + '\n')
    #             file.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
    #         file.write(f'***** nlp_Variables: {nlp_[3]}\n')
    #         file.write(f'***** nlp_Constraints: {nlp_[4]}\n')
    #         file.write(f'***** nlp_objective: {value(nlp_[0].obj_func)}\n')
    #         # file.write(f'***** nlp_Sol_time: {round(nlp_[1].Solver.Time, 3)}\n')
    #         for i in range(0, assign_list.shape[0]):
    #             file.write(f'     Drone ({i + 1}):\n')
    #             file.write(
    #                 ''.ljust(col_widths) + " | ".join(str(item).ljust(col_widths) for item in assign_list[i]) + '\n')
    #             file.write('s_times'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in nlps_values[i]) + '\n')
    #             file.write('c_times'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in nlpc_values[i]) + '\n')
    #             file.write('travels'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in nlptr_values[i]) + '\n')
    #             file.write('charges'.ljust(col_widths) + " | ".join(
    #                 str(item).ljust(col_widths) for item in nlpt_values[i]) + '\n')
    #             file.write('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
    if collective_report:
        print('I am generating collective report.')
        ins_data = {'lp_var': 00, 'lp_cons': 00, 'lp_obj': 00, 'lp_time': 00, 'nlp_var': 00, 'nlp_cons': 00, 'nlp_obj': 00, 'nlp_time': 00,
                    'hx_obj': 00, 'hx_time': 00, 'i_max': 00}
        if lp_:
            ins_data['lp_var'] = lp_[3]
            ins_data['lp_cons'] = lp_[4]
            ins_data['lp_obj'] = round(value(lp_[0].obj_func), 3)
            ins_data['lp_time'] = round(float(lp_[1].Solver.Wall_time), 3)
        if nlp_:
            ins_data['nlp_var'] = nlp_[3]
            ins_data['nlp_cons'] = nlp_[4]
            ins_data['nlp_obj'] = round(value(nlp_[0].obj_func), 3)
            ins_data['nlp_time'] = round(float(nlp_[1].Solver.Wall_time), 3)
        if hx_:
            ins_data['hx_obj'] = h_obj_value
            ins_data['hx_time'] = h_sol_time
            ins_data['i_max'] = i_maxx
    return ins_data

if __name__ == '__main__':

    # SB = 10 Real nodes + 1 iDL + 1 DP
    # RS = 9 Real nodes + 1 iDL + 1 DP
    # LA = 27 Real nodes + 1 iDL + 1 DP
    # SB_RS = 19 Real nodes + 2 iDLs + 1 DP
    # SB_LA = 37 Real nodes + 2 iDLs + 1 DP
    # RS_LA = 36 Real nodes + 2 iDLs + 1 DP
    # SB_RS_LA = 46 Real nodes + 3 iDLs + 1 DP
    # LA: 4,8 and 5,8
    # SB-RS-LA: 8,8 and 9,7
    num_drones = [7, 8, 9, 9]
    num_slots = [9, 9, 8, 7]
    collective_data = pd.DataFrame(columns=['city','Iter','drones','slots','lp_var','lp_cons','lp_obj','lp_time','nlp_var','nlp_cons','nlp_obj','nlp_time', 'hx_obj', 'hx_time', 'i_max', 'seed'])
    # for i in range(len(num_slots)):
    for i in range(4):
        instance_ = [num_drones[i], 'SB_LA', num_slots[i], 360]
        for iter_ in range(10):
            for jf in ['lp.pickle', 'nlp.pickle', 'hexa.pickle']:
                if os.path.exists(jf):
                    os.remove(jf)
                    print(f"{jf} has been deleted.")
                else:
                    print(f"{jf} does not exist.")
            seed1 = random.randrange(sys.maxsize)
            # seed1 = 4116131705355157001
            random.seed(seed1)
            print('Iteration:', (10*i) + iter_+1, '====> seed = ', seed1)
            run(instance_, verbose=True, seed=seed1, sol_time=3600)
            sol_ = compare(instance_, report=False, collective_report=True)
            new_row = {
                'city': instance_[1], 'Iter': 1+iter_, 'drones': num_drones[i], 'slots': num_slots[i],
                'lp_var': sol_['lp_var'], 'lp_cons': sol_['lp_cons'], 'lp_obj': sol_['lp_obj'], 'lp_time': sol_['lp_time'],
                'nlp_var': sol_['nlp_var'], 'nlp_cons': sol_['nlp_cons'], 'nlp_obj': sol_['nlp_obj'], 'nlp_time': sol_['nlp_time'],
                'hx_obj': sol_['hx_obj'], 'hx_time': sol_['hx_time'], 'i_max': str(sol_['i_max']), 'seed': [seed1]}
            collective_data = pd.concat([collective_data, pd.DataFrame([new_row])], ignore_index=True)
            current_directory = os.getcwd()
            filename = 'collective_data'+'_'+str(date.today())+time.strftime("%H%M%S")+'.csv'
            file_path = os.path.join(current_directory, filename)
            collective_data.to_csv(file_path, index=False)
            collective_data.to_excel("collective_data.xlsx", index=False, engine='openpyxl')
