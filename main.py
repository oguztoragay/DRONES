from pulp import *
import numpy as np
import pandas as pd

n_demandnode = 9 # plus depot = 6  Max visit numbers for each location:
family0 = 1
family1 = np.array([2,3,4])
family2 = np.array([5,6,7])
family3 = np.array([8,9])

#n_chargingstation = 1
n_drones = 2
n_slot = 7 ############ 7 before
monitor_time_matrix = np.array([3, 2, 2, 2, 1, 1, 1, 1, 1]) #Pj #[2, 1, 2, 3, 4, 5, 5]
Traveltime_demandnodes_matrix = np.array([[0,3,3,3,4,4,4,1,1 ],[3,0,0,0,1,1,1,2,2],[3,0,0,0,1,1,1,2,2],[3,0,0,0,1,1,1,2,2],[4,1,1,1,0,0,0,3,3],[4,1,1,1,0,0,0,3,3],[4,1,1,1,0,0,0,3,3],[1,2,2,2,3,3,3,0,0],[1,2,2,2,3,3,3,0,0]]) #Sjk
#max_intervisit_time_matrix = np.array([24, 5, 5, 5, 5]) #depot due date should be a large number.
B = 10000
#UB = 24 # upper bound of dj for Z
#LB = 0 # lower bound of dj for Z
due_dates = np.array([0, 24, 24, 24, 24, 24, 24, 5, 10]) #dj
Drone_Charge = 7 #8 before
UB = 1000 # upper bound of C
intervisit_time = 5 # 5 before MAX
# Try to keep the scale of Traveltime_demandnodes_matrix and intervisit_time compatable!
# if drone visits a location at time 1 it should visit the same location by the time 6 at least once. check! YES

for i in family1:
    print(i)

model = LpProblem("Parallel machines",LpMinimize)

Xjri_names = [str(j)+str(r)+str(i) for j in range(1, n_demandnode+1) for r in range (1, n_slot+1) for i in range (1, n_drones+1)]
#if customer j is assigned to the r slot of the drone i
#print("Xjri Indices:", Xjri_names)

Yjkri_names = [str(j)+str(k)+str(r)+str(i) for j in range(1, n_demandnode+1) for k in range (1, n_demandnode+1) for r in range (1, n_slot+1) for i in range (1, n_drones+1)]
# if customer j is assigned to the r slot and customer k is assigned to r-1 slot of drone i
#print("Yjkri Indices:", Yjkri_names)

Cri_names = [str(r)+str(i) for r in range (1, n_slot+1) for i in range (1, n_drones+1)]
#The Completion time of the monitoring customer on the r slot of the drone i
#print("Cri Indices:", Cri_names)

Zri_names = [str(r)+str(i) for r in range (1, n_slot+1) for i in range (1, n_drones+1)]
#The Completion time of the monitoring customer on the r slot of the drone i
#print("Zri Indices:", Zri_names)

Wri_names = [str(r)+str(i) for r in range (1, n_slot+1) for i in range (1, n_drones+1)]
#The Completion time of the monitoring customer on the r slot of the drone i
#print("Zri Indices:", Zri_names)

Vjri_names = [str(j)+str(r)+str(i) for j in range(1, n_demandnode+1) for r in range (1, n_slot+1) for i in range (1, n_drones+1)]
# Replacement for CriXjri
#print("Vjri Indices:", Vjri_names)

X = LpVariable.matrix("X", Xjri_names, cat = "Binary")
allocation1 = np.array(X).reshape(n_demandnode,n_slot,n_drones)
#print("Customer allocation j to slot r of drone i: ")
#print(allocation1)

Y = LpVariable.matrix("Y", Yjkri_names, cat = "Binary")
allocation2 = np.array(Y).reshape(n_demandnode,n_demandnode,n_slot,n_drones)
#print("Consequtive assignment of customer j on slot r after customer k, on drone i: ")
#print(allocation2)

C = LpVariable.matrix("C", Cri_names, cat="Integer")
allocation3 = np.array(C).reshape(n_slot,n_drones)
#print("completion time of monitoring the customer on slot r of drone i: ")
#print(allocation3)

Z = LpVariable.matrix("Z", Zri_names, cat="Binary")
allocation4 = np.array(Z).reshape(n_slot,n_drones)
#print("Consequtive assignment of charging on slot r of drone i: ")
#print(allocation4)

W = LpVariable.matrix("W", Wri_names, cat="Integer")
allocation5 = np.array(W).reshape(n_slot,n_drones)
#print("dummy variable for customer seved on slot r of drone i: ")
#print(allocation5)

V = LpVariable.matrix("V", Vjri_names, cat="Integer")
allocation6 = np.array(V).reshape(n_demandnode,n_slot,n_drones)
#print("dummy variable for customer j seved on slot r of drone i: ")
#print(allocation6)

Lmax= LpVariable ("Lmax", cat="Integer", lowBound= 0)  # when calculating Lmax do not include depot!!!
#print(Lmax)

#Completion = LpVariable ("Completion", cat="Continuous")  # when calculating Lmax do not include depot!!!
#print(Completion)

obj_func = Lmax #Completion
print(obj_func)
model +=  obj_func
print(model)

#Constraint objective function
#print(Completion == lpSum(allocation3[r][i] for r in range (n_slot) for i in range (n_drones)))
#model += Completion == lpSum(allocation3[r][i] for r in range (n_slot) for i in range (n_drones))

#constratint 2 : each job should happen only once
for j in range(1, n_demandnode):
   # print (lpSum(allocation1[j][r][i] for r in range (0, n_slot) for i in range (0, n_drones)) == 1)
    model += lpSum(allocation1[j][r][i] for r in range (0, n_slot) for i in range (0, n_drones)) == 1

#constratint 3:
#each slot on each drone can be empty or filled with a job or going to depot
# Later: Check being empty in the solution!!!!
for r in range(n_slot):
    for i in range(n_drones):
       # print (lpSum(allocation1[j][r][i] for j in range(1,n_demandnode)) <= 1)
        model += lpSum(allocation1[j][r][i] for j in range(1,n_demandnode)) <= 1

#constratint 4: only for depot X111?
#each slot on each drone can be empty or filled with a job or going to depot
# Later: Check being empty in the solution!!!!
for r in range(1,n_slot):
    for i in range(n_drones):
       # print (allocation1[0][r][i] <= 1)
        model += allocation1[0][r][i] <= 1


#constratint 5: Depot cannot happen in the first slot of each drone
for i in range(n_drones):
   # print (allocation1[0][0][i] == 0)
    model += allocation1[0][0][i] == 0

#constratint 6: no empty slots between jobs
for r in range(1,n_slot):
    for i in range(n_drones):
        print (lpSum(allocation1[j][r][i] - allocation1[j][r-1][i] for j in range(n_demandnode)) <= 0)
        model += lpSum(allocation1[j][r][i] - allocation1[j][r-1][i] for j in range(n_demandnode)) <= 0

#constratint 7 : removes repeated solutions: drones are identical
for i in range(n_drones-1):
   # print (lpSum(allocation1[j][r][i+1] - allocation1[j][r][i] for r in range (n_slot) for j in range(n_demandnode)) <= 0)
    model += lpSum(allocation1[j][r][i+1] - allocation1[j][r][i] for r in range (n_slot) for j in range(n_demandnode)) <= 0

#Traveltime_demandnodes_matrix

# Constraint to avoid depot at the first slot of each drone;
#print (allocation1[0][0][0] == 0)
#model += allocation1[0][0][0] == 0

# Constraint to avoid depot at the first slot of each drone;
#print (allocation1[0][0][1] == 0)
#model += allocation1[0][0][1] == 0

#constratint 8 This will take care of not having depot assigned to the first spot on each drone.
for i in range(n_drones):
  #  print (allocation3[0][i] == lpSum((Traveltime_demandnodes_matrix[0][j] + monitor_time_matrix[j])*allocation1[j][0][i] for j in range(1,n_demandnode)))
    model += allocation3[0][i] == lpSum((Traveltime_demandnodes_matrix[0][j]+monitor_time_matrix[j])*allocation1[j][0][i] for j in range(1,n_demandnode))

# constratint 9: Completion time
for i in range(n_drones):
    for r in range(1, n_slot):
        # print (allocation3[r][i]== (allocation3[r-1][i])+ lpSum( Traveltime_demandnodes_matrix[k][j]*allocation2[j][k][r][i] for j in range(n_demandnode) for k in range(n_demandnode) if j is not k)+lpSum(monitor_time_matrix[j]*allocation1[j][r][i] for j in range(n_demandnode)))
        model += allocation3[r][i] == (allocation3[r - 1][i]) + lpSum(
            Traveltime_demandnodes_matrix[k][j] * allocation2[j][k][r][i] for j in range(n_demandnode) for k in
            range(n_demandnode) if j is not k) + lpSum(
            monitor_time_matrix[j] * allocation1[j][r][i] for j in range(n_demandnode))

#constratint 10: Defines Y
for i in range(n_drones):
    for j in range(n_demandnode):
        for k in range(n_demandnode):
            if j is not k:
                for r in range(1,n_slot):
                    #print (lpSum(allocation1[j][r][i]+allocation1[k][r-1][i]-1)<=allocation2[j][k][r][i])
                    model += lpSum(allocation1[j][r][i]+allocation1[k][r-1][i]-1)<=allocation2[j][k][r][i]

#constratint 11: defines Y
for i in range(n_drones):
    for j in range(n_demandnode):
        for k in range(n_demandnode):
            if j is not k:
                for r in range(1,n_slot):
                    #print (lpSum(0.5*(allocation1[j][r][i]+allocation1[k][r-1][i])- allocation2[j][k][r][i])>=0)
                    model += lpSum(0.5*(allocation1[j][r][i]+allocation1[k][r-1][i])- allocation2[j][k][r][i])>=0

#constratint 12:  Lmax original  -B*(1-lpSum(allocation1[j][r][i] for j in range(n_demandnode)))

for r in range(n_slot):
    for i in range(n_drones):
       # print(Lmax>=allocation3[r][i]-lpSum(due_dates[j]*allocation1[j][r][i] for j in range(n_demandnode))-B*(1-lpSum(allocation1[j][r][i] for j in range(n_demandnode))))
        model += Lmax>=allocation3[r][i]-lpSum(due_dates[j]*allocation1[j][r][i] for j in range(n_demandnode))-B*(1-lpSum(allocation1[j][r][i] for j in range(n_demandnode)))

# New constarint for C
#for r in range(n_slot):
  #  for i in range(n_drones):
   #     print(allocation3[r][i] <= B*lpSum(allocation1[j][r][i] for j in range(n_demandnode)))
   #     model += allocation3[r][i] <= B*lpSum(allocation1[j][r][i] for j in range(n_demandnode))

# Constraint: drone should go to charging station if the completion time exceeds charge available on drone
# Try to work with Drone_Charge    *lpSum(1+allocation4[r][i] for r in range(0,a))
#### Didn't read the last slot!!!
#### shouldn't have sum over c: C31 already has c11 + c21 in it!!
#for i in range(n_drones):
 #   for r in range(n_slot-1):
    #    print(allocation3[r][i] - Drone_Charge*(1+lpSum(allocation4[a][i] for a in range(0,r+1))) <= B*allocation4[r+1][i])
    #    model += allocation3[r][i] - Drone_Charge*(1+lpSum(allocation4[a][i] for a in range(0,r+1))) <= B*allocation4[r+1][i]

# Constraint 13

# TEST Constraint: drone should go to charging station if the completion time exceeds charge available on drone
# Try to work with Drone_Charge    *lpSum(1+allocation4[r][i] for r in range(0,a))
#### Didn't read the last slot!!!
#### shouldn't have sum over c: C31 already has c11 + c21 in it!!
for i in range(n_drones):
        #print(allocation3[0][i] - Drone_Charge <= B*allocation4[1][i])
        model += allocation3[0][i] - Drone_Charge <= B*allocation4[1][i]

# Constraint 14 below!!!

# Constraint 15

# TESt 2Constraint: drone should go to charging station if the completion time exceeds charge available on drone
# Try to work with Drone_Charge    *lpSum(1+allocation4[r][i] for r in range(0,a))
#### Didn't read the last slot!!!
#### shouldn't have sum over c: C31 already has c11 + c21 in it!!
for i in range(n_drones):
    for r in range(1,n_slot-1):
        #print(allocation3[r][i] - lpSum(allocation5[b][i] for b in range(0,r)) - Drone_Charge <= B*allocation4[r+1][i])
        model += allocation3[r][i] - lpSum(allocation5[b][i] for b in range(0,r)) - Drone_Charge <= B*allocation4[r+1][i]

# Constraint 16 below!!!

# Define W: 17
for i in range(n_drones):
    for r in range(2,n_slot):
        #print(allocation5[r-1][i] >= allocation3[r-1][i] - UB*(1-allocation4[r][i]))
        model += allocation5[r-1][i] >= allocation3[r-1][i] - UB*(1-allocation4[r][i])

# Define W: 18  ## we do not consider r= 7 the last slot
for i in range(n_drones):
    for r in range(2,n_slot):
        #print(allocation5[r-1][i] <= allocation3[r-1][i])
        model += allocation5[r-1][i] <= allocation3[r-1][i]

# Define W: 19
for i in range(n_drones):
    for r in range(2,n_slot):
        #print(allocation5[r-1][i] <= UB*allocation4[r][i])
        model += allocation5[r-1][i] <= UB*allocation4[r][i]

# Define W: 20


#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^OCT25
#for i in range(n_drones):
 #   for r in range(1,n_slot+1):
        #print(allocation5[r-1][i] >= 0)
  #      model += allocation5[r-1][i] >= 0

# Define W: 21 : means that Z_(2,i) cannot be 1!!! Check if this is OK!
# Shoul dwork even without this cause Z11 = Z21 = 0 always
for i in range(n_drones):
    #print(allocation5[0][i] == 0 )
    model += allocation5[0][i] == 0

# Constraint 14: drone should go to charging station if the completion time exceeds charge available on drone
# Needs two constraint to inforce Z=0 if Drone charge is still available: *lpSum(1+allocation4[r][i] for r in range(0,a))
# Try to work with Drone_Charge
#### Didn't read the last slot!!! @@@@@@@

for i in range(n_drones):
    #print(allocation3[0][i] - Drone_Charge  >= -B*(1-allocation4[1][i]))
    model += allocation3[0][i] - Drone_Charge  >= -B*(1-allocation4[1][i])

# Constraint 16: drone should go to charging station if the completion time exceeds charge available on drone
# Needs two constraint to inforce Z=0 if Drone charge is still available: *lpSum(1+allocation4[r][i] for r in range(0,a))
# Try to work with Drone_Charge
#### Didn't read the last slot!!! @@@@@@@

for i in range(n_drones):
    for r in range(1,n_slot-1):
        #print(allocation3[r][i] - lpSum(allocation5[b][i] for b in range(0,r)) - Drone_Charge  >= -B*(1-allocation4[r+1][i]))
        model += allocation3[r][i] - lpSum(allocation5[b][i] for b in range(0,r)) - Drone_Charge  >= -B*(1-allocation4[r+1][i])

# Constraint 22 connecting Z to X: X is in objective function but does it prevent Z to be 1 all the time?
for i in range(n_drones):
    for r in range(n_slot):
        #print(allocation1[0][r][i] == allocation4[r][i])
        model += allocation1[0][r][i] == allocation4[r][i]

# Costriant 22 - 1 Continue from HEREEEEE
#Job family 1 constraint: jobs 1,2,3,4  ---- j {2,3,4,5,6,7}

for j in range(1,3):
#for j in family1:
    print(lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) <= intervisit_time)
    model += lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) <= intervisit_time

# Costriant 23 -1

#Job family 1 constraint: jobs 1,2,3,4 non negative
for j in range(1,3):
    print(lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) >= 1)
    model += lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) >= 1

# Costriant 22 - 2

#Job family 2 constraint: jobs 5,6,7
for j in range(4,6):
    print(lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) <= intervisit_time)
    model += lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) <= intervisit_time

# Costriant 23 -2

#Job family 2 constraint: jobs 5,6,7 non negative
for j in range(4,6):
    print(lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) >= 1)
    model += lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) >= 1

# Costriant 22 - 3
#Job family 3 constraint: jobs 1,2,3,4  ---- j {2,3,4,5,6,7}

for j in range(7,8):
#for j in family1:
    print(lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) <= intervisit_time)
    model += lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) <= intervisit_time

# Costriant 23 -3

#Job family 3 constraint: jobs 5,6,7 non negative
for j in range(7,8):
    print(lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) >= 1)
    model += lpSum(allocation6[j+1][r][i] - allocation6[j][r][i] for r in range(n_slot) for i in range(n_drones)) >= 1

#Job family constraint: #Dummy 1
for j in range(n_demandnode):
    for r in range(n_slot):
        for i in range(n_drones):
            #print(allocation6[j][r][i] <= UB * allocation1[j][r][i])
            model += allocation6[j][r][i] <= UB * allocation1[j][r][i]

#Job family constraint: #Dummy 2
for j in range(n_demandnode):
    for r in range(n_slot):
        for i in range(n_drones):
            #print(allocation6[j][r][i] <= allocation3[r][i])
            model += allocation6[j][r][i] <= allocation3[r][i]

#Job family constraint: #Dummy 3
for j in range(n_demandnode):
    for r in range(n_slot):
        for i in range(n_drones):
            #print(allocation6[j][r][i] >= allocation3[r][i] - UB *(1-allocation1[j][r][i]))
            model += allocation6[j][r][i] >= allocation3[r][i] - UB *(1-allocation1[j][r][i])

#Job family constraint: #Dummy 4
for j in range(n_demandnode):
    for r in range(n_slot):
        for i in range(n_drones):
            #print(allocation6[j][r][i] >= 0)
            model += allocation6[j][r][i] >= 0


# Job family 1: Break sequence of the same family 2,4,3,...   @@@@@@@@@@@@@@@@@@@@@@@@@ Correct constraint structure
for j in range(1,3):
    for r in range(n_slot-1):
        for i in range(n_drones):
            print(allocation1[j+1][r+1][i] <= B*(1-allocation1[j][r][i] ))
            model += allocation1[j+1][r+1][i]  <= B*(1-allocation1[j][r][i])

# Job family 2: Break sequence of the same family 2,4,3,... @@@@@@@@@@@@@@@@@@@@@@@@@ Correct constraint structure
for j in range(4,6):
    for r in range(n_slot-1):
        for i in range(n_drones):
            print(allocation1[j+1][r+1][i]  <= B*(1-allocation1[j][r][i]))
            model += allocation1[j+1][r+1][i]  <= B*(1-allocation1[j][r][i] )

# Job family 3: Break sequence of the same family 2,4,3,... @@@@@@@@@@@@@@@@@@@@@@@@@ Correct constraint structure
for j in range(7,8):
    for r in range(n_slot-1):
        for i in range(n_drones):
            print(allocation1[j+1][r+1][i]  <= B*(1-allocation1[j][r][i]))
            model += allocation1[j+1][r+1][i]  <= B*(1-allocation1[j][r][i] )

# Limit the operational time of each drone to 24 hours: @@@@@@@@@@@@@@@@@@@@@@@@@ We can use it for larger problems.
#for i in range(n_drones):
 #   print(lpSum(allocation6[j][r][i] for j in range(n_demandnode) for r in range(n_slot)) <= 24)
  #  model += lpSum(allocation6[j][r][i] for j in range(n_demandnode) for r in range(n_slot)) <= 24

# Following two lines list available solvers
# and we use the threads option to use multiple cores

print(listSolvers(onlyAvailable=True))

cbc_solver = PULP_CBC_CMD(threads=64) # change thereads=4 to the number you need


#model.solve()
#model.solve(PULP_CBC_CMD())
model.solve(cbc_solver)

status = LpStatus[model.status]

print(status)


#import pulp as pl
#solver_list = pl.listSolvers(onlyAvailable=True)
#solver = pl.getSolver('CPLEX_CMD')
#solver = pl.getSolver('GUROBI')

print("Total Cost:", model.objective.value())

# Decision Variables

for v in model.variables():
    try:
        print(v.name,"=", v.value())
    except:
        print("error couldnt find value")