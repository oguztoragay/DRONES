from pyomo.environ import *

# Create a concrete model
model = ConcreteModel()

# Define a variable with a name
model.x = Var(name='x_variable')

# You can also define multiple variables with names using the VarList function
# model.y = VarList(range(1, 5), name='y_variable')

# Access the variables and their names
print("Variable x name:", model.x.name)
# for i, var in enumerate(model.y):
#     print(f"Variable y[{i}] name:", var.name)
