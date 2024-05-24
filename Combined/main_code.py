import greedy
import nl_hexaly
import nl_pyomo_model
import pyomo_model
from random_instance import generate
from random_instance import mprint

def run():
    nb_drones = 5
    city = 'SB'
    ins = generate(ndrones=nb_drones, condition=city)
    incumbent = ins2incumbent(ins)

def ins2incumbent(ins):
    nb_drones = ins[2]
    nb_slots = ins[3]
    t_matrix = ins[4]
    return None

def incumbent2pyomo(incumbent):
    return None

if __name__ == '__main__':
    print("jjjjjjjjjjjjjjjjjj")
    run()