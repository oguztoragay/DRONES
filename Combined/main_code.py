import greedy
from nl_hexaly import hexa
from nl_pyomo_model import nl_pyo
from pyomo_model import lp_pyo
from random_instance import generate
from random_instance import mprint

def run(city):
    a, b, c, d, e = city
    ins = generate(ndrones=a, condition=b, slot=c, charge=d, itimes=e)
    incumbent = ins2incumbent(ins, a, b, c, d, e)

def ins2incumbent(ins, a, b, c, d, e):
    hexa_data = [a+1, c, ins[0], ins[4], e, ins[2], ins[1], ins[7]]
    inc = hexa(hexa_data)
    return 0

def incumbent2pyomo(incumbent):
    return None

if __name__ == '__main__':
    fixed = [3, 'fixed', 5, 5, 3]
    SB = [3, 'SB', 7, 0.4, 4]
    SB_RS = [5, 'SB_RS', 8, 0.4, 3]
    SB_RS_LS = [5, 'SB_RS_LS', 20, 5, 5]
    run(fixed)