import greedy
from nl_hexaly2 import hexa
from nl_pyomo_model import nl_pyo
from pyomo_model import lp_pyo
from random_instance import generate
from random_instance import mprint
import ast

def run(city):
    a, b, c, d, e = city
    ins = generate(ndrones=a, condition=b, slot=c, charge=d, itimes=e)
    incumbent = ins2incumbent(ins, a, b, c, d, e)

def ins2incumbent(ins, a, b, c, d, e):
    hexa_data = [a, c, ins[0], ins[4], e, ins[2], ins[1], ins[7]]
    gen_seq = {}
    inc = hexa(hexa_data, gen_seq)
    sequences = []
    for i in inc:
        numbers = i.split()
        for ii in numbers:
            if ii is int:
                print(ii)
        sequences.append(numbers)
    return 0

def incumbent2pyomo(incumbent):
    return None

if __name__ == '__main__':
    fixed = [3, 'fixed', 5, 2, 5]
    SB = [3, 'SB', 10, 0.4, 4]
    SB_RS = [4, 'SB_RS', 15, 0.7, 1]
    SB_RS_LA = [5, 'SB_RS_LA', 15, 2, 5]
    run(SB_RS_LA)