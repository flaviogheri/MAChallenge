"""runs the main code"""
from boat_simulator import Simulator
from boat_simulator2 import local_Simulator

sim = Simulator('data.txt')
#sim = local_Simulator('data.txt')
#sim.simulate()
sim.simulate()






