"""runs the main code"""
# from boat_simulator import Simulator
# from boat_simulator2 import local_Simulator
from boat_simulator_trial import Simulator

# sim = Simulator('Track_02.txt', boat_data_yaml='boat_setup.yaml')
sim = Simulator('waypoints.txt')

#sim = local_Simulator('data.txt')
#sim.simulate()
sim.simulate()

















