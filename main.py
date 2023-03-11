"""runs the main code"""
import boat_simulator
from LoadWPL import load_wpl



if __name__ == '__main__':
    data_file =  load_wpl('data.txt')
    print(tracks_test[0][0])

sim = Simulator.simulate(data_file)
print("/////////----------------")
