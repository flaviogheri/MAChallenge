"""runs the main code"""
import boat_simulator
from LoadWPL import load_wpl



if __name__ == '__main__':
    data_file =  load_wpl('data.txt')
    print(data_file)

sim = boat_similator.simulate(data_file)
print("/////////----------------")
