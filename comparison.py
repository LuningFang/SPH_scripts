#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug  1 14:44:11 2022

@author: luning
"""

import csv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import subprocess

matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'STIXGeneral'
matplotlib.rcParams.update({'font.size': 12})
plt.rc('xtick', labelsize=12)
plt.rc('ytick', labelsize=12)
MARKERSIZE = 2


feature_branch = 'feature_geometry/'
main_branch = 'main/'
#test_name = 'Single_Wheel_Test'
#test_name = 'Cylinder_Drop'
test_name = 'Flexible_Cable'
#test_name = 'Compressibility'


file_type = 'fluid'

for time_step in [0, 5, 10, 15, 20, 49]:

# for time_step in [0, 1, 2, 3, 4, 5]:



    file_a = feature_branch + 'FSI_' + test_name + '/particles/' + file_type + str(time_step) + '.csv'
    file_b = main_branch + 'FSI_' + test_name + '/particles/' + file_type + str(time_step) + '.csv'

    table_a = pd.read_csv(file_a)
    table_b = pd.read_csv(file_b)

    velo_mag_a = table_a["|U|"]
    velo_mag_b = table_b["|U|"]

    pos_x_a = table_a["x"]
    pos_x_b = table_b["x"]

    pos_y_a = table_a["y"]
    pos_y_b = table_b["y"]

    pos_z_a = table_a["z"]
    pos_z_b = table_b["z"]


    num_fluid_particles = table_a["|U|"].shape[0]


    velo_diff = np.linalg.norm(np.array(velo_mag_a) - np.array(velo_mag_b))
    px_diff = np.linalg.norm(pos_x_a - pos_x_b)
    py_diff = np.linalg.norm(pos_y_a - pos_y_b)
    pz_diff = np.linalg.norm(pos_z_a - pos_z_b)


    print("=============== frame = {} =================".format(time_step))
    print("velo mag diff         : 2-norm {:.7e},  avg per particle {:.7e}".format(velo_diff, velo_diff/num_fluid_particles))
    print("pos diff - x direction: 2-norm {:.7e},  avg per particle {:.7e}".format(px_diff, px_diff/num_fluid_particles))
    print("pos diff - y direction: 2-norm {:.7e},  avg per particle {:.7e}".format(py_diff, py_diff/num_fluid_particles))
    print("pos diff - z direction: 2-norm {:.7e},  avg per particle {:.7e}".format(pz_diff, pz_diff/num_fluid_particles))
    print("\n")
    
