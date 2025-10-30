#!/usr/bin/env python
# coding: utf-8

# In[1]:


from sympy import *
from sympy import init_printing
init_printing(use_latex=True)
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
plt.rc('font',family='Times New Roman')
# plt.rc('font', family='serif', serif='Times')
plt.rc('text', usetex=True)
plt.rc('xtick', labelsize=12)
plt.rc('ytick', labelsize=12)
plt.rc('axes', labelsize=12)
plt.rc('legend', fontsize=12)


# In[2]:


ASR_concrete_experiment = pd.read_csv('../models/concrete/outputs/asr_experiment.csv')
asr_concrete_temp = pd.read_csv('../models/concrete/outputs/ASR_validation_temperature1.csv')
asr_concrete_rh = pd.read_csv('../models/concrete/outputs/ASR_humidity.csv')
asr_concrete_vexpansion = pd.read_csv('../models/concrete/outputs/Volumetric_strain_exp.csv')
ASR_concrete_rebar2_experiment = pd.read_csv('../models/concrete/outputs/asr_experiment_rebarz.csv')
ASR_concrete_rebar3_experiment = pd.read_csv('../models/concrete/outputs/ASR_experiment_3Rebar.csv')


# In[3]:


# asr_concrete_block_calibration_creep_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_temp2_hum2_out.csv')
# asr_concrete_block_calibration_creep_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_out.csv')
# asr_concrete_block_calibration_creep_refine_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_refine_out.csv')
# asr_concrete_block_calibration_creep_updated_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_calibration_creep_updated_out.csv')
# asr_concrete_block_calibration_creep_updated_refine_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_calibration_creep_updated_refine_out.csv')
# asr_concrete_block_calibration_creep_updated_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_updated_out.csv')
# asr_concrete_block_calibration_creep_time0_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_time0_out.csv')
# asr_concrete_block_calibration_creep_time1_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_time1_out.csv')
# asr_concrete_block_calibration_creep_time2_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_time2_out.csv')
# asr_concrete_block_validation_rebar = pd.read_csv('../models/concrete/outputs/asr_concrete_block_validation_rebar_creep_out.csv')
# asr_concrete_block_validation_rebar_damage = pd.read_csv('../models/concrete/outputs/asr_concrete_block_validation_rebar_creep_damage_out.csv')
# asr_concrete_block_validation_rebar2 = pd.read_csv('../models/concrete/outputs/asr_concrete_block_validation_rebar_creep2_out.csv')
# asr_concrete_block_validation_rebar3_creep_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_validation_rebar3_creep_out.csv')
# asr_concrete_block_validation_rebar_temp1 = pd.read_csv('../models/concrete/outputs/asr_concrete_rebar_temp1.csv')
# asr_concrete_block_validation_rebar_temp2 = pd.read_csv('../models/concrete/outputs/asr_concrete_rebar_temp2.csv')
#
# asr_concrete_block_calibration_creep_updated_out['disp_x_p1_pos']
#
#
# # In[4]:
#
#
# width = 50
# height = 50
#
# ax = plt.figure(1)
#
# # plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['strain_xx']*100, 'r', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'b', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_xx']*100, 'r', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['strain_xx']*100, 'g', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# # plt.savefig("conc_calibration.pdf", bbox_inches='tight')
# # plt.show()
#
# ax = plt.figure(2)
#
# plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['vstrain']*100, 'r',label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_vexpansion['time'], asr_concrete_vexpansion['Vexpansion'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Volumetric Strain (%)')
# plt.legend(frameon=False)
# #plt.savefig("conc_calibration_vstrain.pdf", bbox_inches='tight')
# #plt.show()
#
# ax = plt.figure(3)
#
# # plt.plot(asr_concrete_rh['time_sec']/86400, asr_concrete_rh['rh']*100, 'k*',label='Experiment', markersize = 5)
# plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity']*100, 'r-',label='Average response from simulation', markersize = 5)
# plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity_bc']*100, 'b+',label='Experimental conditions applied on the boundaries', markersize = 5)
# plt.xlim(1,500)
# # plt.ylim(0,100)
# plt.xlabel('Time (days)')
# plt.ylabel('Relative Humidity (%)')
# plt.legend(frameon=False)
# #plt.savefig("conc_calibration_rh.pdf", bbox_inches='tight')
# # plt.show()
#
# ax = plt.figure(4)
#
# # plt.plot(asr_concrete_rh['time_sec']/86400, asr_concrete_rh['rh']*100, 'k*',label='Experiment', markersize = 5)
# plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['temp'], 'r-',label='Average response from simulation', markersize = 5)
# plt.plot(asr_concrete_temp['time']/86400, asr_concrete_temp['temp'], 'b+',label='Experimental conditions ', markersize = 5)
# plt.xlim(1,500)
# plt.xlabel('Time (days)')
# plt.ylabel('Temperature ($^o$C)')
# plt.legend(frameon=False)
# #plt.savefig("conc_calibration_temp.pdf", bbox_inches='tight')
# # plt.show()
#
#
# # In[5]:
#
# asr_concrete_block_calibration_creep_updated_strainxx = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_101']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_102'])/0.48
# asr_concrete_block_calibration_creep_updated_strainyy = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_103']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_106'])/0.48
# asr_concrete_block_calibration_creep_updated_strainzz = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_105']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_104'])/0.48
#
# ax = plt.figure(5)
#
# # plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['strain_xx']*100, 'r', label='Simulation', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'b', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['disp_x_101'], 'r', label='101', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['disp_x_102'], 'r', label='102', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['disp_z_103'], 'g', label='103', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['disp_z_104'], 'g', label='104', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['disp_y_105'], 'b', label='105', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['disp_y_106'], 'b', label='106', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['strain_xx']*100, 'g', label='Simulation', markersize = 5, linewidth=3)
# # plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# # plt.savefig("conc_calibration.pdf", bbox_inches='tight')
# # plt.show()
#
# asr_concrete_block_calibration_creep_updated_dispx_p1 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p1_neg'])
# asr_concrete_block_calibration_creep_updated_dispx_p2 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p2_neg'])
# asr_concrete_block_calibration_creep_updated_dispx_p3 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p3_neg'])
# asr_concrete_block_calibration_creep_updated_dispx_p4 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p4_neg'])
# asr_concrete_block_calibration_creep_updated_dispx_p5 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p5_neg'])
# asr_concrete_block_calibration_creep_updated_dispx_p6 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p6_neg'])
#
#
# ax = plt.figure(6)
#
# # plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['strain_xx']*100, 'r', label='Simulation', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'b', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_strainxx*100, 'r', label='101', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_strainyy*100, 'b', label='102', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_strainzz*100, 'g', label='103', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_xx']*100, 'r--', label='104', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_yy']*100, 'b--', label='105', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_zz']*100, 'g--', label='106', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['strain_xx']*100, 'g', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# # plt.savefig("conc_calibration.pdf", bbox_inches='tight')
# plt.show()
#
# ax = plt.figure(7)
#
# # plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['strain_xx']*100, 'r', label='Simulation', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'b', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_dispx_p1, 'r', label='101', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_dispx_p2, 'b', label='102', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_dispx_p3, 'g', label='103', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_dispx_p4, 'r--', label='104', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_dispx_p5, 'b--', label='105', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_dispx_p6, 'g--', label='106', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['strain_xx']*100, 'g', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# # plt.savefig("conc_calibration.pdf", bbox_inches='tight')
# plt.show()
#
#
#
#
# width = 50
# height = 50
#
# ax = plt.figure(10)
# plt.plot(asr_concrete_block_validation_rebar['time']/86400, asr_concrete_block_validation_rebar['strain_xx']*100, 'r-', label='$\epsilon_{xx}$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar['time']/86400, asr_concrete_block_validation_rebar['strain_yy']*100, 'b-',label='$\epsilon_{yy}$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar['time']/86400, asr_concrete_block_validation_rebar['strain_zz']*100, 'g-',label='$\epsilon_{zz}$', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_rebar2_experiment['x'], ASR_concrete_rebar2_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# plt.savefig("uniaxial_rebar.pdf", bbox_inches='tight')
# #plt.show()
#
# ax = plt.figure(9)
# plt.plot(asr_concrete_block_validation_rebar2['time']/86400, asr_concrete_block_validation_rebar2['strain_xx']*100, 'r-', label='$\epsilon_{xx}$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar2['time']/86400, asr_concrete_block_validation_rebar2['strain_yy']*100, 'b-',label='$\epsilon_{yy}$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar2['time']/86400, asr_concrete_block_validation_rebar2['strain_zz']*100, 'g-',label='$\epsilon_{zz}$', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_rebar2_experiment['x'], ASR_concrete_rebar2_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# # plt.savefig("uniaxial_rebar.pdf", bbox_inches='tight')
# #plt.show()
#
# width = 50
# height = 50
#
# ax = plt.figure(11)
#
# plt.plot(asr_concrete_block_validation_rebar3_creep_out['time']/86400, asr_concrete_block_validation_rebar3_creep_out['strain_xx']*100, 'r-', label='$\epsilon_{xx}$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar3_creep_out['time']/86400, asr_concrete_block_validation_rebar3_creep_out['strain_yy']*100, 'b-',label='$\epsilon_{yy}$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar3_creep_out['time']/86400, asr_concrete_block_validation_rebar3_creep_out['strain_zz']*100, 'g-',label='$\epsilon_{zz}$', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_rebar3_experiment['x'], ASR_concrete_rebar3_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# plt.savefig("rebar_model.pdf", bbox_inches='tight')
# #plt.show()
#
#
# # In[6]:
#
#
# width = 50
# height = 50
#
# ax = plt.figure(12)
# plt.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'r', label='Set-1', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_time1_out['time']/86400, asr_concrete_block_calibration_creep_time1_out['strain_xx']*100, 'b', label='Set-2', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_time2_out['time']/86400, asr_concrete_block_calibration_creep_time2_out['strain_xx']*100, 'g', label='set-3', markersize = 5, linewidth=3)
# plt.ylim(0,)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# # plt.show()
# plt.savefig("plain_conc_sets.pdf", bbox_inches='tight')
#
#
# width = 50
# height = 50
#
# ax = plt.figure(13)
# plt.plot(asr_concrete_block_validation_rebar['time']/86400, asr_concrete_block_validation_rebar['vstrain']*100, 'r-', label='$T_0 = 41 ^\circ C$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar_temp1['time']/86400, asr_concrete_block_validation_rebar_temp1['vstrain']*100, 'b-', label='$T_0 = 35 ^\circ C$', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_validation_rebar_temp2['time']/86400, asr_concrete_block_validation_rebar_temp2['vstrain']*100, 'g-', label='$T_0 = 20 ^\circ C$', markersize = 5, linewidth=3)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# plt.savefig("rebar_temp.pdf", bbox_inches='tight')
# #plt.show()
#
# ax = plt.figure(14)
# plt.plot(asr_concrete_block_validation_rebar_temp2['time']/86400, asr_concrete_block_validation_rebar_temp2['humidity']*100, 'r+--',label='$T_0 = 35 ^\circ C$', markersize = 5)
# plt.plot(asr_concrete_block_validation_rebar_temp1['time']/86400, asr_concrete_block_validation_rebar_temp1['humidity']*100, 'g+--',label='$T_0 = 20 ^\circ C$', markersize = 5)
#
# plt.xlabel('Time (days)')
# plt.ylabel('Relative Humidity (%)')
# plt.legend(frameon=False)
# #plt.show()
#
#
# fig, (ax1, ax2, ax3) = plt.subplots(3, figsize=(6, 9), gridspec_kw={'height_ratios': [1, 1, 2]})
#
# ax1.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['temp'], 'r-',label='Average response from simulation', markersize = 5)
# ax1.plot(asr_concrete_temp['time']/86400, asr_concrete_temp['temp'], 'b+',label='Experimental conditions ', markersize = 5)
# ax1.set_xlim(1,)
# ax1.set_ylim(0,)
# ax1.set_ylabel('Temperature ($^o$C)')
#
# ax2.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity']*100, 'r-', markersize = 5)
# ax2.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity_bc']*100, 'b+', markersize = 5)
# ax2.set_xlim(1,)
# ax2.set_ylim(0,)
# ax2.set_ylabel('Relative Humidity (%)')
#
# ax3.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'm-', markersize = 5, linewidth=2)
# ax3.plot(asr_concrete_block_calibration_creep_time1_out['time']/86400, asr_concrete_block_calibration_creep_time1_out['strain_xx']*100, 'k-', markersize = 5, linewidth=2)
# ax3.plot(asr_concrete_block_calibration_creep_time2_out['time']/86400, asr_concrete_block_calibration_creep_time2_out['strain_xx']*100, 'g-', markersize = 5, linewidth=2)
# ax3.set_ylabel('Axial Expansion (%)')
# ax3.set_ylim(0,)
# plt.xlabel('Time (days)')
#
# fig.tight_layout()
# ax1.legend(bbox_to_anchor=(0.5, -0.05), loc='lower center', frameon=False )
# # plt.show()
# fig.savefig("environment_effect.pdf", bbox_inches='tight')
#
# fig, (ax1, ax2) = plt.subplots(2, figsize=(7, 6))
#
# #Drop points up to approximately 28 days
# asr_concrete_block_calibration_creep_out = asr_concrete_block_calibration_creep_out.iloc[50:]
# ax1.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['temp'], 'r-',label='Average response from simulation', markersize = 5, linewidth=3)
# ax1.plot(asr_concrete_temp['time']/86400, asr_concrete_temp['temp'], 'b+',label='Experimental conditions ', markersize = 6, markeredgewidth=2)
# ax1.set_xlim(0,)
# ax1.set_ylim(0,)
# ax1.set_ylabel('Temperature ($^o$C)')

# ax2.plot(asr_concrete_rh['time_sec']/86400, asr_concrete_rh['rh']*100, 'b+',label='Experimental conditions', markersize = 6, markeredgewidth=2)
# ax2.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity']*100, 'r-', label='Average response from simulation', markersize = 5, linewidth=3)
# # ax2.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity_bc']*100, 'b+', label='Experimental conditions', markersize = 5)
# ax2.set_xlim(0,)
# ax2.set_ylim(0,)
# ax2.set_ylabel('Relative Humidity (%)')
#
# plt.xlabel('Time (days)')
# fig.tight_layout()
#
# ax1.legend(bbox_to_anchor=(0.5, 0), loc='lower center', frameon=False )
#
# # plt.show()
# fig.savefig("temp_rh_history.pdf", bbox_inches='tight')
#

asr_concrete_block_calibration_creep_updated_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_updated_out.csv')
asr_concrete_block_calibration_creep_updated_refine_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_updated_refine_out.csv')
asr_concrete_block_calibration_creep_updated_refine_time1_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_updated_refine_time1_out.csv')
asr_concrete_block_calibration_creep_updated_refine_time2_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_calibration_creep_updated_refine_time2_out.csv')
asr_concrete_block_validation_rebar_creep_damage_updated_out = pd.read_csv('../models/concrete/outputs/asr_concrete_block_validation_rebar_creep_damage_updated_refine_out.csv')

# asr_concrete_block_calibration_creep_updated_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_calibration_creep_updated_out.csv')
# asr_concrete_block_calibration_creep_updated_refine_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_calibration_creep_updated_refine_out.csv')
# asr_concrete_block_calibration_creep_updated_refine_time1_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_calibration_creep_updated_refine_time1_out.csv')
# asr_concrete_block_calibration_creep_updated_refine_time2_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_calibration_creep_updated_refine_time2_out.csv')
# asr_concrete_block_validation_rebar_creep_damage_updated_out = pd.read_csv('/Users/bisws/programs/blackbear/test/tests/concrete_ASR_swelling/asr_concrete_block_validation_rebar_creep_damage_updated_refine_out.csv')

asr_concrete_block_calibration_creep_updated_strainxx = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_101']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_102'])
asr_concrete_block_calibration_creep_updated_strainyy = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_103']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_104'])
asr_concrete_block_calibration_creep_updated_strainzz = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_105']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_106'])




# ax = plt.figure(6)
#
# # plt.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['strain_xx']*100, 'r', label='Simulation', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_time0_out['time']/86400, asr_concrete_block_calibration_creep_time0_out['strain_xx']*100, 'b', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_strainxx*100, 'r', label='101', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_strainyy*100, 'b', label='102', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_strainzz*100, 'g', label='103', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_xx']*100, 'r--', label='104', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_yy']*100, 'b--', label='105', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_out['time']/86400, asr_concrete_block_calibration_creep_updated_out['strain_zz']*100, 'g--', label='106', markersize = 5, linewidth=3)
# # plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['strain_xx']*100, 'g', label='Simulation', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (%)')
# plt.legend(frameon=False)
# plt.savefig("conc_calibration.pdf", bbox_inches='tight')
# plt.show()

asr_concrete_block_calibration_creep_updated_dispx_p1 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p1_neg'])
asr_concrete_block_calibration_creep_updated_dispx_p2 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p2_neg'])
asr_concrete_block_calibration_creep_updated_dispx_p3 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p3_neg'])
asr_concrete_block_calibration_creep_updated_dispx_p4 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p4_neg'])
asr_concrete_block_calibration_creep_updated_dispx_p5 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p5_neg'])
asr_concrete_block_calibration_creep_updated_dispx_p6 = abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_x_p6_neg'])

asr_concrete_block_calibration_creep_updated_dispy_p1 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p1_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p2 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p2_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p3 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p3_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p4 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p4_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p5 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p5_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p6 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p6_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p7 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p7_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p7_neg'])
asr_concrete_block_calibration_creep_updated_dispy_p8 = abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p8_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_y_p8_neg'])

asr_concrete_block_calibration_creep_updated_dispz_p1 = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p1_neg'])
asr_concrete_block_calibration_creep_updated_dispz_p2 = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p2_neg'])
asr_concrete_block_calibration_creep_updated_dispz_p3 = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p3_neg'])
asr_concrete_block_calibration_creep_updated_dispz_p4 = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p4_neg'])
asr_concrete_block_calibration_creep_updated_dispz_p5 = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p5_neg'])
asr_concrete_block_calibration_creep_updated_dispz_p6 = abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_out['disp_z_p6_neg'])

asr_concrete_block_calibration_creep_updated_refine_dispx_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispx_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispx_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispx_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispx_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispx_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_x_p6_neg'])

asr_concrete_block_calibration_creep_updated_refine_dispy_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p6_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p7 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p7_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p7_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispy_p8 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p8_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_y_p8_neg'])

asr_concrete_block_calibration_creep_updated_refine_dispz_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispz_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispz_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispz_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispz_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_dispz_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_out['disp_z_p6_neg'])


asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p1 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p1_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p1_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p2 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p2_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p2_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p3 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p3_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p3_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p4 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p4_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p4_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p5 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p5_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p5_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p6 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p6_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_x_p6_neg'])

asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p1 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p1_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p1_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p2 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p2_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p2_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p3 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p3_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p3_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p4 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p4_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p4_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p5 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p5_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p5_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p6 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p6_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p6_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p7 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p7_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p7_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p8 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p8_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_y_p8_neg'])

asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p1 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p1_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p1_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p2 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p2_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p2_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p3 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p3_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p3_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p4 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p4_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p4_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p5 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p5_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p5_neg'])
asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p6 = abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p6_pos']) + abs(asr_concrete_block_validation_rebar_creep_damage_updated_out['disp_z_p6_neg'])


asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_x_p6_neg'])

asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p6_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p7 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p7_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p7_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p8 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p8_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_y_p8_neg'])

asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time1_out['disp_z_p6_neg'])


asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_x_p6_neg'])

asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p6_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p7 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p7_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p7_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p8 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p8_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_y_p8_neg'])

asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p1 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p1_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p1_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p2 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p2_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p2_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p3 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p3_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p3_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p4 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p4_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p4_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p5 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p5_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p5_neg'])
asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p6 = abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p6_pos']) + abs(asr_concrete_block_calibration_creep_updated_refine_time2_out['disp_z_p6_neg'])

asr_concrete_block_calibration_creep_updated_strain_xx = pd.DataFrame([asr_concrete_block_calibration_creep_updated_dispx_p1, asr_concrete_block_calibration_creep_updated_dispx_p2, asr_concrete_block_calibration_creep_updated_dispx_p3, asr_concrete_block_calibration_creep_updated_dispx_p4, asr_concrete_block_calibration_creep_updated_dispx_p5, asr_concrete_block_calibration_creep_updated_dispx_p6])
asr_concrete_block_calibration_creep_updated_strain_yy = pd.DataFrame([asr_concrete_block_calibration_creep_updated_dispy_p1, asr_concrete_block_calibration_creep_updated_dispy_p2, asr_concrete_block_calibration_creep_updated_dispy_p3, asr_concrete_block_calibration_creep_updated_dispy_p4, asr_concrete_block_calibration_creep_updated_dispy_p5, asr_concrete_block_calibration_creep_updated_dispy_p6])
asr_concrete_block_calibration_creep_updated_strain_zz = pd.DataFrame([asr_concrete_block_calibration_creep_updated_dispz_p1, asr_concrete_block_calibration_creep_updated_dispz_p2, asr_concrete_block_calibration_creep_updated_dispz_p3, asr_concrete_block_calibration_creep_updated_dispz_p4, asr_concrete_block_calibration_creep_updated_dispz_p5, asr_concrete_block_calibration_creep_updated_dispz_p6])

asr_concrete_block_calibration_creep_updated_strain_xx_mean = asr_concrete_block_calibration_creep_updated_strain_xx.mean()
asr_concrete_block_calibration_creep_updated_strain_yy_mean = asr_concrete_block_calibration_creep_updated_strain_yy.mean()
asr_concrete_block_calibration_creep_updated_strain_zz_mean = asr_concrete_block_calibration_creep_updated_strain_zz.mean()

asr_concrete_block_calibration_creep_updated_refine_strain_xx = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_dispx_p1, asr_concrete_block_calibration_creep_updated_refine_dispx_p2, asr_concrete_block_calibration_creep_updated_refine_dispx_p3, asr_concrete_block_calibration_creep_updated_refine_dispx_p4, asr_concrete_block_calibration_creep_updated_refine_dispx_p5, asr_concrete_block_calibration_creep_updated_refine_dispx_p6])
asr_concrete_block_calibration_creep_updated_refine_strain_yy = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_dispy_p1, asr_concrete_block_calibration_creep_updated_refine_dispy_p2, asr_concrete_block_calibration_creep_updated_refine_dispy_p3, asr_concrete_block_calibration_creep_updated_refine_dispy_p4, asr_concrete_block_calibration_creep_updated_refine_dispy_p5, asr_concrete_block_calibration_creep_updated_refine_dispy_p6])
asr_concrete_block_calibration_creep_updated_refine_strain_zz = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_dispz_p1, asr_concrete_block_calibration_creep_updated_refine_dispz_p2, asr_concrete_block_calibration_creep_updated_refine_dispz_p3, asr_concrete_block_calibration_creep_updated_refine_dispz_p4, asr_concrete_block_calibration_creep_updated_refine_dispz_p5, asr_concrete_block_calibration_creep_updated_refine_dispz_p6])

asr_concrete_block_calibration_creep_updated_refine_strain_xx_mean = asr_concrete_block_calibration_creep_updated_refine_strain_xx.mean()
asr_concrete_block_calibration_creep_updated_refine_strain_yy_mean = asr_concrete_block_calibration_creep_updated_refine_strain_yy.mean()
asr_concrete_block_calibration_creep_updated_refine_strain_zz_mean = asr_concrete_block_calibration_creep_updated_refine_strain_zz.mean()


asr_concrete_block_calibration_creep_updated_refine_time1_strain_xx = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p1, asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p2, asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p3, asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p4, asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p5, asr_concrete_block_calibration_creep_updated_refine_time1_dispx_p6])
asr_concrete_block_calibration_creep_updated_refine_time1_strain_yy = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p1, asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p2, asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p3, asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p4, asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p5, asr_concrete_block_calibration_creep_updated_refine_time1_dispy_p6])
asr_concrete_block_calibration_creep_updated_refine_time1_strain_zz = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p1, asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p2, asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p3, asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p4, asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p5, asr_concrete_block_calibration_creep_updated_refine_time1_dispz_p6])


asr_concrete_block_calibration_creep_updated_refine_time1_strain_xx_mean = asr_concrete_block_calibration_creep_updated_refine_time1_strain_xx.mean()
asr_concrete_block_calibration_creep_updated_refine_time1_strain_yy_mean = asr_concrete_block_calibration_creep_updated_refine_time1_strain_yy.mean()
asr_concrete_block_calibration_creep_updated_refine_time1_strain_zz_mean = asr_concrete_block_calibration_creep_updated_refine_time1_strain_zz.mean()

asr_concrete_block_calibration_creep_updated_refine_time2_strain_xx = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p1, asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p2, asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p3, asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p4, asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p5, asr_concrete_block_calibration_creep_updated_refine_time2_dispx_p6])
asr_concrete_block_calibration_creep_updated_refine_time2_strain_yy = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p1, asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p2, asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p3, asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p4, asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p5, asr_concrete_block_calibration_creep_updated_refine_time2_dispy_p6])
asr_concrete_block_calibration_creep_updated_refine_time2_strain_zz = pd.DataFrame([asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p1, asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p2, asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p3, asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p4, asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p5, asr_concrete_block_calibration_creep_updated_refine_time2_dispz_p6])


asr_concrete_block_calibration_creep_updated_refine_time2_strain_xx_mean = asr_concrete_block_calibration_creep_updated_refine_time2_strain_xx.mean()
asr_concrete_block_calibration_creep_updated_refine_time2_strain_yy_mean = asr_concrete_block_calibration_creep_updated_refine_time2_strain_yy.mean()
asr_concrete_block_calibration_creep_updated_refine_time2_strain_zz_mean = asr_concrete_block_calibration_creep_updated_refine_time2_strain_zz.mean()



asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_xx = pd.DataFrame([asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p1, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p2, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p3, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p4, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p5, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispx_p6])
asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_yy = pd.DataFrame([asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p1, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p2, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p3, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p4, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p5, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispy_p6])
asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_zz = pd.DataFrame([asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p1, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p2, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p3, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p4, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p5, asr_concrete_block_validation_rebar_creep_damage_updated_out_dispz_p6])



asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_xx_mean = asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_xx.mean()
asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_yy_mean = asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_yy.mean()
asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_zz_mean = asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_zz.mean()


# fig, (ax1, ax2) = plt.subplots(2, figsize=(7, 6))
#
# #Drop points up to approximately 28 days
# # asr_concrete_block_validation_rebar_creep_damage_updated_out = asr_concrete_block_validation_rebar_creep_damage_updated_out.iloc[1:]
# ax1.plot(asr_concrete_block_validation_rebar_creep_damage_updated_out['time']/86400, asr_concrete_block_validation_rebar_creep_damage_updated_out['temp'], 'r-',label='Average response from simulation', markersize = 5, linewidth=3)
# ax1.plot(asr_concrete_temp['time']/86400, asr_concrete_temp['temp'], 'b+',label='Experimental conditions ', markersize = 6, markeredgewidth=2)
# ax1.set_xlim(0,)
# ax1.set_ylim(0,)
# ax1.set_ylabel('Temperature ($^o$C)')
#
# ax2.plot(asr_concrete_rh['time_sec']/86400, asr_concrete_rh['rh']*100, 'b+',label='Experimental conditions', markersize = 6, markeredgewidth=2)
# ax2.plot(asr_concrete_block_validation_rebar_creep_damage_updated_out['time']/86400, asr_concrete_block_validation_rebar_creep_damage_updated_out['humidity']*100, 'r-', label='Average response from simulation', markersize = 5, linewidth=3)
# # ax2.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity_bc']*100, 'b+', label='Experimental conditions', markersize = 5)
# ax2.set_xlim(0,)
# ax2.set_ylim(0,)
# ax2.set_ylabel('Relative Humidity (%)')
#
# plt.xlabel('Time (days)')
# fig.tight_layout()
#
# ax1.legend(bbox_to_anchor=(0.5, 0), loc='lower center', frameon=False )
#
# # plt.show()
# fig.savefig("temp_rh_history2.pdf", bbox_inches='tight')

ax = plt.figure(10)
plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_xx_mean/0.48*100, 'r-', label='$\epsilon_{xx}$', markersize = 5, linewidth=3)
plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_yy_mean/0.48*100, 'b-',label='$\epsilon_{yy}$', markersize = 5, linewidth=3)
plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_zz_mean/0.48*100, 'g-',label='$\epsilon_{zz}$', markersize = 5, linewidth=3)
plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.plot(ASR_concrete_rebar2_experiment['x'], ASR_concrete_rebar2_experiment['y'], 'k*',label='Experiment', markersize = 5)
plt.xlabel('Time (days)')
plt.ylabel('Axial Expansion (\%)')
plt.legend(frameon=False)
plt.savefig("conc_calibration3.pdf", bbox_inches='tight')
plt.show()



# ax = plt.figure(10)
# plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_xx_mean, 'r', label='$\epsilon_{xx}', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_yy_mean, 'g', label='$\epsilon_{yy}', markersize = 5, linewidth=3)
# plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_zz_mean, 'b', label='$\epsilon_{zz}', markersize = 5, linewidth=3)
# plt.plot(ASR_concrete_experiment['x'], ASR_concrete_experiment['y'], 'k*',label='Experiment', markersize = 5)
# plt.xlabel('Time (days)')
# plt.ylabel('Axial Expansion (\%)')
# plt.legend(frameon=False)
# # plt.savefig("conc_calibration.pdf", bbox_inches='tight')
# plt.show()

width = 50
height = 50

ax = plt.figure(11)
plt.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_strain_xx_mean/0.48*100, 'r', label='Set-1', markersize = 5, linewidth=3)
plt.plot(asr_concrete_block_calibration_creep_updated_refine_time1_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_time1_strain_xx_mean/0.48*100, 'b', label='Set-2', markersize = 5, linewidth=3)
plt.plot(asr_concrete_block_calibration_creep_updated_refine_time2_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_time2_strain_xx_mean/0.48*100, 'g', label='set-3', markersize = 5, linewidth=3)
plt.ylim(0,)
plt.xlabel('Time (days)')
plt.ylabel('Axial Expansion (\%)')
plt.legend(frameon=False)
# plt.show()
plt.savefig("plain_conc_sets3.pdf", bbox_inches='tight')

ax = plt.figure(12)
plt.plot(asr_concrete_block_validation_rebar_creep_damage_updated_out['time']/86400, asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_xx_mean/0.48*100, 'r-', label='$\epsilon_{xx}$', markersize = 5, linewidth=3)
plt.plot(asr_concrete_block_validation_rebar_creep_damage_updated_out['time']/86400, asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_yy_mean/0.48*100, 'b-',label='$\epsilon_{yy}$', markersize = 5, linewidth=3)
plt.plot(asr_concrete_block_validation_rebar_creep_damage_updated_out['time']/86400, asr_concrete_block_validation_rebar_creep_damage_updated_out_strain_zz_mean/0.48*100, 'g-',label='$\epsilon_{zz}$', markersize = 5, linewidth=3)
plt.plot(ASR_concrete_rebar2_experiment['x'], ASR_concrete_rebar2_experiment['y'], 'k*',label='Experiment', markersize = 5)
plt.xlabel('Time (days)')
plt.ylabel('Axial Expansion (\%)')
plt.legend(frameon=False)
plt.savefig("uniaxial_rebar3.pdf", bbox_inches='tight')
plt.show()

asr_concrete_block_calibration_creep_updated_refine_out = asr_concrete_block_calibration_creep_updated_refine_out.iloc[1:]

fig, (ax1, ax2) = plt.subplots(2, figsize=(7, 6))
#Drop points up to approximately 28 days
ax1.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['temp'], 'r-',label='Average response from simulation', markersize = 5, linewidth=3)
ax1.plot(asr_concrete_temp['time']/86400, asr_concrete_temp['temp'], 'b+',label='Experimental conditions ', markersize = 6, markeredgewidth=2)
ax1.set_xlim(0,)
ax1.set_ylim(0,)
ax1.set_ylabel('Temperature ($^o$C)')

ax2.plot(asr_concrete_rh['time_sec']/86400, asr_concrete_rh['rh']*100, 'b+',label='Experimental conditions', markersize = 6, markeredgewidth=2)
ax2.plot(asr_concrete_block_calibration_creep_updated_refine_out['time']/86400, asr_concrete_block_calibration_creep_updated_refine_out['humidity']*100, 'r-', label='Average response from simulation', markersize = 5, linewidth=3)
# ax2.plot(asr_concrete_block_calibration_creep_out['time']/86400, asr_concrete_block_calibration_creep_out['humidity_bc']*100, 'b+', label='Experimental conditions', markersize = 5)
ax2.set_xlim(0,)
ax2.set_ylim(0,)
ax2.set_ylabel('Relative Humidity (\%)')

plt.xlabel('Time (days)')
fig.tight_layout()

ax1.legend(bbox_to_anchor=(0.5, 0), loc='lower center', frameon=False )

# plt.show()
fig.savefig("temp_rh_history3.pdf", bbox_inches='tight')
