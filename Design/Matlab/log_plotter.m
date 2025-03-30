clear
clc
close all

addpath('C:\Users\Nathan Dablain\OneDrive\Documents\Quad Mark 3\Quadrotor\Design\Simulation');

Sim_log = readtable('Sim_log.txt');
Sim_data = table2array(Sim_log);
MCU_log = readtable('MCU_log.txt');
MCU_data = table2array(MCU_log);

Sim_t = Sim_data(:,1);
Sim_position_NED = Sim_data(:,2:4);
Sim_pressure = Sim_data(:,5);
Sim_Euler = Sim_data(:,6:8);
Sim_w = Sim_data(:,9:11);
Sim_v = Sim_data(:,12:14);

MCU_t = MCU_data(:,1);
MCU_position_NED = MCU_data(:,2:4);
MCU_pressure = MCU_data(:,5);
MCU_Euler = MCU_data(:,6:8);
MCU_w = MCU_data(:,9:11);
MCU_v = MCU_data(:,12:14);
MCU_w_ref = MCU_data(:,18:20);
MCU_Euler_ref = MCU_data(:,15:17);

h1 = vector_plot(Sim_t,Sim_position_NED,MCU_position_NED,{'North (m)', 'East (m)', 'Down (m)', 'NED Position Review'},[0 450 600 350],[-10 10]);
h2 = reference_plot(Sim_t,Sim_Euler,MCU_Euler,MCU_Euler_ref,{'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)', 'Euler Angles Review'},[900 450 600 350],[-2*pi 2*pi]);
h3 = reference_plot(Sim_t,Sim_w,MCU_w,MCU_w_ref,{'w_x (rad/s)', 'w_y (rad/s)', 'w_z (rad/s)', 'Body Angular Rate Review'},[0 25 600 350],[-500 500].*pi/180);
h4 = vector_plot(Sim_t,Sim_v,MCU_v,{'v_x (m/s)', 'v_y (m/s)', 'v_z (m/s)', 'Body Linear Velocity Review'},[900 25 600 350],[-50 50]);
% figure('position',[900 25 600 350]);
% plot(Sim_t, Sim_pressure, MCU_t, MCU_pressure, 'k--')
% ylim([95000 105000])
% legend('Simulation', 'MCU')