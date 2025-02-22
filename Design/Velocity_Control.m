clear 
close all
clc
%%
Ixx = 1;
Iyy = 1;
Izz = 1;    
I = diag([Ixx Iyy Izz]);
g = 9.8065;
m = 1.5;

wn_E = 0.5;
wn_T = 0.1;
% Attitude controller gain calcs
theta = -20:5:20;
phi = -20:5:20;
C = eye(3);
A = [zeros(3,6);-C zeros(3,3)];
count_map = zeros(length(theta),length(phi));
count = 0;
K_orientation = zeros(3,6,length(phi)*length(theta));
Q = diag([1 1 1 100 100 100]);
R = eye(3);
for i = 1:length(theta)
    for j = 1:length(phi)
        count = count+1;
        count_map(i,j) = count;
        B = [linearize_orientation(theta(i), phi(j)); zeros(3,3)];
        K_orientation(:,:,count) = lqr(A,B,Q,R);
    end
end
K_attitude = [10 2 0.2]; % P-I-D
% Position Controller Design
zeta = 0.707;
w_n_p = 1;
A_ref = [0 1;...
         -2*zeta*w_n_p -w_n_p^2];
B_ref = [0; 1];
Q = 1.*eye(2);
P_lyap = lyap(A_ref, Q);
gamma_x = 1.*eye(2);
gamma_r = 0.05;
gamma_w = 0.1;
%% Simulation
tstep = 0.001;
tf = 10;
t = 0:tstep:tf;
P_des = [0;0;5];

out = sim('Control_Sim');
% Model states
w_real = out.w.signals.values;
Euler_real = out.Euler.signals.values;
Thrust_real = out.Thrust_real.signals.values;
P_real = out.P.signals.values*diag([1 1 -1]);
% References produced by controllers
w_ref = out.w_ref.signals.values;
Euler_ref = [out.phi_theta_des.signals.values(:,1) out.phi_theta_des.signals.values(:,2) zeros(length(t),1)];
%%
h1 = vector_plot(t,P_des'.*ones(length(t),3),P_real,{'North (m)', 'East (m)', 'Down (m)', 'NED Position Review'},[0 450 600 350]);
figure('position', [900 450 600 350])
h2 = plot(t, Thrust_real);
h3 = vector_plot(t,Euler_ref.*180/pi,Euler_real.*180/pi,{'\phi (deg)', '\theta (deg)', '\psi (deg)', 'Euler Angle Review'},[0 25 600 350]);
h4 = vector_plot(t,w_ref,w_real,{'w_x (rad/s)', 'w_y (rad/s)', 'w_z (rad/s)', 'Body Angular Velocity Review'},[900 25 600 350]);
