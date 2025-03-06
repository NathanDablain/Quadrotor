clear
close all
clc
% This runs the Gyro/prediction loop at 200 Hz, and the KF/accel/mag loop
% at 40 Hz
L = 0.2;
M = -1;
N = 0.9;
Ixx = 1;
Iyy = 1;
Izz = 1;    
t_step_model = 0.001;
t_step_filter = 0.04;
tf = 50;
t_model = 0:t_step_model:tf;
t_filter = 0:t_step_filter:tf;
Q = 1.*eye(4);
R = eye(6);

out = sim('Navigation_Sim');
q_real = out.Quaternion.signals.values;
q_estimated = out.qhat.signals.values;
y = out.y.signals.values;
P = out.P.signals.values;
P(:,:,end)
% w = [t_model' out.w.signals.values]
yhat = out.yhat.signals.values;
ybar = out.ybar.signals.values;
%
figure()
sgtitle('Quaternion Comparison')
for i=1:4
    subplot(2,2,i)
    plot(t_model,q_real(:,i),t_model,q_estimated(:,i))
end
legend('Real','Estimated')
%
figure()
sgtitle('Gravity Vector Comparison')
for i=1:6
    subplot(6,1,i)
    plot(t_model,y(:,i),t_filter,yhat(:,i))
end
legend('Real','Estimated')
%
figure()
sgtitle('Gravity Vector Error')
for i=1:6
    subplot(6,1,i)
    plot(t_filter,ybar(:,i))
end
norm(ybar(:,1:3))
%%
clear
clc
syms P11 P12 P13 P14 P22 P23 P24 P33 P34 P44 q0 q1 q2 q3 wx wy wz real

q = [q0;q1;q2;q3];
g_vec = [1;0;0];
t_vec = 2*cross(q(2:4), g_vec);
g_rot = g_vec + q(1)*t_vec + cross(q(2:4), t_vec)
%%
y_hat = g_rot;
H = [2*q(3) 2*q(4) 2*q(1) 2*q(2);...
 -2*q(2) -2*q(1) 2*q(4) 2*q(3);...
 0 -4*q(2) -4*q(3) 0;...
  0       0      -4*q(3) -4*q(4);...
 2*q(4)  2*q(3)  2*q(2)  2*q(1);...
-2*q(3)  2*q(4) -2*q(1)  2*q(2)];
P = [P11 P12 P13 P14;...
    P12 P22 P23 P24;...
    P13 P23 P33 P34;...
    P14 P24 P34 P44];
simplify(P*H')
%%
ybar = sym('ybar',[6 1]);
L = sym('L',[4 6]);

P = [0.9963    0.1469   -0.0064   -0.0533
    0.1469    0.6042    0.0399   -0.0111
   -0.0064    0.0399    0.3870    0.1360
   -0.0533   -0.0111    0.1360    0.5641];
P.*2
%%
% L = P*H'*inv(R);    
xhat_dot = 0.5.*[0 -wx -wy -wz;...
        wx 0 wz -wy;...
        wy -wz 0 wx;...
        wz wy -wx 0]*q + (1-dot(q,q))*q + L*ybar;
simplify(xhat_dot)
%%
clear
close all
clc

deg = 108;
min_1 = 47;
min_2 = 44524;

(((deg*60)+min_1)*1e5)+min_2;

(50345/6e4)*0.005