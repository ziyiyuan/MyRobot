clc
clear all
close all

M = 0.5;
b = 0.1; g = 9.81; m = 1; r = 0.1;
% tau = M * qdd + mgr cos(q)
Kd = 2;
Kp = 2.205;
Ki = 1;
% 初始值
q0 = -pi/2;
qd0 = 0;
% 目标值
qt = 0;
qtd = 0;
figure
i = 0:0.1:10


%% PD Control
syms y(t)
eqn = M * diff(y, t, 2) + (Kd + b) * diff(y, t) + Kp * y == m * g * r * cos(qd0);
Dy = diff(y,t);
cond = [y(0)==qd0 - q0, Dy(0)==0];
y1(t) = dsolve(eqn,cond)
plot(i,eval(y1(i)), 'r', 'LineWidth',3)

%% PID Control
Ki = 1;
syms y(t)
eqn = M * diff(y, t, 3) + (Kd + b) * diff(y, t, 2) + Kp * diff(y, t)  + Ki * y== 0;
Dy = diff(y,t);
Ddy = diff(y,t,2);
cond = [y(0)==qd0 - q0, Dy(0)==0, Ddy(0)==0];
y2(t) = dsolve(eqn,cond)

hold on
plot(i,eval(y2(i)), 'b', 'LineWidth',3)

xlabel('t')
ylabel('qe')
legend('PD','PID')