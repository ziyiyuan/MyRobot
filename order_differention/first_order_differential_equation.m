% 一阶微分方程求解

clc
clear all ; close all
% syms r(t) K0 Pext
% K0 = 100;
% Pext = 10;
% 
% figure(1)
% y(t) = dsolve(diff(r(t),t) == K0*(Pext - r(t)), r(0) == 0)
% fplot(y,[0,1])

%% total energy observer
syms Kd P  d(t)
% Kd = 10000; %越大响应越快
% P = 1;
% a = [1];
% b = [-Kd];
% c = [Kd* P];
% sys = ss(a,b,c,0);
% impulse(sys)

y(t) = dsolve(diff(d(t),t) == -Kd*d(t) + Kd* P , d(0) == 0)
% fplot(y,[0,1])


