% 一阶微分方程求解

clc
clear all ; close all
syms r(t) K0 Pext
% K0 = 10000;
% Pext = 10;

figure(1)
y(t) = dsolve(diff(r(t),t) == K0*(Pext - r(t)), r(0) == 0)
% fplot(y,[0,1])