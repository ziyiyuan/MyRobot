clc
clear all
close all
D1 = load('data_0817.offt');
D2 = load('data1_0820.offt');
plot(D1(1:8000,1))
hold on
plot(D2(1:8000,1))

plot(trajectory.qdd)
