clc; clear all; close all
format short
%% initialize
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %
%%
global optimal_res identifyModel
identifyModel = 'Internal';
optimal_res.cond = []; % 存储每次计算结果；
optimal_res.count = 0; % 迭代index
optimal_res.MaxFunEvals = 30000; % 最大迭代次数
% qqInitial = TrajInitialPara(); %% 计算轨迹参数优化初始值；
qqInitial = (rand(1,66)*2 - 1);
tic
[finalPara, optimizationIndex] = FindperiodicEexcitationTrajectory(qqInitial, 'robot', optimal_res.MaxFunEvals, 'External');
toc
plot(optimal_res.cond)


