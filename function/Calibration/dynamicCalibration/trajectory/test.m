clc; clear all; close all
format short
%% initialize
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %
%%
global optimal_res 

optimal_res.cond = []; % 存储每次计算结果；
optimal_res.count = 0; % 迭代index
optimal_res.MaxFunEvals = 30000; % 最大迭代次数
identify_model = 'Internal';

load('qq1.mat')
motionPara = ObtainMotionParaForOptimal(qq, Traj.optimal_sample)

ff = resultFunction(qq, identify_model);
[c,ceq] = constrainConditions(identify_model);

valid = validationTrajectory(qq, identify_model)


