clc; clear all; close all
format short
%% initialize
global Robot Traj %#ok<NUSED>
robotType = 'I5';
ParaCAD(robotType); % robot para
trajParaOption(); %
%%
global optimal_res 

optimal_res.cond = []; % �洢ÿ�μ�������
optimal_res.count = 0; % ����index
optimal_res.MaxFunEvals = 30000; % ����������
identify_model = 'Internal';

load('qq1.mat')
motionPara = ObtainMotionParaForOptimal(qq, Traj.optimal_sample)

ff = resultFunction(qq, identify_model);
[c,ceq] = constrainConditions(identify_model);

valid = validationTrajectory(qq, identify_model)


% qqInitial = cal_traj_initial_paraCoeff(Robot,Traj,sampleRate)
% motionPara = cal_motionPara_from_fourier_series(Robot,Traj, qqInitial, sampleRate)
% 
% f = opitimal_objective_function_traj(qqInitial, identifyModel, Robot, Traj)
% [c,ceq] = optimal_constrain_conditions(Robot, motionPara, identificationModel) %% Լ������