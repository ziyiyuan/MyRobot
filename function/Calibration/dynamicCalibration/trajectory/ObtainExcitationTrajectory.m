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
optimal_res.cond = []; % �洢ÿ�μ�������
optimal_res.count = 0; % ����index
optimal_res.MaxFunEvals = 30000; % ����������
% qqInitial = TrajInitialPara(); %% ����켣�����Ż���ʼֵ��
qqInitial = (rand(1,66)*2 - 1);
tic
[finalPara, optimizationIndex] = FindperiodicEexcitationTrajectory(qqInitial, 'robot', optimal_res.MaxFunEvals, 'External');
toc
plot(optimal_res.cond)


