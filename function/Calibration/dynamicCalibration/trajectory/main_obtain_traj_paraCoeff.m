% 优化轨迹主函数；
clc
clear all
robotType = 'I5';
Robot = get_cad_model_para(robotType);
Traj = set_excitation_traj_feature();

sampleRate = 10;
identificationModel = 'Internal';
%%
global optimal_res 

optimal_res.cond = []; % 存储每次计算结果；
optimal_res.count = 0; % 迭代index
optimal_res.MaxFunEvals = 30000; % 最大迭代次数

qqInitial = (rand(1,66)*2 - 1);
% qqInitial = cal_traj_initial_paraCoeff(Robot,Traj,sampleRate);
qqInitial = ones(1,66);
tic
[finalPara, optimizationIndex] = optimal_find_periodic_excitation_trajectory(Robot, Traj, 'robot', identificationModel, optimal_res.MaxFunEvals, qqInitial, sampleRate);
toc
plot(optimal_res.cond)



