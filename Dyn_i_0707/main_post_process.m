clc
clear all
%% initiallize
robotType = 'I5';
Robot = get_cad_model_para(robotType);
Traj = set_excitation_traj_feature();
sampleRate = 200;
%% set up identification para
% identificationModel = 'External'; % 辨识模型
identificationModel = 'Internal';

% 参数估计方法；OLS 和 SDP 或者 增量最小二乘
estILS = 0; %采用增量最小二乘

% 参数估计增加的约束
addConstraints.M.cons = 1;
addConstraints.M.offset = 1;
addConstraints.M.reff = Robot.Para.DP.Mreff;

%% load data
addpath(genpath('C:\Users\Sherry\Desktop\handguiding\test_data\data_20200908'))

datafile = 'jointStatusRecord_lizy1.txt';
motionParaCoeff = 'qq_lizy1.mat';

postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate); % 预处理
postData.currentData = postData.currentData./Robot.Para.TC;
% sensoroffset = [37.902191, 26.883101, 27.587963, -0.585879, -2.121200, 0.568004]';
% postData.sensorData = postData.sensorData + sensoroffset;

%% estimate and validation
[paraEst,E0] = estimate_SDP(Robot, identificationModel, postData, addConstraints, estILS)

data = [1,2,4];
for i = 1:1:3
    datafile = ['jointStatusRecord_lizy',num2str(data(i)),'.txt'];
    motionParaCoeff = ['qq_lizy',num2str(data(i)),'.mat'];
    postDataValidation = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate);
    postDataValidation.currentData = postDataValidation.currentData./Robot.Para.TC;
%     postDataValidation.sensorData = postDataValidation.sensorData + sensoroffset;
    res{i} = validation(Robot,identificationModel,postDataValidation, paraEst);
end
res
%% 计算符号表达式
% CAD
identifyPara.linkModel = 1;
identifyPara.offsetModel = 0;
identifyPara.frictionModel = 0;
identifyPara.roterInertiaModel = 0;

regression = cal_identification_matrix(Robot, postData.motionTraj, identificationModel, identifyPara);
[Col,beta] = get_mini_para_set_numerical(regression);
[X1,expersion] = get_mini_para_set_symbol(identifyPara,Col,beta) % 符号表达式
parasetCAD = get_mini_para_set_CAD(Robot,Col,beta)
compare_wrench_based_cad_and_measure(Robot, postData.motionTraj, postData.sensorData, postData.currentData./Robot.Para.TC)
% 
