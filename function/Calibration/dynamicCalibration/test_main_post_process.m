clc
clear all
robotType = 'I5';
Robot = get_cad_model_para(robotType);
Traj = set_excitation_traj_feature();
sampleRate = 200;
identificationModel = 'External';
% identificationModel = 'Internal';
addConstraints.M.cons = 1;
addConstraints.M.offset = 1;
addConstraints.M.reff = Robot.Para.DP.Mreff;


%% load data
% addpath(genpath('C:\Users\Sherry\Desktop\handguiding\test_data\date_internal'))
% NUM = 1;
% %
% q_all = load(['aubo_q_',num2str(NUM),'.txt']);
% qd_all = load(['aubo_qd_',num2str(NUM),'.txt']);
% qdd_all = load(['aubo_qdd_',num2str(NUM),'.txt']);
% current_all = load(['aubo_current_',num2str(NUM),'.txt']);
%
% postData.motionTraj.q = q_all';
% postData.motionTraj.qd = qd_all';
% postData.motionTraj.qdd = qdd_all';
% postData.currentData = current_all';

addpath(genpath('C:\Users\Sherry\Desktop\handguiding\test_data\data_20200908'))

datafile = 'jointStatusRecord_lizy1.txt';
motionParaCoeff = 'qq_lizy1.mat';
estILS = 0; %采用增量最小二乘
postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate);

%% get CADPara
% identifyPara.linkModel = 1;
% identifyPara.offsetModel = 0;
% identifyPara.frictionModel = 0;
% identifyPara.roterInertiaModel = 0;
% regression = cal_ele_identification_matrix(Robot, postData.motionTraj,identificationModel)
% [Col,beta] = get_mini_para_set_numerical(regression)
% [X1,expersion] = get_mini_para_set_symbol(identifyPara,Col,beta)
% parasetCAD = get_mini_para_set_CAD(Robot,Col,beta)
%%

[paraEst,E0] = estimate_SDP(Robot, identificationModel, postData, addConstraints, estILS)

% compare_wrench_based_cad_and_measure(Robot, postData.motionTraj, [], postData.sensorData二乘
data = [1,2,4];
for i = 1:1:3
    datafile = ['jointStatusRecord_lizy',num2str(data(i)),'.txt'];
    motionParaCoeff = ['qq_lizy',num2str(data(i)),'.mat'];
    postDataValidation = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate);
    res{i} = validation(Robot,identificationModel,postDataValidation, paraEst);
end
res

%%