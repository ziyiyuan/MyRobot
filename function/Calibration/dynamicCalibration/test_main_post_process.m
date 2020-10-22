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

estILS = 0; %采用增量最小二乘
%% load data for internal identification
% addpath(genpath('C:\Users\Sherry\Desktop\MyRobot\dataLib\dynamicCalibrationData\date_internal'))
% NUM = 1;
% q_all = load(['aubo_q_',num2str(NUM),'.txt']);
% qd_all = load(['aubo_qd_',num2str(NUM),'.txt']);
% qdd_all = load(['aubo_qdd_',num2str(NUM),'.txt']);
% current_all = load(['aubo_current_',num2str(NUM),'.txt']);
% 
% postData.motionTraj.q = q_all';
% postData.motionTraj.qd = qd_all';
% postData.motionTraj.qdd = qdd_all';
% postData.currentData = current_all';
%% load data 
% addpath(genpath('C:\Users\Sherry\Desktop\MyRobot\dataLib\dynamicCalibrationData\data_20200908'))
% datafile = 'jointStatusRecord_lizy1.txt';
% motionParaCoeff = 'qq_lizy1.mat';
% postData = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate);
load('postData.mat')
%% get CADPara
if 0
Robot.gravity = [1,2,3]'/norm([1,2,3]);
identifyPara.linkModel = 1;
identifyPara.offsetModel = 0;
identifyPara.frictionModel = 0;
identifyPara.roterInertiaModel = 0;
regression = cal_identification_matrix(Robot, postData.motionTraj, identificationModel,identifyPara);
[Col,beta] = get_mini_para_set_numerical(regression)
[X1,expersion] = get_mini_para_set_symbol(identifyPara,Col,beta)
% parasetCAD = get_mini_para_set_CAD(Robot,Col,beta)
end
%%
[paraEst,E0] = estimate_SDP(Robot, identificationModel, postData, addConstraints, estILS)

data = [1,2,4];
for i = 1:1:3
    datafile = ['jointStatusRecord_lizy',num2str(data(i)),'.txt'];
    motionParaCoeff = ['qq_lizy',num2str(data(i)),'.mat'];
    postDataValidation = post_sensor_data_process(Robot, Traj, datafile, motionParaCoeff, sampleRate);
    res{i} = validation(Robot,identificationModel,postDataValidation, paraEst);
end
res*100

%%