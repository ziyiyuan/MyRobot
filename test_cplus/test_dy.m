clc; clear all; close all
format short
addpath( genpath( '..\function' ) )
%% initialize
robotType = 'I5';
Robot = get_cad_model_para(robotType);

q = [0.6641,    0.3930,   -1.1606,    0.0172,   -1.5708,    0.6641]';
qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';

motionPara.q = q;
motionPara.qd = qd;
motionPara.qdd = qdd;

%%
q_all = load('q.txt');
qd_all = load('qd.txt');
qdd_all = load('qdd.txt');
sensor_data = load('sensor_data.txt');
motionTraj.q = q_all';
motionTraj.qd = qd_all';
motionTraj.qdd = qdd_all';
identificationModel = 'External';

postData.motionTraj = motionTraj;
postData.sensorData = sensor_data';
addConstraints.M.cons = 0;


estILS = 0; %采用增量最小二乘

[paraEst,E0] = estimate_SDP(Robot, identificationModel, postData, addConstraints, estILS)

regression = cal_identification_matrix(Robot, motionTraj, identificationModel)



HH = cal_ele_identification_matrix(Robot, motionPara,'Internal')
HE = cal_ele_identification_matrix(Robot, motionPara,'External')
 
 identifyPara.linkModel = 1;
 identifyPara.roterInertiaModel = 1;
 identifyPara.frictionModel = 1;
 identifyPara.offsetModel = 1;
 HS = cal_identification_matrix(Robot, motionPara, 'Internal', identifyPara)