clc; clear all; close all
format short

%% initialize
robotType = 'I5';
Robot = get_cad_model_para(robotType);
%%
e = [[]];
% te
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
% qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';
q = [0.6641,    0.3930,   -1.1606,    0.0172,   -1.5708,    0.6641]';
qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';

motionPara.q = q;
motionPara.qd = qd;
motionPara.qdd = qdd;

%%

 HH = cal_ele_identification_matrix(Robot, motionPara,'Internal')
 
 identifyPara.linkModel = 1;
 identifyPara.roterInertiaModel = 1;
 identifyPara.frictionModel = 1;
 identifyPara.offsetModel = 1;
 HE = cal_identification_matrix(Robot, motionPara, 'Internal', identifyPara)



%%
% test ID_NewtonEuler // 返回每个关节的力和力矩，线性牛顿欧拉和迭代牛顿欧拉；
% linear model and ordinary model
[tau1,ft] = ID_NewtonEuler(motionPara,'linear'); %
tau2 = ID_NewtonEuler(motionPara,'linear'); %
[tau3,ft2] = ID_NewtonEuler(motionPara);
e{1} = ft - ft2;
%% identification model
J = Robot.Para.DP.J;
MS = Robot.Para.DP.MS;
M = Robot.Para.DP.M;
for i = 1:Robot.DOF
    P{i} = [J{i}(1,1) J{i}(1,2) J{i}(1,3) J{i}(2,2) J{i}(2,3) J{i}(3,3) MS{i}(1) MS{i}(2) MS{i}(3) M(i)]';
end
pp = [P{1}; P{2}; P{3}; P{4}; P{5}; P{6}];

% test External model // wrench in base 
wrench = identificationModel(motionPara,'External',FKLink(motionPara.q,1)); %% iter NE  wrench inbase
wrench1 = IdentificationMatrix(motionPara,'External') * pp; %%Linear NE 
e{2} = wrench - wrench1;

% test Internal model // torque in joint
tau = identificationModel(motionPara,'Internal'); %% torque in joint
tau1 = IdentificationMatrix(motionPara,'Internal') * pp;
e{3} = tau - tau1;

% test MiniPara // Internal model 解析解
HH = IdentificationMatrix(motionPara,'Internal');
[index, MDP] = GetRobotIdyMimParaSet('Internal');
e{4} = HH(:,index) * MDP - tau;

% test MiniPara // External model jiexijie 
HH = IdentificationMatrix(motionPara,'External');
[index, MDP] = GetRobotIdyMimParaSet('External');
e{5} = HH(:,index) * MDP - wrench;

%% test rotor inertia
[tau1,ft1] = ID_NewtonEulerWithRotorInertia(motionPara)
[tau2,ft2] = ID_RotorInertia(motionPara)
[tau3,ft3] = ID_NewtonEuler(motionPara)
e{6} = tau1 - (tau2 + tau3)
e{6} = tau1 - (tau2 + tau3)



for i = 1:1:length(e)
    if norm(e{i}) >  1e-10
        error(['cal error' num2str(i)])
    end
end



