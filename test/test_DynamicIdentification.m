clc; clear all; close all
format short
addpath( genpath( '..\MyRobot' ) );
delete('Robot.mat');
robotType = 'I5';
Robot = ParaCAD(robotType);
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
wrench = identificationModel(motionPara,'External',FKLink(motionPara.q,1));
wrench1 = IdentificationMatrix(motionPara,'External') * pp;
e{2} = wrench - wrench1;

% test Internal model // torque in joint
tau = identificationModel(motionPara,'Internal');
tau1 = IdentificationMatrix(motionPara,'Internal') * pp;
e{3} = tau - tau1;

for i = 1:1:length(e)
    if norm(e{i}) >  1e-10
        error(['cal error' num2str(i)])
    end
end



