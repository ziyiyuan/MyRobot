 clc; clear all; close all
format short
% addpath( genpath( '..\MyRobot' ) );
addpath( genpath( '.\function' ) )
%% initialize
global robotType
robotType = 'I5';
%%

Robot = get_cad_model_para(robotType);
%% 
syms q1 q2 q3 q4 q5 q6 real
q = [q1;q2;q3;q4;q5;q6];
% q = [1;1;1;1;1;1];
% M1 = cal_M(Robot, q)
% diff(M,q1)

DHMatrix = Robot.DH;
T0 = forward_kinematics(q, DHMatrix);
massCenter = Robot.Para.DP.c;
calCenter = 1;
z0 = [[]]; o0 = [[]];
P = [[]];
for  i = 1:1:6
    
    z0{i} = T0{i}(1:3,3); % 旋转轴 in base
    o0{i} = T0{i}(1:3,4); % 位置
    if calCenter
        P{i} = T0{i}(1:3,4) + T0{i}(1:3,1:3) * massCenter{i}; % 质心在世界坐标系的描述
    else
        P{i} = o0{i}; % joint coordinate position in base
    end
end

% cal jacobian
JLv = [[]];
JLw = [[]];
Jall = [[]];
for i = 1:1:6
    for j = 1:1:6
        if j <= i
            JLv{i}(:,j) = cross(z0{j},(P{i}-o0{j}));
            JLw{i}(:,j) = z0{j};
        else
            JLv{i}(:,j) = zeros(3,1);
            JLw{i}(:,j) = zeros(3,1);
        end
    end
    Jall{i} = [JLv{i};JLw{i}];
end
J = Jall;
% calM
Ji = [[]];
for  i = 1:1:6
    % I 刚体绕质心转动的惯性张量在质心坐标系的描述；
    % 平行移轴定理：刚体绕关节坐标系转动的惯性张量在关节坐标系的描述；
    % J 刚体绕质心转动的惯性张量 在世界坐标系的描述；
    Ji{i} = T0{i}(1:3,1:3) * Robot.Para.DP.Ic{i} * T0{i}(1:3,1:3)';%
end

% 计算质心处的雅克比
JLv = [[]];
JLw = [[]];
for i = 1:1:6
    JLv{i} = J{i}(1:3,:);
    JLw{i} = J{i}(4:6,:);
end

M = [];
for i = 1:1:6
    if i==1
        M = JLv{i}' .* Robot.Para.DP.M(i) * JLv{i} + JLw{i}' * Ji{i} * JLw{i};
    else
        M = M + JLv{i}' .* Robot.Para.DP.M(i) * JLv{i} + JLw{i}' * Ji{i} * JLw{i};
    end
end
M
Mq1 = diff(M,q2)
subs(Mq1,[q1;q2;q3;q4;q5;q6],[1;1;1;1;1;1])
