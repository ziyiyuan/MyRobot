function [tau_tool,FTt] = calToolWrench(Robot, motionPara, toolPara)%(J,MS,M)
%Inputs : motionPara: the struct of q qd qdd
%         varargin  : the modle type, ordinary, or linear NE
%outputs： tau： 关节处的力矩 6*1
%          ft：关节坐标系处的力和力矩在关节坐标系的描述

% Example Inputs:
%
% clear; clc;
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
% qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';
% motionPara.q = q;
% motionPara.qd = qd;
% motionPara.qdd = qdd;
%
% tau = ID_NewtonEuler(motionPara, 'linear') %
% tau = ID_NewtonEuler(motionPara)

ARM_DOF = Robot.DOF;
DH = Robot.DH;

link = get_link_velocity(Robot, motionPara);
T = [[]];
for j = 1:1:ARM_DOF
    T{j} = homogeneous_transfer(DH(j,1),DH(j,2),DH(j,3),DH(j,4) + motionPara.q(j));
end
Wii = get_rigid_body_wrench_linear_matrix(link.w{ARM_DOF},link.wd{ARM_DOF},link.vd{ARM_DOF}); % wii wrench in joint 6;
FTt = Wii*toolPara;
for j = ARM_DOF:-1:1
    if(ARM_DOF == j)
        T1 = eye(4);
    else
        T1 = T{j+1}*T1;
    end
    HA{j} = trans_wrench_linear_matrix(T1, Wii);
end
WW = [];
for k = 1:1:6
    S1 = [0,0,0,0,0,1] * HA{k}; % 工具运动 在 joint j 处产生的力
    WW = [WW;S1];
end
tau_tool = WW * toolPara;
end


function Wii = get_rigid_body_wrench_linear_matrix(w,wd,vd)
% Fi = Wii * Para_i
Wii = [zeros(3,6) skew(wd)+skew(w)*skew(w) vd;skewStar(wd)+skew(w)*skewStar(w) -skew(vd) zeros(3,1)]; %Wii
end

function HA = trans_wrench_linear_matrix(T, Wii)
RR = [T(1:3,1:3) zeros(3,3); skew(T(1:3,4))*T(1:3,1:3) T(1:3,1:3)]; %TAi
HA = RR * Wii;
end