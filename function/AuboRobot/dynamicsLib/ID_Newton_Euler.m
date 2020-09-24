function [tau,ft] = ID_Newton_Euler(Robot, motionPara, varargin)
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
gravity = Robot.gravity;
M = Robot.Para.DP.M;
MS = Robot.Para.DP.MS;
C = Robot.Para.DP.c; %质心 in joint
Ic = Robot.Para.DP.Ic; % 绕质心坐标系转动的惯性张量
J = Robot.Para.DP.J  ; % 关节坐标系的惯性张量；
DH = Robot.DH;

if (nargin < 2)
    paraType = 'ordinary';
else
    paraType = 'linear';
end

q = motionPara.q;
qd = motionPara.qd;
qdd =motionPara.qdd;
% fk
Tadd = [[]];R = [[]];p = [[]];
for i = 1:1:ARM_DOF
    Tadd{i} = homogeneous_transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
    R{i}= Tadd{i}(1:3,1:3);
    p{i} = Tadd{i}(1:3,4);
end

%% NewtonEuler itera
% expressed in link i coordinate;
w = [[]];  w0 = [0 0 0]';
wd = [[]]; wd0 = [0 0 0]';
v = [[]];  v0 = [0 0 0]';
vd = [[]]; vd0 =  - [gravity(1);gravity(2);gravity(3)];
vc = [[]];
vcd = [[]];
F = [[]];
T = [[]];
% forward 计算连杆的角速度，角加速度，关节和质心的速度和加速度
e = [0 0 1]';
for i = 1:ARM_DOF
    if i == 1
        w{i} =  R{i}'*w0 + qd(i)*e;
        wd{i} = R{i}'*wd0 + qdd(i)*e + cross(R{i}'*w0, qd(i)*e);
        v{i} =  R{i}'*(v0 + cross(w0, p{i}));
        vd{i} = R{i}'*(vd0 + cross(w0, cross(w0, p{i})) + cross(wd0, p{i}));
        vc{i} = v{i} + cross(w{i},C{i});
        vcd{i} = vd{i} + cross(wd{i},C{i}) + cross(w{i},cross(w{i},C{i}));
    else
        w{i} =  R{i}'*w{i-1} + qd(i)*e;
        wd{i} = R{i}'*wd{i-1} + qdd(i)*e + cross(R{i}'*w{i-1},qd(i)*e);
        v{i} =  R{i}'*(v{i-1} + cross(w{i-1},p{i}));
        vd{i} = R{i}'*(vd{i-1} + cross(w{i-1}, cross(w{i-1}, p{i})) + cross(wd{i-1}, p{i}));
        vc{i} = v{i} + cross(w{i},C{i});
        vcd{i} = vd{i} + cross(wd{i},C{i}) + cross(w{i},cross(w{i},C{i}));
    end
    if strcmp(paraType,'ordinary')
        F{i} = M(i) * vcd{i}; % 以质心为参考点，作用在质心处的合力和合力矩；
        T{i} = Ic{i} * wd{i} + cross(w{i}, Ic{i} * w{i});
    elseif  strcmp(paraType,'linear')
        F{i} = M(i) * vd{i} + cross(wd{i}, MS{i}) + cross(w{i}, cross(w{i}, MS{i}));% 以关节为参考点，作用在关节处的合力和合力矩；
        T{i} = J{i} * wd{i} + cross(w{i}, J{i} * w{i}) + cross(MS{i}, vd{i});
    end
end

% back 计算作用的在关节 i 的力和力矩；
f = [[]];
t = [[]];
tau = zeros(ARM_DOF,1);
if strcmp(paraType,'ordinary')
    for i = 6:-1:1
        if i == 6
            f{i} = F{i};
            t{i} = T{i} + cross(C{i},F{i});
        else
            f{i} = R{i+1}* f{i+1} + F{i};
            t{i} = T{i} + R{i+1}* t{i+1} + cross(C{i},F{i}) + cross(p{i+1},R{i+1}* f{i+1});
        end
        tau(i) = t{i}'*[0 0 1]';
    end
elseif  strcmp(paraType,'linear')
    for i = 6:-1:1
        if i == 6
            f{i} = F{i};
            t{i} = T{i};
        else
            f{i} = R{i+1}* f{i+1} + F{i};
            t{i} = T{i} + R{i+1}* t{i+1} + cross(p{i+1},R{i+1}* f{i+1});
        end
        tau(i) = t{i}'*[0 0 1]';
    end
end
%%
if nargout > 1
    % force and torque of joint 1 to 6
    for i = 6:-1:1
        ft(:,i) = [f{i};t{i}];
    end
end
end

