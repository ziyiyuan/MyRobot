function link = Link_Velocity(motionPara)
%Inputs : motionPara: the struct of q qd qdd
%         varargin  : the modle type, ordinary, or linear NE
%outputs�� tau�� �ؽڴ������� 6*1
%          ft���ؽ�����ϵ�������������ڹؽ�����ϵ������

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
load('Robot.mat');

ARM_DOF = Robot.DOF;
gravity = Robot.gravity;
C = Robot.Para.DP.c; %���� in joint
DH = Robot.DH;

q = motionPara.q;
qd = motionPara.qd;
qdd =motionPara.qdd;
% fk
T = [[]];R = [[]];p = [[]];
for i = 1:1:ARM_DOF
    T{i} = transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
    R{i}= T{i}(1:3,1:3);
    p{i} = T{i}(1:3,4);
end

%% NewtonEuler itera
% expressed in link i coordinate;
w = [[]];  w0 = [0 0 0]';
wd = [[]]; wd0 = [0 0 0]';
v = [[]];  v0 = [0 0 0]';
vd = [[]]; vd0 =  - [gravity(1);gravity(2);gravity(3)];
vc = [[]];
vcd = [[]];
% forward �������˵Ľ��ٶȣ��Ǽ��ٶȣ��ؽں����ĵ��ٶȺͼ��ٶ�
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
end
link.w = w;
link.wd = wd;
link.v = v;
link.vd = vd;
link.vc = vc;
link.vcd = vcd;
end