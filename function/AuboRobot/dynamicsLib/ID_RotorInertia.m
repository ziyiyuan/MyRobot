function [tau,ft] = ID_RotorInertia(motionPara)
% clear all
% close all
global Robot 
% q = [0.6641,    0.3930,   -1.1606,    0.0172,   -1.5708,    0.6641]';
% qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
% qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';

% motionPara.q = q;
% motionPara.qd = qd;
% motionPara.qdd = qdd;

paraType = 'ordinary';
%     paraType = 'linear';

T_radio = [1,1,1,1,1,1];% ������
IA_r = [1,1,1,1,1,1]; %Izz
Link.r = {[1],[2,3],[4],[5],[6],[0]};
% Link.r = {[0],[0],[0],[0],[0],[0]};

ARM_DOF = Robot.DOF;
gravity = Robot.gravity;
M = Robot.Para.DP.M;
MS = Robot.Para.DP.MS;
C = Robot.Para.DP.c; %���� in joint
Ic = Robot.Para.DP.Ic; % ����������ϵת���Ĺ�������
J = Robot.Para.DP.J  ; % �ؽ�����ϵ�Ĺ���������
DH = Robot.DH;

q = motionPara.q;
qd = motionPara.qd;
qdd =motionPara.qdd;
% fk
Tadd = [[]];R = [[]];p = [[]];
for i = 1:1:ARM_DOF
    Tadd{i} = transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
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
T_r = [[]];
link.wmd = [[]];
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
    if strcmp(paraType,'ordinary')
        F{i} = [0;0;0]; % ������Ϊ�ο��㣬���������Ĵ��ĺ����ͺ����أ�
        ss = Link.r{i};
        T_r{i} = zeros(3,1);
        for j = 1:size(ss,2)
            k = ss(j); % ��k��ת�ӣ�
            if k == 0
                T_r{i} = [0;0;0];
            else
                if i == k
                    er = e;
                else
                    R_r = eye(3);
                    for m = i+1 : k
                        R_r = R_r * R{m};
                        er = R_r(:,3);
                    end
                end
                T_r{i} = T_r{i} + T_radio(k)*qdd(k)*IA_r(k)*er + T_radio(k)*qd(k)*IA_r(k)*cross(w{i}, er); % �������Ϳ������ڹؽ�i��ͶӰ
                Link.wmd{k} = wd{i} +  T_radio(k)*qdd(k)*e + T_radio(k)*qd(k)*cross(w{i}, e); %���ת�Ӽ��ٶȣ��ڹؽڴ���������
            end
        end
        T{i} = T_r{i};
    elseif  strcmp(paraType,'linear')
        F{i} = M(i) * vd{i} + cross(wd{i}, MS{i}) + cross(w{i}, cross(w{i}, MS{i}));% �Թؽ�Ϊ�ο��㣬�����ڹؽڴ��ĺ����ͺ����أ�
        T{i} = J{i} * wd{i} + cross(w{i}, J{i} * w{i}) + cross(MS{i}, vd{i});
    end
end

% back �������õ��ڹؽ� i ���������أ�
f = [[]];
t = [[]];
tau = zeros(ARM_DOF,1);
tau_r = [[]];
if strcmp(paraType,'ordinary')
    for i = 6:-1:1
        if i == 6
            f{i} = F{i};
            t{i} = T{i} + cross(C{i},F{i});
        else
            f{i} = R{i+1}* f{i+1} + F{i};
            t{i} = T{i} + R{i+1}* t{i+1} + cross(C{i},F{i}) + cross(p{i+1},R{i+1}* f{i+1});
        end
        tau_r{i} = T_radio(i)*IA_r(i)*Link.wmd{i}'*e;% ����ת���˶���Ҫ�����أ�
        tau(i) = t{i}'*[0 0 1]' + tau_r{i};
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

% force and torque of joint 1 to 6
for i = 6:-1:1
    ft(:,i) = [f{i};t{i}];
end
end



