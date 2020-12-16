function Jall = transfer_inertia_tensor(Robot, q)
% for dynamics equation M*qdd + C + G = Tau; cal M
% input 1: all kinematics and dynamics para structure,
%       2: joint angle; 6*1
% output Mass matrix 6*6

% Example Input (Aubo Robot):
% clear; clc;
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% M = CalM(q)

%output:
% M =
%
%     2.4409   -1.1648    0.2324    0.0400   -0.0488    0.0001
%    -1.1648    6.5074   -2.7762    0.1924   -0.1214    0.0022
%     0.2324   -2.7762    1.8888   -0.2413    0.1736   -0.0022
%     0.0400    0.1924   -0.2413    0.0958   -0.0511    0.0022
%    -0.0488   -0.1214    0.1736   -0.0511    0.0481   -0.0000
%     0.0001    0.0022   -0.0022    0.0022   -0.0000    0.0022

% ƽ�����ᶨ���������������ת�Ĺ���������ƽ���������ؽ�����ϵ
ARM_DOF = Robot.DOF;
for i= 1:1:6
     Ij{i} = Robot.Para.DP.Ic{i} - Robot.Para.DP.M(i)* skew(Robot.Para.DP.c{i})*skew(Robot.Para.DP.c{i});
end
DH = Robot.DH;
Tadd = [[]];
for i = 1:1:ARM_DOF
    Tadd{i} = homogeneous_transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
end

for i = 1:1:6
    Jall{i}=zeros(3,3);
    for j = i:1:6
        if j == i
            T = eye(4);
        else
            T = T*Tadd{j};
        end
        p = T(1:3,4);
        R = T(1:3,1:3);
        Jt = Ij{j} + Robot.Para.DP.M(j)* skew(p)*skew(p);%% ��j ������������ƽ������ i ������ϵ��
        Jr = R *  Jt * R';%��j ��������������ת���� i ������ϵ��
        Jall{i} = Jall{i} + Jr;
    end
end
end

