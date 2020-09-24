function M = cal_M(Robot, q)
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

DHMatrix = Robot.DH;
T0 = forward_kinematics(q, DHMatrix);
J = cal_jacobian(q, DHMatrix, Robot.Para.DP.c);

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

M = zeros(6,6);
for i = 1:1:6
    M = M + JLv{i}' .* Robot.Para.DP.M(i) * JLv{i} + JLw{i}' * Ji{i} * JLw{i};
end
end

