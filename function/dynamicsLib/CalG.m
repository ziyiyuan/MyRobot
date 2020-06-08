function G = CalG(q)
% for dynamics equation M*qdd + C + G = Tau; cal G
% input 1: all kinematics and dynamics para structure,
%       2: joint angle; 6*1
% output G matrix 6*1

% Example Input (Aubo Robot):
% clear; clc;
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% M = CalG(q)

%output:
% G =
% 
%          0
%    46.1645
%   -40.0441
%     5.2810
%    -4.6001
%     0.0000

load('Robot.mat');

J = CalJacobian(q, 'center');

JLv = [[]];

for i = 1:1:6
    JLv{i} = J{i}(1:3,:);
end

%% cal G
G = zeros(6,1);
Gf = [[]];
for i = 1:1:6
    Gf{i} = Robot.Para.DP.M(i) * Robot.gravity;
    G = G + JLv{i}' * (-Gf{i}); % 机器人受到向下的重力，关节要产生一个反向的作用力来与之平衡
end
end

