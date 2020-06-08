function  J = CalJacobian(q, point)

% input: q: jopint angle 
%        point : out put jacoboan in the point, if point == 'joint', then
%        out put the jacobian of joint point; if point == 'center'; output
%        the jacobian of link center
% output: J; the jacobian of choose point in base ; is a 1 * 6 cell;

% Example Input (Aubo Robot):
% clear; clc;
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% J = CalJacobian(q,'joint')
% J{6} % the jacooboan od end joint point 

% output:
% ans =
% 
%     0.1999   -0.5597    0.1535    0.0304   -0.0318         0
%     0.4689   -0.0186    0.0051    0.0010   -0.0058         0
%          0    0.4620   -0.4259    0.0980   -0.0883         0
%          0    0.0332   -0.0332    0.0332    0.9396    0.0505
%          0   -0.9994    0.9994   -0.9994    0.0312   -0.9976
%     1.0000    0.0000   -0.0000    0.0000   -0.3410    0.0478

load('Robot.mat');
DH = Robot.DH;

T = [[]];
for i = 1:1:6
    T{i} = transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
end

T0 = [[]];
z0 = [[]]; c0 = [[]];o0 = [[]];
for  i = 1:1:6
    if i == 1
        T0{i} =  T{i};
    else
        T0{i} = T0{i-1} * T{i};
    end
    z0{i} = T0{i}(1:3,3); % 旋转轴 in base
    c0{i} = T0{i}(1:3,4) + T0{i}(1:3,1:3) * Robot.Para.DP.c{i}; % 质心在世界坐标系的描述
    o0{i} = T0{i}(1:3,4); % joint position in base
    % I 刚体绕质心转动的惯性张量在质心坐标系的描述；
    % 平行移轴定理：刚体绕关节坐标系转动的惯性张量在关节坐标系的描述；
    % J 刚体绕质心转动的惯性张量 在世界坐标系的描述；
end

if strcmp(point,'center')
    P = c0;
elseif strcmp(point,'joint')
    P = o0;
else
    P = o0;
end

% cal jacobian
JLv = [[]];
JLw = [[]];
J = [[]];
for i = 1:1:6
    JLv{i} = zeros(3,6);
    JLw{i} = zeros(3,6);
    for j = 1:1:i
        JLv{i}(:,j) = cross(z0{j},(P{i}-o0{j}));
        JLw{i}(:,j) = z0{j};
    end
    J{i} = [JLv{i};JLw{i}];
end

end