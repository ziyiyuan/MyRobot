function  J = cal_jacobian(q,DHMatrix,massCenter)
% cal_jacobian 计算雅克比矩阵
% 输入参数：
%   q: 输入的关节角，6*1
%   DHMatrix: DH参数矩阵，顺序为 [alpha, a, d, theta]，6*4
%   massCenter：质心在连杆坐标系的坐标；
% 输出参数：
%   J：雅克比矩阵6*6*6
% 调用说明：
%   J = cal_jacobian(q,DHMatrix,massCenter) :输出所有连杆质心处的雅克比矩阵；用在求动力学 M 矩阵
%   J = cal_jacobian(q,DHMatrix) ：输出所有连杆坐标系处的雅克比矩阵；

% 版本号V1.0，编写于2020/8/27，修改于2020/8/27，作者：ziyi

calCenter = 1;
if nargin > 3
    error('输入变量过多！');
elseif nargin < 3
    calCenter = 0;
end

T0 = forward_kinematics(q, DHMatrix);

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
    JLv{i} = zeros(3,6);
    JLw{i} = zeros(3,6);
    for j = 1:1:i
        JLv{i}(:,j) = cross(z0{j},(P{i}-o0{j}));
        JLw{i}(:,j) = z0{j};
    end
    Jall{i} = [JLv{i};JLw{i}];
end
J = Jall;
end