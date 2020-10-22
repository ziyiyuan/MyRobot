function T = forward_kinematics_DHcomped(q, DHMatrix, i) 
% forward_kinematics_DHcomped 计算带 beta 的正解
% 输入参数：
%   q: 输入的关节角，6*1
%   DHMatrix: DH参数矩阵，顺序为 [alpha, a, d, theta, beta]，6*5
%   i：第 i 个关节角在基坐标系的描述
% 输出参数：
%   T：齐次变换矩阵
% 调用说明：
%   T = forward_kinematics_DHcomped(q, DHMatrix) :输出所有关节坐标系在基坐标系的描述；
%   T = forward_kinematics_DHcomped(q, DHMatrix, i) ：输出第 i 个关节坐标系在基坐标系的描述；

% 版本号V1.0，编写于2020/8/27，修改于2020/8/27，作者：ziyi

if nargin > 3
    error('输入变量过多！');
end
p = zeros(3,1);
T0_1 = RpToTrans(RotY(DHMatrix(1,5)), p) * homogeneous_transfer(DHMatrix(1,1),DHMatrix(1,2),DHMatrix(1,3),DHMatrix(1,4) + q(1));
T1_2 = RpToTrans(RotY(DHMatrix(2,5)), p) * homogeneous_transfer(DHMatrix(2,1),DHMatrix(2,2),DHMatrix(2,3),DHMatrix(2,4) + q(2));
T2_3 = RpToTrans(RotY(DHMatrix(3,5)), p) * homogeneous_transfer(DHMatrix(3,1),DHMatrix(3,2),DHMatrix(3,3),DHMatrix(3,4) + q(3));
T3_4 = RpToTrans(RotY(DHMatrix(4,5)), p) * homogeneous_transfer(DHMatrix(4,1),DHMatrix(4,2),DHMatrix(4,3),DHMatrix(4,4) + q(4));
T4_5 = RpToTrans(RotY(DHMatrix(5,5)), p) * homogeneous_transfer(DHMatrix(5,1),DHMatrix(5,2),DHMatrix(5,3),DHMatrix(5,4) + q(5));
T5_6 = RpToTrans(RotY(DHMatrix(6,5)), p) * homogeneous_transfer(DHMatrix(6,1),DHMatrix(6,2),DHMatrix(6,3),DHMatrix(6,4) + q(6));

Ti{1} = T0_1;
Ti{2} = Ti{1}*T1_2;
Ti{3} = Ti{2}*T2_3;
Ti{4} = Ti{3}*T3_4;
Ti{5} = Ti{4}*T4_5;
Ti{6} = Ti{5}*T5_6;

if nargin == 2
    T = Ti;
else
    T = Ti{i};
end
end