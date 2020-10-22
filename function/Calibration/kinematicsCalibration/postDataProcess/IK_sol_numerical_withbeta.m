function  [qSol,iter] = IK_sol_numerical_withbeta(q_ref, T, DHMatrix, qLimit, maxIter)
% IK_sol_numerical  计算带beta的数值解；在关节角限制范围内；
% 输入参数：
%   T: 目标位置和姿态，6*1
%   DHMatrix: DH参数矩阵，顺序为 [alpha, a, d, theta]，6*4
%   qLimit:关节角限制，6*1
% 输出参数：
%   qSols：解的数组，6*n;
% 调用说明：
%   qSols = IK_all_sol_analytic(T, DHMatrix, qLimit)； 输出所有在关节限制内存在的解析解；
%   qSols = IK_all_sol_analytic(T，DHMatrix) :输出所有在[-175,175]关节限制内存在的解析解；
% 版本号V1.0，编写于2020/8/27，修改于2020/9/1，作者：ziyi
if nargin >5
    error('输入变量过多！');
elseif (nargin == 4)
    maxIter = 30;
end

if size(DHMatrix,2) < 5
    calibeta = 0;
else
    calibeta = 1;
    DHBeta = DHMatrix;
    DHMatrix = DHBeta(:,1:4);
end

Tt = T;
qSol = q_ref;
tolerancePos = 1e-14;
toleranceOri = 1e-14;

for i = 1:1:maxIter
    if calibeta
        Treal = forward_kinematics_DHcomped(qSol, DHBeta, 6);
    else
        Treal = forward_kinematics(qSol, DHMatrix, 6);
    end
    delta = tr2delta(Treal, Tt);% in base;
    eP = norm(delta(4:6));
    eO = norm(delta(1:3));
    if (eP < tolerancePos && eO < toleranceOri)
        return
    end
    J = cal_jacobian(qSol, DHMatrix);
    dq = J{6}\delta;
    qSol = qSol + dq;
end
iter = i;
for i = 1:1:6
    while(qSol(i) > qLimit(i))
        qSol(i) = qSol(i) - 2 * pi;
    end
    while(qSol(i) < -qLimit(i))
        qSol(i) = qSol(i) + 2 * pi;
    end
end
end
