function  qSol = IK_sol_numerical_for_leica(q_ref, T, DHBeta, qLimit)
% IK_sol_numerical  计算数值解；在关节角限制范围内；目前只支持[-pi,pi]
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
Tt = T;
qSol = q_ref;
Treal = FK_DH_COMP(DHBeta,qSol);
DHMatrix = DHBeta(:,1:4);

% Treal = forward_kinematics(qSol, DHMatrix, 6);
delta = tr2delta(Treal, Tt);% in base;
eP = norm(delta(4:6));
eO = norm(delta(1:3));
tolerancePos = 1e-14;
toleranceOri = 1e-14;

iter = 0;
while eP > tolerancePos || eO > toleranceOri
    iter = iter + 1;
    J = cal_jacobian(qSol, DHMatrix);
    dq = J{6}\delta;
    qSol = qSol + dq;
    Treal = FK_DH_COMP(DHBeta,qSol);
%     Treal = forward_kinematics(qSol, DHMatrix, 6);
    delta = tr2delta(Treal, Tt);% in base;
    eP = norm(delta(4:6));
    eO = norm(delta(1:3));
end
for i = 1:1:6
    while(qSol(i) > qLimit(i))
        qSol(i) = qSol(i) - 2 * pi;
    end
    while(qSol(i) < -qLimit(i))
        qSol(i) = qSol(i) + 2 * pi;
    end
end
% 逆解接口去判断 迭代解是否超出关节限制；
iter
end
