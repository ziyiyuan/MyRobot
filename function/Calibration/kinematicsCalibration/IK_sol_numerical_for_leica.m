function  qSol = IK_sol_numerical_for_leica(q_ref, T, DHBeta, qLimit)
% IK_sol_numerical  ������ֵ�⣻�ڹؽڽ����Ʒ�Χ�ڣ�Ŀǰֻ֧��[-pi,pi]
% ���������
%   T: Ŀ��λ�ú���̬��6*1
%   DHMatrix: DH��������˳��Ϊ [alpha, a, d, theta]��6*4
%   qLimit:�ؽڽ����ƣ�6*1
% ���������
%   qSols��������飬6*n;
% ����˵����
%   qSols = IK_all_sol_analytic(T, DHMatrix, qLimit)�� ��������ڹؽ������ڴ��ڵĽ����⣻
%   qSols = IK_all_sol_analytic(T��DHMatrix) :���������[-175,175]�ؽ������ڴ��ڵĽ����⣻
% �汾��V1.0����д��2020/8/27���޸���2020/9/1�����ߣ�ziyi
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
% ���ӿ�ȥ�ж� �������Ƿ񳬳��ؽ����ƣ�
iter
end
