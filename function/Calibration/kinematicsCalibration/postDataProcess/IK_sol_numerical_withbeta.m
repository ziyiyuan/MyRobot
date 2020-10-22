function  [qSol,iter] = IK_sol_numerical_withbeta(q_ref, T, DHMatrix, qLimit, maxIter)
% IK_sol_numerical  �����beta����ֵ�⣻�ڹؽڽ����Ʒ�Χ�ڣ�
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
if nargin >5
    error('����������࣡');
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
