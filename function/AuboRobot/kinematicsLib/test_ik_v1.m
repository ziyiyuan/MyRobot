% 测试逆解相关函数
% 版本号V1.0，编写于2020/8/27，修改于2020/8/27，作者：ziyi
clc
clear all
robotType = 'I5';
Robot = get_cad_model_para(robotType);
%%
qLimit = Robot.Limit.q;
q_all = [];

indexNoSol = [];
numNoSol = 0;

indexLoseSol = [];
numLoseSol = 0;

indexSingular = [];
numSingular = 0;

index8Solu = [];
numindex8Solu = 0;

q = [ -1.7550   -2.2192   -1.0521    0.6865    0.0012    2.9225]';
J = cal_jacobian(q,Robot.DH);
jd = det(J{6})

for i = 1:1:1000
    
    q_in = -qLimit + 2*qLimit.*rand(6,1);
    q_ref = q_in + rand(6,1)*0.1;
    %     q_in(3) = 0;
    q_all = [q_all,q_in];
    T1 = forward_kinematics(q_in, Robot.DH,6);
    detJ = get_jacobian_det(q_in, Robot);
    if abs(detJ) < 1e-8
        indexSingular = [indexSingular;i];
        numSingular = numSingular + 1;
    end
    
    qSols = IK_all_sol_analytic(T1, Robot.DH);
    
    q_sol = IK_choose_one_sol_norm(qSols, q_in)
    
    
    qSolNumerical = IK_sol_numerical(q_ref, T1, Robot.DH)
    
    TN = forward_kinematics(qSolNumerical, Robot.DH,6)
    T1
    e = norm(TN(1:3,4) - T1(1:3,4))
    
    
    solNum(i,1) = size(qSols,2);
    
    %     if size(qSols,2) < 8
    %         if isempty(qSols)
    %             indexNoSol = [indexNoSol;i];
    %             numNoSol = numNoSol + 1;
    %         else
    %
    %             indexLoseSol = [indexLoseSol;i];
    %             numLoseSol = numLoseSol + 1;
    %         end
    %
    %     else
    %         numindex8Solu = numindex8Solu + 1;
    %         index8Solu = [index8Solu;i];
    %     end
    %
    %     q_sol = IK_choose_one_sol_norm(qSols, q_in);
end

