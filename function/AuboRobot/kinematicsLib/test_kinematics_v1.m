% 测试运动学相关函数
% 版本号V1.0，编写于2020/8/27，修改于2020/8/27，作者：ziyi
%
clc
clear all
robotType = 'I5';
Robot = get_cad_model_para(robotType);

%%
q_in = ones(6,1)*2;
q_in = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
qd = q_in;

aubo_i = robot_toolbox_model();
Tadd = homogeneous_transfer(Robot.DH(1,1), Robot.DH(1,2), Robot.DH(1,3), Robot.DH(1,4))

T1 = forward_kinematics(q_in, Robot.DH)
T0_1 = forward_kinematics(q_in,Robot.DH,1)
T = forward_kinematics(q_in,Robot.DH,6)
T = aubo_i.fkine(q_in)

qSols = IK_all_sol_analytic(T, Robot.DH, Robot.Limit.q)
qSols = IK_all_sol_analytic(T, Robot.DH)
q_sol = IK_choose_one_sol_norm(qSols, q_in)

J1 = cal_jacobian(q_in, Robot.DH,Robot.Para.DP.c)
J2 = cal_jacobian(q_in, Robot.DH)
J2{6}
J2 = aubo_i.jacob0(q_in)
Jdot = cal_jacobian_dot(q_in,qd,Robot.DH)




% test ik
qLimit = Robot.Limit.q;
a2 = Robot.Para.KP.a(3);
a3 = Robot.Para.KP.a(4);
d5 = Robot.Para.KP.d(5);

q_all = [];
q_all_1 = [];
q_all_2 = [];
index = [];
j = 0;
k = 0;


  q = [ -1.7550   -2.2192   -1.0521    0.6865    0.0012    2.9225]
for i = 1:1:10000
    if i == 8776
        flag = 1;
    end
    
    q_in = qq(i,:)';
    
%     q_in = -limit + 2* limit * rand(6,1);
    
    T1 = forward_kinematics(q_in, Robot.DH);
    
    qSols = IK_all_sol_analytic(T1, Robot.DH);
    q = q_in;
    q_all = [q_all,q_in];
    if size(qSols,2) < 8
        q_all_2 = [q_all_2,q_in];
        index = [index;i-1];
        
        j = j+1;
        J(j) = (a2 * sin(q(2)) + a3 * sin(q(2) - q(3)) + d5 * sin(q(2) - q(3) + q(4)))*sin(q(3))*sin(q(5));
    else
        k = k +1;
        JJ(k) = (a2 * sin(q(2)) + a3 * sin(q(2) - q(3)) + d5 * sin(q(2) - q(3) + q(4)))*sin(q(3))*sin(q(5));
    end
    
    if isempty(qSols)
        q_all_1 = [q_all_1,q_in];
        
    end
    
    q_sol = IK_choose_one_sol_norm(qSols, q_in);
end

