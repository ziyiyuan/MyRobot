% ���Զ���ѧ��غ���
% �汾��V1.0����д��2020/8/27���޸���2020/8/27�����ߣ�ziyi
%
clc
clear all
robotType = 'I5';
Robot = get_cad_model_para(robotType);

e = [[]];
q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';

motionPara.q = q;
motionPara.qd = qd;
motionPara.qdd = qdd;
%%
% test ID_Newton_Euler // ����ÿ���ؽڵ��������أ�����ţ��ŷ���͵���ţ��ŷ����
% linear model and ordinary model
[tau1,ft] = ID_Newton_Euler(Robot, motionPara, 'linear');
tau2 = ID_Newton_Euler(Robot, motionPara,'linear'); %
[tau3,ft2] = ID_Newton_Euler(Robot, motionPara);
e{1} = ft - ft2;

%% test rotor inertia
[tau1,ft1] = ID_Newton_Euler_with_rotor_inertia(Robot,motionPara);
[tau2,ft2] = ID_rotor_inertia(Robot, motionPara);
[tau3,ft3] = ID_Newton_Euler(Robot, motionPara);
e{2} = tau1 - (tau2 + tau3);



%% identification model
pp = Robot.Para_cad;

% test External model // wrench in base 
wrench1 = cal_identification_matrix(Robot, motionPara,'External') * pp;
wrench = get_wrench_from_diff_identificationModel(Robot, motionPara, 'External');
e{3} = wrench - wrench1;

% test Internal model // torque in joint
tau1 = cal_identification_matrix(Robot, motionPara,'Internal') * pp;
tau = get_wrench_from_diff_identificationModel(Robot, motionPara, 'Internal');
e{4} = tau - tau1;

% test MiniPara // Internal model ������
HH =cal_identification_matrix(Robot, motionPara,'External');
[index, MDP] = get_mini_para_set_analytic(Robot,'External');
e{5} = HH(:,index) * MDP - wrench;

wrench1 = get_base_wrench_base_miniPara(Robot,motionPara);
e{6} = wrench1 - wrench;

% test MiniPara // External model ������ 
HH =cal_identification_matrix(Robot, motionPara,'Internal');
[index, MDP] = get_mini_para_set_analytic(Robot,'Internal');
e{7} = HH(:,index) * MDP - tau;



for i = 1:1:length(e)
    if norm(e{i}) >  1e-10
        error(['cal error' num2str(i)])
    end
end






