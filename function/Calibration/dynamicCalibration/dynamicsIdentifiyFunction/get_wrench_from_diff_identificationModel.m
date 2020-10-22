function wrench = get_wrench_from_diff_identificationModel(Robot, motionPara, identificationModel)
%  varargin = T01
%Returns  1: �������ʶģ��Ϊ�ڲ�ģ�ͣ����ÿ���ؽڵĹؽ����أ�
%         2: �����ʶģ��Ϊ�ⲿģ�ͣ����������ϵ�����������أ�
[tau,ft] = ID_Newton_Euler(Robot,motionPara); %% ����ÿ���ؽڴ������غ�����

if strcmp(identificationModel,'External') % ���base����ϵ�����������أ�
    T01 = forward_kinematics(motionPara.q, Robot.DH, 1);
    R = T01(1:3,1:3);
    p = T01(1:3,4);
    wrench = ([R  zeros(3,3); skew(p)*R R])*ft(:,1); % �ؽ�1 ��������ӳ�䵽base��
elseif  strcmp(identificationModel,'Internal')
    wrench = tau;
else
    message('choose model');
end
end
