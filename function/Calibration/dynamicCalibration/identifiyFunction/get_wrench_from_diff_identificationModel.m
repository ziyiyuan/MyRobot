function wrench = get_wrench_from_diff_identificationModel(Robot, motionPara, identificationModel)
%  varargin = T01
%Returns  1: 如果，辨识模型为内部模型，输出每个关节的关节力矩；
%         2: 如果辨识模型为外部模型，输出基坐标系处的力和力矩；
[tau,ft] = ID_Newton_Euler(Robot,motionPara); %% 返回每个关节处的力矩和力；

if strcmp(identificationModel,'External') % 输出base坐标系处的力和力矩；
    T01 = forward_kinematics(motionPara.q, Robot.DH, 1);
    R = T01(1:3,1:3);
    p = T01(1:3,4);
    wrench = ([R  zeros(3,3); skew(p)*R R])*ft(:,1); % 关节1 处的力矩映射到base；
elseif  strcmp(identificationModel,'Internal')
    wrench = tau;
else
    message('choose model');
end
end
