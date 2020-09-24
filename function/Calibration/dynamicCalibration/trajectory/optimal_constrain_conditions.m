function [c,ceq] = optimal_constrain_conditions(Robot,Traj, motionParaCoeff, identificationModel, sampleRate)
global optimal_res
% motionPara = cal_motionPara_from_fourier_series(Robot, Traj, motionParaCoeff, sampleRate);
motionPara = optimal_res.motionPara;
c = [];
ceq = [];
if strcmp(identificationModel,'Internal')
    %     torLimit = Robot.Limit.torque;
    %     posLimit = Robot.Limit.q;
    torLimit = [150,150,150,40,40,40];
    posLimit = [3,3,3,3,3,3];
elseif strcmp(identificationModel,'External')
    torLimit = Robot.Limit.sensor;  %%%input the sensor force limit
    posLimit = Robot.Limit.sensorP; %% 防止碰到基座传感器；
end

ARM_DOF = Robot.DOF;
velLimit = Robot.Limit.qd;
accLimit = Robot.Limit.qdd;

for i = 1 : size(motionPara.q,2)
    for j = 1:ARM_DOF
        %% kinematics constraints 位置， 速度， 加速度约束；% 6 * 6
        c(50 * i + 6 * j + 1 - 56) = motionPara.q(j,i) - posLimit(j);
        c(50 * i + 6 * j + 2 - 56) = -posLimit(j) - motionPara.q(j,i);
        c(50 * i + 6 * j + 3 - 56) = motionPara.qd(j,i) - velLimit(j);
        c(50 * i + 6 * j + 4 - 56) = -velLimit(j) - motionPara.qd(j,i);
        c(50 * i + 6 * j + 5 - 56) = motionPara.qdd(j,i) - accLimit(j);
        c(50 * i + 6 * j + 6 - 56) = -accLimit(j) - motionPara.qdd(j,i);
    end
    %% torque constraints of each joint
    jointPara.q = motionPara.q(:,i);
    jointPara.qd = motionPara.qd(:,i);
    jointPara.qdd = motionPara.qdd(:,i);
    
    %     torque = get_wrench_from_diff_identificationModel(Robot, jointPara, identificationModel)
    torque = get_joint_torque_base_miniPara(Robot,jointPara);
    for k = 1:ARM_DOF %6 * 2
        c(50 * i + 2*k -1 - 14) = (torque(k)) - torLimit(k);
        c(50 * i + 2*k - 14) =  -torLimit(k) - (torque(k));
    end
    
    %% collision free constraints
    q = motionPara.q(:,i);
    %%
    s1 = sin(q(1)); s2 = sin(q(2)); s3 = sin(q(3)); s4 = sin(q(4)); s5 = sin(q(5)); s6 = sin(q(6));
    c1 = cos(q(1)); c2 = cos(q(2)); c3 = cos(q(3)); c4 = cos(q(4)); c5 = cos(q(5)); c6 = cos(q(6));
    
    a2 = Robot.Para.KP.a(3); a3 = Robot.Para.KP.a(4);
    d1 = Robot.Para.KP.d(1); d2 = Robot.Para.KP.d(2); d5 = Robot.Para.KP.d(5); d6 = Robot.Para.KP.d(6);
    
    tmp0 = c2*s3;
    tmp1 = c3*s2;
    tmp2 = c1*tmp0 - c1*tmp1;
    tmp3 = c2*c3;
    tmp4 = s2*s3;
    tmp5 = c1*tmp3 + c1*tmp4;
    tmp6 = a2*s2;
    tmp7 = s1*tmp0 - s1*tmp1;
    tmp8 = s1*tmp3 + s1*tmp4;
    tmp9 = tmp3 + tmp4;
    tmp10 = tmp0 - tmp1;
    X = a3*tmp2 - c1*tmp6 + d2*s1 + d5*(c4*tmp2 - s4*tmp5) + d6*(c5*s1 + s5*(c4*tmp5 + s4*tmp2));
    Y = a3*tmp7 - c1*d2 + d5*(c4*tmp7 - s4*tmp8) - d6*(c1*c5 - s5*(c4*tmp8 + s4*tmp7)) - s1*tmp6;
    Z = a2*c2 + a3*tmp9 + d1 + d5*(c4*tmp9 + s4*tmp10) - d6*s5*(c4*tmp10 - s4*tmp9);
    %%
    %     T0_6 = FK(q);
    %     X = T0_6(1,4);
    %     Y = T0_6(2,4);
    %     Z = T0_6(3,4);
    %
    c(50 * i + 13 - 14) =  - 0.1 - Z;           % the end position is higher than -0.1m
    c(50 * i + 14 - 14) =  0.23^2 - X^2 - Y^2;  % the end will not located in a circle of radius 0.23m
end
end
