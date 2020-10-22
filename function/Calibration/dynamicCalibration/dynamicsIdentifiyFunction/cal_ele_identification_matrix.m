function HH = cal_ele_identification_matrix(Robot, motionPara,identificationModel)
%%% obtain identification matrix of internal model and external model

%Inputs : motionPara: the struct of q qd qdd
%         identificationModel ：'External' or 'Internal'
%Returns  1: 如果，辨识模型为内部模型，输出每个关节的关节力矩辨识矩阵，size(HH) = 6 * 60；
%         2: 如果辨识模型为外部模型，输出基坐标系处的力和力矩；size(HH) = 6 * 60

% Example Inputs:
%
% q = [0.033225 -0.088711 0.970674 -0.859352 -0.050902 -0.853022]';
% qd = [1.033225, -0.088711, 1.970674, -0.859352, -1.050902,  -1.853022]';
% qdd = [1.033225, -1.088711, 1.970674, -1.859352, -1.050902, -1.853022]';
% motionPara.q = q;
% motionPara.qd = qd;
% motionPara.qdd = qdd;
%
% HH = IdentificationMatrix(motionPara,identificationModel)


ARM_DOF = Robot.DOF;
gravity = Robot.gravity;
DH = Robot.DH;


%************load and set CAD data************
q = motionPara.q;
qd = motionPara.qd;
qdd =motionPara.qdd;
% fk
T = [[]];R = [[]];p = [[]];
for i = 1:1:ARM_DOF
    T{i} = homogeneous_transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
    R{i}= T{i}(1:3,1:3);
    p{i} = T{i}(1:3,4);
end

% ************* cal w wd vd in link coordinate***********
w = [[]];  w0 = [0 0 0]';
wd = [[]]; wd0 = [0 0 0]';
vd = [[]]; vd0 =  - [gravity(1);gravity(2);gravity(3)];

e = [0 0 1]';
for i = 1:ARM_DOF
    if i == 1
        w{i} =  R{i}'*w0 + qd(i)*e;
        wd{i} = R{i}'*wd0 + qdd(i)*e + cross(R{i}'*w0, qd(i)*e);
        vd{i} = R{i}'*(vd0 + cross(w0, cross(w0, p{i})) + cross(wd0, p{i}));
    else
        w{i} =  R{i}'*w{i-1} + qd(i)*e;
        wd{i} = R{i}'*wd{i-1} + qdd(i)*e + cross(R{i}'*w{i-1},qd(i)*e);
        vd{i} = R{i}'*(vd{i-1} + cross(w{i-1}, cross(w{i-1}, p{i})) + cross(wd{i-1}, p{i}));
    end
end

%*******************Use reaction method to obtain force and torque of point
Twb = eye(4);
T0{1} = Twb*T{1};
for i = 2:ARM_DOF
    T0{i} = T0{i-1} * T{i};
end

%** identification matrix and identification para
% H{i}: link i 运动在 A 点产生的力矩为 H{i}*Phai{i} = TAi*A*Phai{i};
%Phai in order I  MS  M
H = [[]];
P = [[]];

if strcmp(identificationModel,'External')
    for i = 1:ARM_DOF
        RR = [T0{i}(1:3,1:3) zeros(3,3); skew(T0{i}(1:3,4))*T0{i}(1:3,1:3) T0{i}(1:3,1:3)]; %TAi
        A = [zeros(3,6) skew(wd{i})+skew(w{i})*skew(w{i}) vd{i};skewStar(wd{i})+skew(w{i})*skewStar(w{i}) -skew(vd{i}) zeros(3,1)]; %Wii
        H{i} = RR*A;
    end
    HH = [H{1} H{2} H{3} H{4} H{5} H{6}];
else % cal torque in joint coordinate
    HA = [[]];
    for i = 1:ARM_DOF
        T1 = eye(4);
        A = [zeros(3,6) skew(wd{i})+skew(w{i})*skew(w{i}) vd{i};skewStar(wd{i})+skew(w{i})*skewStar(w{i}) -skew(vd{i}) zeros(3,1)]; %Wii link I 运动在i关节产生的力
        for j = i:-1:1
            if(i == j)
                HA{j,i} = A;
            else
                T1 = T{j+1}*T1;
                RR = [T1(1:3,1:3) zeros(3,3); skew(T1(1:3,4))*T1(1:3,1:3) T1(1:3,1:3)]; %TAi
                HA{j,i} = RR * A;
            end
        end
    end
    
    HH = [];
    for i= 1:1:6
        S1 = [];
        for j = i:6
            S1 = [S1 [0,0,0,0,0,1] * HA{i,j}]; % J link 运动 在 joint i 处产生的力矩
        end
        HH = [HH;zeros(1,10*(i-1)) S1];
    end
end

% %**** identification para
% for i = 1:ARM_DOF
%     P{i} = [J{i}(1,1) J{i}(1,2) J{i}(1,3) J{i}(2,2) J{i}(2,3) J{i}(3,3) MS{i}(1) MS{i}(2) MS{i}(3) M(i)]';
% end
% pp = [P{1}' P{2}' P{3}' P{4}' P{5}' P{6}']';
% tau = HH*pp
% tau_1 = ID_NewtonEuler(motionPara)
%
% %% cal torque in joint coordinate
%     for i= 1:1:6
%         FT{i} = 0;
%         for j = i:6
%             FT{i} =  FT{i} + HA{i,j} * P{j}; % JOINT TORQUE AND FORCE in joint
%         end
%     end
%
% tau_b = ([R{1} zeros(3,3); skew(p{1})*R{1} R{1}])*FT{1} % force and torque in base
% identificationModel = 'External';
% tau_b1 = ID_NewtonEuler(motionPara)
end

