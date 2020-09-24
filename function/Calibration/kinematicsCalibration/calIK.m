% 运动学标定移入算法库
clc
clear all
%% initiallize
robotType = 'I5';
Robot = get_cad_model_para(robotType);
%%
ziyi = 0;
if ziyi
    load('sim.mat')
    nom.baseInLeica.rot = sim.baseInLeica.rot;
    nom.baseInLeica.pos = sim.baseInLeica.pos;
    nom.toolPosInFlange.pos = sim.toolPosInFlange;
    DHComp = sim.dhArr;
else
    %% load data    
    cBase = [218.749	481.9519	123.2761	0.5971	-0.3439	-4.3988];
    cTool = [-13.2527	-12.5386	37.3099	90	0	-90.0003];
    
    nom.baseInLeica.pos = cBase(1:3)'/1000; % parameters with error.
    nom.baseInLeica.rpy = cBase(4:6)'.*pi/180; % parameters with error.ZYX order,%rpy rz*ry*rx
    nom.toolPosInFlange.pos = cTool(1:3)'/1000; % parameters with error.
    nom.toolPosInFlange.rpy = cTool(4:6)'.*pi/180; % parameters with error.
    % 变换欧拉角的顺序；%rpy rz*ry*rx
    nom.baseInLeica.rpy = flipud (nom.baseInLeica.rpy);
    nom.toolPosInFlange.rpy = flipud (nom.toolPosInFlange.rpy);
    
    nom.baseInLeica.rot = RotZ(nom.baseInLeica.rpy(1))*RotY(nom.baseInLeica.rpy(2))*RotX(nom.baseInLeica.rpy(3));
    nom.toolPosInFlange.rot = eye(3);
    nom.DH = Robot.DH;
    nom.DOF = Robot.DOF;
    
    load('DH_LEICA.mat')
    DHComp = [DH_LEICA(:,1)*pi/180,DH_LEICA(:,[2,3])/1000,DH_LEICA(:,4)*pi/180]
end
    load('points.mat')

for i = 1:1:4
    quat = points(i,4:7');
    q_ref = points(i,8:13)';
    Rbf = QuattoRotMatrix(quat);
    Pwt = points(i,1:3)'./1000;
    
    Rwb = nom.baseInLeica.rot;
    Pwb = nom.baseInLeica.pos;
    Twb = RpToTrans(Rwb,Pwb);
    
    Rft = eye(3);
    Pft = nom.toolPosInFlange.pos;
    Tft = RpToTrans(Rft,Pft);
    
    Rwt = Rwb * Rbf * Rft;
    Twt = RpToTrans(Rwt,Pwt);
    
    Tbf = inv(Twb) * Twt * inv(Tft);
    % 解析解 norminal
    qSols = IK_all_sol_analytic(Tbf, Robot.DH);
    qSolA = IK_choose_one_sol_norm(qSols, q_ref);
    % 迭代解，comped
    if ziyi
        qSolN = IK_sol_numerical_for_leica(q_ref, Tbf, DHComp, Robot.Limit.q)
        TrealN = FK_DH_COMP( DHComp, qSolN);
    else
        qSolN = IK_sol_numerical(q_ref, Tbf, DHComp, Robot.Limit.q);
        TrealN = forward_kinematics(qSolN, DHComp, 6);
    end
    %
    TrealA = forward_kinematics(qSolA, Robot.DH, 6);
    quatN = RotMatrixtoQuat(TrealN(1:3,1:3));
    e = norm(qSolA - q_ref);
    eA(i,1) = norm(Twb * TrealA * Tft  - Twt)
    Tn = Twb * TrealN  * Tft;
    eB(i,1) = norm(Tn(1:3,4)  - Twt(1:3,4))
    
    seqA(i,:) = [TrealA(1:3,4)',quat, qSolA'];
    seqN(i,:) = [TrealN(1:3,4)',quatN', qSolN'];
end
dlmwrite('seqN.txt', seqN, 'precision', '%10f', 'delimiter', ',')











