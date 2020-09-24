% 运动学标定移入算法库
clc
clear all
close all
%% initiallize
robotType = 'I5';
Robot = get_cad_model_para(robotType);
%%
caliMethod = 'leica';
caliBeta = 1;
useEstPara   = 0;

filename = 'Leica0922';
% filename = 'Leica0921_1';
% filename = 'Leica0918';
% filename = 'Leica0101';

%% load data
nom.joints = load(['Data\',filename,'\CAL.txt']); % six joint angles
nom.mesuredData = load(['Data\',filename,'\mesurements.txt'])/1000;
if strcmp(filename,'Leica0922')
    cBase = [359.962	1010.3215	112.9991	0.5985	0.1212	-4.7401];
    cTool = [-13.1097	-12.7803	39.1872 	90	0	-90.0002];
elseif strcmp(filename,'Leica0921_1')
    cBase = [218.749	481.9519	123.2761	0.5971	-0.3439	-4.3988];
    cTool = [-13.2527	-12.5386	37.3099	90	0	-90.0003];
elseif strcmp(filename,'Leica0920_1')
    nom.mesuredData = nom.mesuredData(:,1:3);
    cBase = [218.8764	481.848	123.265	0.5963	-0.3443	-4.4069];
    cTool = [-13.2574	-12.5234	37.3126	90	0	-90.0002];
    
elseif strcmp(filename,'Leica0917')
    cBase = [257.7003	267.1985	120.9809	0.5647	0.0715	-3.8581];
    cTool = [-12.9308	-12.5244	37.1128	 90 	0	-90.0002];
elseif strcmp(filename,'Leica0918')
    cBase = [199.5783 488.5875 123.0136 0.548 -0.0512 -3.7073];
    cTool = [-12.8753 -12.5266 37.1857 90 0 -90.0002];
    nom.mesuredData = nom.mesuredData(:,4:6);
end
nom.baseInLeica.pos = cBase(1:3)'/1000; % parameters with error.
nom.baseInLeica.rpy = cBase(4:6)'.*pi/180; % parameters with error.ZYX order,%rpy rz*ry*rx
nom.toolPosInFlange.pos = cTool(1:3)'/1000; % parameters with error.
nom.toolPosInFlange.rpy = cTool(4:6)'.*pi/180; % parameters with error.
% 变换欧拉角的顺序；%rpy rz*ry*rx
nom.baseInLeica.rpy = flipud (nom.baseInLeica.rpy);
nom.toolPosInFlange.rpy = flipud (nom.toolPosInFlange.rpy);

if strcmp(filename,'Leica0101')
    nom.joints = nom.joints * pi/180;
    nom.baseInLeica.pos = [3.783869 1.88846 0.06325]';
    nom.baseInLeica.rpy = [-1.1047 0.0103 0.0048]';  %rpy rz*ry*rx
    nom.toolPosInFlange.pos = [-0.0031 0.0123 0.0265]';
end

nom.baseInLeica.rot = RotZ(nom.baseInLeica.rpy(1))*RotY(nom.baseInLeica.rpy(2))*RotX(nom.baseInLeica.rpy(3));
nom.toolPosInFlange.rot = eye(3);

nom.joint_num = size(nom.joints,1);
nom.DH = Robot.DH;
nom.DOF = Robot.DOF;

eBefore = [];
if 1 % fk test right;
    for jn = 1:1:size(nom.joints,1)
        T = forward_kinematics(nom.joints(jn,:)', Robot.DH, 6);
        Twb = RpToTrans(nom.baseInLeica.rot, nom.baseInLeica.pos);
        Tft = RpToTrans(nom.toolPosInFlange.rot, nom.toolPosInFlange.pos);
        Tend = Twb * T * Tft
        eBefore = [eBefore; abs(Tend(1:3,4) - nom.mesuredData(jn,:)')];
    end
end
criterBefore.avgErr=sum(eBefore)/(nom.joint_num * 3)*1000;
criterBefore.maxErr = max(eBefore)*1000;
criterBefore.rmsErr = sqrt(eBefore'*eBefore/(nom.joint_num * 3))*1000;
criterBefore

%-----------------------------------------------------
% leica test
%-----------------------------------------------------
dh_para_mask = ones(6,5); % alpha a d theta beta
% dh_para_mask = [ 0     1     1     1     0     0
%     0     0     0     0     0     0
%     0     0     0     0     0     0
%     0     0     0     0     0     0
%     0     0     0     0     0     0]';
if(caliBeta == 0)
    dh_para_mask(:,5) = 0; % set by API
end
%-----------------------------------------------------
% rough estimation of 9 envirenment parameters
%-----------------------------------------------------
[xyz, rpy, pt] = FuncRoughCalibPositionMeas(nom);
xyz_rpy_pt_rough = [xyz', rpy', pt'];
Leica=[nom.baseInLeica.pos' nom.baseInLeica.rpy' nom.toolPosInFlange.pos'];
% using rough estimation
if(useEstPara)
    nom.baseInLeica.pos = xyz;
    nom.baseInLeica.rpy = rpy;
    nom.toolPosInFlange.pos = pt;
    nom.baseInLeica.rot = RotZ(nom.baseInLeica.rpy(1))*RotY(nom.baseInLeica.rpy(2))*RotX(nom.baseInLeica.rpy(3));
end
%-----------------------------------------------------
% accurate estimation of 9 envirenment parameters
%-----------------------------------------------------
sim = FuncDhCalibPositionMeas(nom, dh_para_mask);
%-----------------------------------------------------
%-----------------------------------------------------
% dlmwrite('mesurements.txt', mesurements, 'precision', '%5f', 'delimiter', ',')