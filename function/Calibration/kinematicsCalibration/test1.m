clear;close all;clc;
%% initialize
global Robot 
leicaTestEna=1;
dynalogTestEna=0;

caliBeta = 1;
useEstPara = 1;
%%
%-----------------------------------------------------
% nominal data from Leica Report before calibration
%-----------------------------------------------------
if leicaTestEna == 1
    nom.joints=load('./Data/Leica/CAL.txt')*pi/180; % six joint angles
    %     nom.toolPosInFlange=[-3.0974 12.2589 26.4622]'/1000; % parameters with error.
    %     nom.baseInLeica.pos=[3783.8695 1888.4613 63.2584]'/1000; % parameters with error.
    %     nom.baseInLeica.rpy=[-63.2966 0.5894 0.2756]'*pi/180; % parameters with error.ZYX order
    nom.baseInLeica.pos = [3.783869 1.88846 0.06325]';
    nom.baseInLeica.rpy = [-1.1047 0.0103 0.0048]';  %rpy rz*ry*rx
    nom.toolPosInFlange = [-0.0031 0.0123 0.0265]';
    
    nom.baseInLeica.rot = RotZ(nom.baseInLeica.rpy(1)) * RotY(nom.baseInLeica.rpy(2)) * RotX(nom.baseInLeica.rpy(3));
    nom.mesuredData=load('./Data/Leica/mesurements.txt')/1000;
else
    if robotI5==1
        %         filePath='dynalog\i5DynacalTestData\19071093\DC\19071093\';
        filePath='dynalog\i5DynacalTestData\19071101\DC\19071101\';
        %         filePath='dynalog\i5DynacalTestData\19071102\DC\19071102\';
    else
        filePath='dynalog\i7DynacalTestData\19081001\calidata\';
    end
    nom.joints=load(strcat(filePath,'point60.txt'))*pi/180; % six joint angles
    nom.joints=nom.joints(:,2:end);
    fid=fopen(strcat(filePath,'DynaCal1.msr'));
    fgetl(fid);
    nom.measuredDis=[];
    for i=1:60
        nom.measuredDis=[nom.measuredDis;str2double(fgetl(fid))/1000];
    end
    fclose(fid);
    
    nom.toolPosInFlange = [ 0.0001   -0.1035    0.0956]';
    nom.dynalogInBase.pos=[0.0407   -1.1701   -0.0845]';

end
nom.joint_num = size(nom.joints,1);
nom.DH = Robot.DH;
%-----------------------------------------------------
% leica test
%-----------------------------------------------------
if leicaTestEna
    dh_para_mask=ones(6,5); % alpha A D theta beta
    %     dh_para_mask = [
    %      0     0     0     0     0
    %      0     0     0     0     0
    %      0     0     0     0     0
    %      0     0     0     0     0
    %      0     0     0     0     0
    %      0     0     0     0     0];
    if(caliBeta == 0)
        dh_para_mask(:,5) = 0;
    end
    %-----------------------------------------------------
    % rough estimation of 9 envirenment parameters
    %-----------------------------------------------------
    [xyz, rpy, pt] = FuncRoughCalibPositionMeas(nom);
    xyz_rpy_pt_rough=[xyz', rpy', pt'];
    Leica=[nom.baseInLeica.pos' nom.baseInLeica.rpy' nom.toolPosInFlange'];
    % using rough estimation
    if(useEstPara)
        nom.baseInLeica.pos = xyz;
        nom.baseInLeica.rpy = rpy;
        nom.toolPosInFlange = pt;
        nom.baseInLeica.rot = RotZ(nom.baseInLeica.rpy(1))*RotY(nom.baseInLeica.rpy(2))*RotX(nom.baseInLeica.rpy(3));
    end
    %-----------------------------------------------------
    % accurate estimation of 9 envirenment parameters
    %-----------------------------------------------------
    sim = FuncDhCalibPositionMeas(nom, dh_para_mask);
end
%-----------------------------------------------------
% dynalog test with dammy data from Leica test.
%-----------------------------------------------------
if dynalogTestEna
    dh_para_mask=ones(5,6); % alpha A D theta beta
%     dh_para_mask = [ 0     1     1     1     0     0
%                      0     0     0     0     0     0
%                      0     0     0     0     0     0
%                      0    0   0     0     0     0
%                      0     0     0    0     0     0];
    if(caliBeta == 0)
        dh_para_mask(5,:)=0; % set by API
    end
    
    if leicaData == 1
        nom.dynalogInBase.pos=-nom.baseInLeica.rot'*nom.baseInLeica.pos;
        for i=1:nom.size
            nom.measuredDis(i)=norm(nom.mesuredData(i,1:3));
        end
        xyz_pt_init=[-nom.baseInLeica.rot'*nom.baseInLeica.pos; nom.toolPosInFlange]'
    end
    
    [xyz,pt]=FuncRoughCalibDistanceMeas(nom);
    xyz_pt_rough=[xyz;pt]'

    % using rough estimation
    if(useEstPara)
        nom.toolPosInFlange=pt;
        nom.dynalogInBase.pos=xyz;
    end
    
    % using leica estimation
    %     nom.baseInLeica.pos=[3784.0678 1887.7354 62.6907]'/1000;
    %     nom.baseInLeica.rpy=[-63.378 0.3047 0.2214]'*pi/180;
    %     nom.baseInLeica.rot=rotZ(nom.baseInLeica.rpy(1))*rotY(nom.baseInLeica.rpy(2))*rotX(nom.baseInLeica.rpy(3));
    %     nom.dynalogInBase.pos=-nom.baseInLeica.rot'*nom.baseInLeica.pos;
    %     nom.toolPosInFlange=[-2.9016 11.8619 26.6352]'/1000;
    
    [xyz,pt,dhArr]=FuncDhCalibDistanceMeas(nom, dh_para_mask);
    xyz_pt_dynalog=[xyz;pt]'
end