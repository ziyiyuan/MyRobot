function [est_xyz,est_rpy,est_pt,realDh]=FuncDhCalibPositionMeas(nom, dh_para_mask)

identifiable_mask=dh_para_mask;

para_num = 39;
env_para_num = 9;
frame_pos_idx = 1;
tool_pos_idx = 4;
% Pw7 = Rw0 * R06 * P67 + Rw0 * p06 + pw0
% f(x+dx) = Pm = f(x) + f(x)'*dx;
% dx = (pm-f(x))/f(x)'; x = x + dx;
% d(pm) = [dx,dy,dz], J(pm)= Pm' = eye(3);
% d(rm) = [dr dp dy], J(rm) = Rw0'*(R06*Pt+P06) = Rw0' * P0t;
% d(pt) = [dx,dy,dz], J(pt) = Rw0*R06*Pt' = Rw0*R06*eye(3);
% d(dh) = [dalpha dA dD dtheta dbeta ], J(dh) = Rw0*P0t';
% dx=[d(pm) d(rm) d(pt) d(dh1) ... d(dh6)' ]'
% total 3 + 3 + 3 + 4*6 + 6 = 33 + 6=39.
%-----------------------------------------------------
global Robot

DH = Robot.DH;

rpy = nom.baseInLeica.rpy;
nom.residualPos = zeros(nom.joint_num,1);
for data_idx = 1:nom.joint_num
    % nominal p(i), rot(i).
    T06=eye(4);
    for i=1:6
        Tii = transfer(nom.DH(1,1),nom.DH(1,2),nom.DH(1,3),nom.DH(1,4) + nom.joints(data_idx,i));
        T06 = T06*Tii;
        P{i}= T06(1:3,4);
        R{i}= T06(1:3,1:3);
    end
    P{7}= R{6}*nom.toolPosInFlange + P{6};
    JFrame = [eye(3) dRotZ(rpy(1))*RotY(rpy(2))*RotX(rpy(3))*P{7} RotZ(rpy(1))*dRotY(rpy(2))*RotX(rpy(3))*P{7} RotZ(rpy(1))*RotY(rpy(2))*dRotX(rpy(3))*P{7}];
    JPt = R{6};
    JF = JPt;
    
    if data_idx == 1
        valid_dh_idx=[];
    end
    
    for i=1:6
        if i==1
            JAlpha{i}=zeros(3,1);
            JBeta{i}=zeros(3,1);
            JA{i}=zeros(3,1);
        else
            JAlpha{i}=cross(R{i-1}(:,1),P{7}-P{i-1});% ÈÆ x ÖáÐý×ª alpha 
            JBeta{i}=cross(R{i-1}(:,2),P{7}-P{i-1}); % ÈÆ y ÖáÐý×ª beta 
            JA{i}=R{i-1}(:,1); % ÈÆ x ÖáÆ½ÒÆa
        end
        JJ{i}=cross(R{i}(:,3),P{7}-P{i});
        JD{i}=R{i}(:,3);
        
        if dh_para_mask(i,1) ~= 0
            JF = [JF JAlpha{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-4];
            end
        end
        if dh_para_mask(i,2) ~= 0
            JF=[JF JA{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-3];
            end
        end
        if dh_para_mask(i,3) ~= 0
            JF=[JF JD{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-2];
            end
        end
        if dh_para_mask(i,4) ~= 0
            JF=[JF JJ{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-1];
            end
        end
        if dh_para_mask(i,5) ~= 0
            JF=[JF JBeta{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i];
            end
        end
    end
    
    nom.Phi{data_idx} = [JFrame nom.baseInLeica.rot*JF];
    Pw7 = nom.baseInLeica.rot*P{7}+nom.baseInLeica.pos;
    
    if data_idx == 1
        W = nom.Phi{data_idx};
        Y = Pw7 - nom.mesuredData(data_idx,1:3)';
    else
        W=[W;nom.Phi{data_idx}];
        Y=[Y;Pw7 - nom.mesuredData(data_idx,1:3)'];
    end
end

[Qw, Rw]=qr(W);
Rwii=diag(Rw);
qr_rii_thr=size(Rw,2)*eps*max(abs(diag(Rw)));
% if all dh para are selected, then
% without beta=>1     2    11    15, a1 A1 D1 J1 D3 D4 D6 J6.
% with beta=>1     2     5    10    13    18    25    30, aditional b1 b2 b5 b6 (-1).

% // todo: if env para unidentifiable
% update valid dh para mask and W.
excludeColumns = find(Rwii<qr_rii_thr & Rwii>-qr_rii_thr)' ;


for i=length(excludeColumns):-1:1
    W=[W(:,1:excludeColumns(i)-1) W(:,excludeColumns(i)+1:end)];
    if excludeColumns(i) > env_para_num
        identifiable_mask(valid_dh_idx(excludeColumns(i)-env_para_num))=0;
    end
end
identifiable_mask'

% para estimation.Y+WX=Y-W(W'W)W'Y
estPara = -inv(W'*W)*W'*Y;

% update valid dh para.
realDh = [nom.DH, zeros(6,1)];% full dh para
valid_dh_para_num=0;
for i=1:30
    if identifiable_mask(i) ~= 0
        valid_dh_para_num = valid_dh_para_num + 1;
        realDh(i)=realDh(i)+estPara(env_para_num+valid_dh_para_num);
    end
end
% vpa(estPara)
%
realDhComp=[realDh(:,1)'*180/pi realDh(:,2:3)'*1000 realDh(:,4:5)'*180/pi]
% realDhComp
% dhLeica=[
%     -0.0319 0.5461 97.6632 180.0411;
%     -90.0137 1.2928 121.2651 -90.2571;
%     179.9181 409.1147 -0.3706 0.0365;
%     180.0277 376.6818 -0.2375 -90.1281;
%     -89.9502 -0.2694 102.0559 0.8172;
%     89.6058 -1.6541 93.8492 -0.1112]
%
% dhYzy=[
%     0         0  180.0000   98.5000;
%     -89.9981   -0.7140  -89.9740  121.6187;
%     179.9249  409.0894    0.0381         0;
%     179.9831  376.6947  -90.0030         0;
%     -89.9790    0.2379    0.0018  102.7185;
%     89.9965    0.2233         0   94.0000];
% dhYzy=[dhYzy(:,1:2) dhYzy(:,4) dhYzy(:,3)]

% ptLeica=[-2.9016 11.8619 26.6352]'/1000;
%-----------------------------------------------------
% average err & RMS
%-----------------------------------------------------
sim.dhArr = realDhComp;
sim.baseInLeica.pos = nom.baseInLeica.pos + estPara(1:3);
sim.baseInLeica.rpy = nom.baseInLeica.rpy + estPara(4:6);
sim.toolPosInFlange=nom.toolPosInFlange+estPara(7:9);

sim.baseInLeica.rot=rotZ(sim.baseInLeica.rpy(1))*rotY(sim.baseInLeica.rpy(2))*rotX(sim.baseInLeica.rpy(3));

for data_idx=1:nom.joint_num
    T06=eye(4);
    for i=1:6
        Tii=[RotY(realDh(i,5)) zeros(3,1);0 0 0 1]* transfer(nom.DH(1,1),nom.DH(1,2),nom.DH(1,3),nom.DH(1,4) + nom.joints(data_idx,i));;
        T06=T06*Tii;
    end
    P0t=T06(1:3,1:3)*sim.toolPosInFlange+T06(1:3,4);
    Pw7 = sim.baseInLeica.rot*P0t + sim.baseInLeica.pos;
    currErr(3*data_idx-2:3*data_idx,1) = abs(Pw7 - nom.mesuredData(data_idx,1:3)');
    
    %     currErr=abs(norm(P0t-nom.dynalogInBase.pos)-nom.measuredDis(data_idx));
    %     if maxErr < currErr
    %         maxErr = currErr;
    %     end
    %     avgErr=avgErr+currErr;
    % %     rmsErr=rmsErr+currErr^2;
end
criter.avgErr=sum(currErr)/(nom.joint_num * 3)*1000;
criter.maxErr = max(currErr)*1000;
criter.rmsErr = sqrt(currErr'*currErr/(nom.joint_num * 3))*1000;
criter
% rmsErr=(rmsErr./[nom.joint_num-1 size(W,1)-size(W,2) size(Rw,1)-size(Rw,2)]).^0.5
% rmsErr1=norm(Y+W*estPara)./[nom.joint_num-1 size(W,1)-size(W,2) size(Rw,1)-size(Rw,2)].^0.5
% est_xyz=nom.dynalogInBase.pos;
% est_pt=sim.toolPosInFlange;
est_xyz = sim.baseInLeica.pos*1000;
est_rpy = sim.baseInLeica.rpy*180/pi;
est_pt = sim.toolPosInFlange*1000;
real_mt_para = [est_xyz,est_rpy,est_pt]
end