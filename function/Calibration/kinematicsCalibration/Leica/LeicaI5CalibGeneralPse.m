clear;close all;clc;
a2=0.408;a3=0.376;d1=0.0985;d2=0.1215;d4=0;d5=0.1025;d6=0.094;
%-----------------------------------------------------
% nominal data
%-----------------------------------------------------
nom.joints=load('CAL.txt')*pi/180; % six joint angles
nom.size=size(nom.joints,1);
nom.dhArr=getDhPara(0.408,0.376,0.0985,0.1215,0,0.1025,0.094);
nom.toolPosInBase=[-3.0974 12.2589 26.4622]'/1000; % parameters with error.
nom.baseInLeica.pos=[3783.8695 1888.4613 63.2584]'/1000; % parameters with error.
nom.baseInLeica.rpy=[-63.2966 0.5894 0.2756]'*pi/180; % parameters with error.ZYX order
nom.baseInLeica.rot=rotZ(nom.baseInLeica.rpy(1))*rotY(nom.baseInLeica.rpy(2))*rotX(nom.baseInLeica.rpy(3));
% RPY_Rot_transfer(RPY_Rot_transfer(nom.baseInLeica.rpy,1),2)-nom.baseInLeica.rpy % test ok
mesuredData=load('mesurements.txt')/1000;
%-----------------------------------------------------
% 0=f(x)-P=f(x0)+Phi*dx-P=Phi*dx+e.
% e=R(0,1)*P(1,7)+P(0,1)-Pmeas(0,7)
% d(e')/d(fr)=[de1/dfr1 de1/dfr2 ...;de2/dfr1 ...;de3/dfr1 ...]
% d(fr)=[dx dy dz drz dry drx]'
% alpha A D theta for Leica Results.
%-----------------------------------------------------
nom.residualPos=zeros(nom.size,1);
rpy=nom.baseInLeica.rpy;
for data_idx=1:nom.size
    T=eye(4);
    for i=1:6
        Tii=connectingRodTransfer(nom.dhArr(:,i),nom.joints(data_idx,i));
        T=T*Tii;
        P{i}=T(1:3,4);
        R{i}=T(1:3,1:3);        
    end
    P{7}=R{6}*nom.toolPosInBase+P{6};
    
    PhiFr=[eye(3) dRotZ(rpy(1))*rotY(rpy(2))*rotX(rpy(3))*P{7} rotZ(rpy(1))*dRotY(rpy(2))*rotX(rpy(3))*P{7} rotZ(rpy(1))*rotY(rpy(2))*dRotX(rpy(3))*P{7}];
    PhiPt=R{6};
    nom.Phi{data_idx}=PhiPt;
    for i=1:6
        if i==1
            PhiAlpha{i}=zeros(3,1);
            PhiBeta{i}=zeros(3,1);
            PhiA{i}=zeros(3,1);
        else
            PhiAlpha{i}=cross(R{i-1}(:,1),P{7}-P{i-1});
            PhiBeta{i}=cross(R{i-1}(:,2),P{7}-P{i-1});
            PhiA{i}=R{i-1}(:,1);
        end
        PhiJ{i}=cross(R{i}(:,3),P{7}-P{i});
        PhiD{i}=R{i}(:,3);
        nom.Phi{data_idx}=[nom.Phi{data_idx} PhiAlpha{i}];
        nom.Phi{data_idx}=[nom.Phi{data_idx} PhiA{i}];
        nom.Phi{data_idx}=[nom.Phi{data_idx} PhiD{i}];
        nom.Phi{data_idx}=[nom.Phi{data_idx} PhiJ{i}];
    end
 
    nom.residualPos(data_idx)=norm(nom.baseInLeica.rot*P{7}+nom.baseInLeica.pos-mesuredData(data_idx,1:3)');
    
    nom.Phi{data_idx}=[PhiFr kron(ones(1,27),nom.baseInLeica.pos)+nom.baseInLeica.rot*nom.Phi{data_idx}];
    if data_idx == 1
        W=nom.Phi{data_idx};
        Y=nom.baseInLeica.rot*P{7}+nom.baseInLeica.pos-mesuredData(data_idx,1:3)';
    else
        W=[W;nom.Phi{data_idx}];
        Y=[Y;nom.baseInLeica.rot*P{7}+nom.baseInLeica.pos-mesuredData(data_idx,1:3)'];
    end
end
(nom.residualPos-mesuredData(:,4))' % 10e-6

[Qw, Rw]=qr(W);
Rwii=diag(Rw);
qr_rii_thr=size(Rw,2)*eps*max(abs(diag(Rw))); 
excludeColumns=find(Rwii<qr_rii_thr & Rwii>-qr_rii_thr)' % 10    11    12    13    20    24    32    33, a1 A1 D1 J1 D3 D4 D6 J6.
realDh=nom.dhArr([2:4 1],:);
includeColums=[];
for i=length(excludeColumns):-1:1
    W=[W(:,1:excludeColumns(i)-1) W(:,excludeColumns(i)+1:end)];
    excludeColumns(i)=excludeColumns(i)-9;
    if i==length(excludeColumns)
        includeColums=[excludeColumns(i)+1:24];
    else
        includeColums=[excludeColumns(i)+1:excludeColumns(i+1)-1 includeColums];
    end
end

estPara=-inv(W'*W)*W'*Y;
realDh(includeColums')=realDh(includeColums')+estPara(10:end);
realDhComp=[realDh(1,:)'*180/pi realDh(2:3,:)'*1000 realDh(4,:)'*180/pi]

dhYzy=[
    0         0  180.0000   98.5000;
    -89.9981   -0.7140  -89.9740  121.6187;
    179.9249  409.0894    0.0381         0;
    179.9831  376.6947  -90.0030         0;
    -89.9790    0.2379    0.0018  102.7185;
    89.9965    0.2233         0   94.0000];
dhYzy=[dhYzy(:,1:2) dhYzy(:,4) dhYzy(:,3)]

dhLeica=[
    -0.0319 0.5461 97.6632 180.0411;
    -90.0137 1.2928 121.2651 -90.2571;
    179.9181 409.1147 -0.3706 0.0365;
    180.0277 376.6818 -0.2375 -90.1281;
    -89.9502 -0.2694 102.0559 0.8172;
    89.6058 -1.6541 93.8492 -0.1112]
