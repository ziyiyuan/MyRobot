function [est_xyz,est_pt,realDh]=FuncDhCalibDistanceMeas(nom, dh_para_mask)

identifiable_mask=dh_para_mask;

para_num=36;
env_para_num=6;
frame_pos_idx=1;
tool_pos_idx=4;
%-----------------------------------------------------
% D^2=||F(x)=f(x)-P(x)=Px17-Px=F(0)+J(F)dx||^2
% =F'F+2F'Jdx+dx'J'Jdx, i.e. Adx=B, A=2F'J, B=D^2-F'F.
% df/dx=[de1/dx1 de1/dx2 ...;de2/dx1 ...;de3/dx1 ...]
% d(xf)=[dx dy dz]'
% d(pt)=[dx dy dz]'
% d(dh)=[da dA dD dJ db]'
% dx=[d(xf)' d(pt)' d(dh1)' d(dh2)' ... d(dh6)' ]'
% total 3 + 3 + 4*6 + 6 = 33 + 6=36.
%-----------------------------------------------------
nom.residualPos=zeros(nom.size,1);
for data_idx=1:nom.size
    % nominal p(i), rot(i).
    T=eye(4);
    for i=1:6
        Tii=connectingRodTransfer(nom.dhArr(:,i),nom.joints(data_idx,i));
        T=T*Tii;
        P{i}=T(1:3,4);
        R{i}=T(1:3,1:3);        
    end
    P{7}=R{6}*nom.toolPosInFlange+P{6};    

    JFr=-eye(3);
    JPt=R{6};
    JF=[JFr JPt];
    
    if data_idx==1
        valid_dh_idx=[];
    end
    
    for i=1:6
        if i==1
            JAlpha{i}=zeros(3,1);
            JBeta{i}=zeros(3,1);
            JA{i}=zeros(3,1);
        else
            JAlpha{i}=cross(R{i-1}(:,1),P{7}-P{i-1});
            JBeta{i}=cross(R{i-1}(:,2),P{7}-P{i-1});
            JA{i}=R{i-1}(:,1);
        end
        JJ{i}=cross(R{i}(:,3),P{7}-P{i});
        JD{i}=R{i}(:,3);
                
        if dh_para_mask(1,i) ~= 0
            JF=[JF JAlpha{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-4];
            end
        end
        if dh_para_mask(2,i) ~= 0
            JF=[JF JA{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-3];
            end
        end
        if dh_para_mask(3,i) ~= 0
            JF=[JF JD{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-2];
            end
        end
        if dh_para_mask(4,i) ~= 0
            JF=[JF JJ{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i-1];
            end
        end
        if dh_para_mask(5,i) ~= 0
            JF=[JF JBeta{i}];
            if data_idx==1
                valid_dh_idx=[valid_dh_idx 5*i];
            end
        end
    end
    
    F0=P{7}-nom.dynalogInBase.pos;
 
    nom.Phi{data_idx}=2*F0'*JF;
    
    if data_idx == 1
        W=nom.Phi{data_idx};
        Y=F0'*F0-nom.measuredDis(data_idx)^2;
    else
        W=[W;nom.Phi{data_idx}];
        Y=[Y;F0'*F0-nom.measuredDis(data_idx)^2];
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
excludeColumns=find(Rwii<qr_rii_thr & Rwii>-qr_rii_thr)';
for i=length(excludeColumns):-1:1
    W=[W(:,1:excludeColumns(i)-1) W(:,excludeColumns(i)+1:end)];
    if excludeColumns(i) > env_para_num
        identifiable_mask(valid_dh_idx(excludeColumns(i)-env_para_num))=0;
    end
end
identifiable_mask

% para estimation.Y+WX=Y-W(W'W)W'Y
estPara=-inv(W'*W)*W'*Y;

% update valid dh para.
realDh=[nom.dhArr([2:4 1],:); zeros(1,6)];% full dh para
valid_dh_para_num=0;
for i=1:30
    if identifiable_mask(i) ~= 0
        valid_dh_para_num = valid_dh_para_num+1;
        realDh(i)=realDh(i)+estPara(env_para_num+valid_dh_para_num);
    end
end
% vpa(estPara)
% 
realDhComp=[realDh(1,:)'*180/pi realDh(2:3,:)'*1000 realDh([4 5],:)'*180/pi]
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

ptLeica=[-2.9016 11.8619 26.6352]'/1000;
%-----------------------------------------------------
% average err & RMS
%-----------------------------------------------------
avgErr=0;
maxErr=0;
rmsErr=0;

sim.dhArr=nom.dhArr;
sim.toolPosInFlange=nom.toolPosInFlange;

sim.dhArr=[realDhComp(:,[4 1])*pi/180 realDhComp(:,[2 3])/1000]';
sim.toolPosInFlange=nom.toolPosInFlange+estPara(4:6);
sim.dynalogInBase.pos=nom.dynalogInBase.pos+estPara(1:3);

for data_idx=1:nom.size
    T=eye(4);
    for i=1:6
        Tii=[rotY(realDh(5*i)) zeros(3,1);0 0 0 1]*connectingRodTransfer(sim.dhArr(:,i),nom.joints(data_idx,i));
        T=T*Tii;
    end                                                      
    ptInBase=T(1:3,1:3)*sim.toolPosInFlange+T(1:3,4);
   
    currErr=abs(norm(ptInBase-sim.dynalogInBase.pos)-nom.measuredDis(data_idx));
    EE(data_idx) = currErr;
    if maxErr < currErr
        maxErr = currErr;
    end
    avgErr=avgErr+currErr;
%     rmsErr=rmsErr+currErr^2;
end
avgErr=avgErr/nom.size
maxErr
criter.avgErr=sum(EE)/(nom.size * 1)*1000;
criter.maxErr = max(EE)*1000;
criter.rmsErr = sqrt(EE*EE'/(nom.size *1))*1000;
criter
% rmsErr=(rmsErr./[nom.size-1 size(W,1)-size(W,2) size(Rw,1)-size(Rw,2)]).^0.5
% rmsErr1=norm(Y+W*estPara)./[nom.size-1 size(W,1)-size(W,2) size(Rw,1)-size(Rw,2)].^0.5
est_xyz=nom.dynalogInBase.pos*1000;
est_pt=sim.toolPosInFlange*1000;