function [Se, Je, Fe]=FuncGetGNParaPositionMeas(dhArr,js,xyz,pt,meas)
%-----------------------------------------------------
% => de/dPt=2*R16'(P7-P)
%    de/dP=-2*(P7-P)
%-----------------------------------------------------
T=eye(4);
for i=1:6
    Tii=connectingRodTransfer(dhArr(:,i),js(i)); 
    T=T*Tii;
end
P=T(1:3,1:3)*pt+T(1:3,4);

Je=2*(P-xyz)'*[-eye(3) T(1:3,1:3)];
Fe=(P-xyz)'*(P-xyz)-meas'*meas;
Se=Fe'*Fe;
   