function [Se, Je, Fe]=FuncGetGNParaPositionMeas(dhArr,joint,xyz,rpy,pt,meas)
%-----------------------------------------------------
% de/Pm = Pm' = eye(3);
% de/Rm = Rw0'*(R06*Pt+P06) = Rw0' * P0t;
% de/Pt = Rw0*R06*Pt' = Rw0*R06*eye(3);
% J = [df/Pm,df/Rm,df/Pt];
%-----------------------------------------------------
T=eye(4);
for i=1:6
    Tii = transfer(dhArr(i,1),dhArr(i,2),dhArr(i,3),dhArr(i,4) + joint(i));
    T=T*Tii; %T16
end

rz = RotZ(rpy(1));
rzry = rz * RotY(rpy(2));
Rw0 = rzry * RotX(rpy(3));
P0t = T(1:3,1:3)*pt + T(1:3,4);
Pwt = Rw0 * P0t + xyz;

J_fra = eye(3);
J_rpy = [cross(J_fra(:,3),Rw0 * P0t),cross(rz(:,2),Rw0 * P0t),cross(rzry(:,1),Rw0 * P0t)];% rz ry rx// R01 * P1t in world
J_txyz = Rw0 * T(1:3,1:3);

Je = [J_fra,J_rpy,J_txyz];
Fe = Pwt - meas;
Se=Fe'*Fe;