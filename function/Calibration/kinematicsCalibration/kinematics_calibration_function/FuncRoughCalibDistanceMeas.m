function [xyz,pt]=FuncRoughCalibDistanceMeas(nom)
%-----------------------------------------------------
% rough estimation of 6 envirenment parameters
%-----------------------------------------------------
% syms x y z tx ty tz real;
%-----------------------------------------------------
% Guass-Netow Method
% s=e'*e, e=||f(x)||^2-Dmeas^2=||R16*Pt+P16-P||^2-Dmeas^2 
% =Pt'Pt+P16'P16+P'P+2*P16'*R16*Pt-2*P'*(R16*Pt+P16)-Dmeas^2 
% => de/dPt=2*Pt+2*R16'(P16-P)
%    de/dP=2*P-2*(R16*Pt+P16)
% or e=||P7-P||^2-Dmeas^2 =P7'P7+P'P-2*P7'P-Dmeas^2
% => de/dPt=2*R16'(P7-P)
%    de/dP=2*(P-P7)
% => H(s)=2*J'*J.
% line search --- wolfe 
% parameter: c1(=0.1)<0.5, c1<c2=0.7(in [0.6,0.8])<1.
% phi(lamda)=s(xk+lamda*dk)
%-----------------------------------------------------
c1=0.1;
c2=0.7;
lamda=1.0;
xyz=[0 0 0]';
pt=[0 0 0]';
groupSz=6;
wolfeSearch=1;
iters=0;
data_idx=0;
while iters<200
    iters=iters+1;    
for org_data_idx=1:nom.size
    data_idx=data_idx+1;
    if data_idx > nom.size
        data_idx=data_idx-nom.size;
    end
    if mod(data_idx,groupSz)==1
        Je = [];
        Fe = [];
        Se = 0;
        gSe = zeros(9,9);
    end
    [Se_sec, Je_sec, Fe_sec]=FuncGetGNParaDistanceMeas(nom.dhArr,nom.joints(data_idx,:),xyz,pt,nom.measuredDis(data_idx));
    
    Se=Se+Se_sec; % phi(0)
    Je=[Je;Je_sec];
    Fe=[Fe;Fe_sec];
    if mod(data_idx,groupSz)==0
        hS=2*Je'*Je;
        gS=2*Je'*Fe; % phi'(0)
        if 0
            dk=-gS/2
        else
            dk=-inv(hS)*gS;
        end
        
        %----------------------------------
        % line search---wolfe
        % x=x+a*d,
        % f(x+)<=f(x)+c1*a*Gf(x)'*d.
        % Gf(x+)'*d>=c2*Gf(x)'*d.
        %----------------------------------
%         rho = 0.25; sigma = 0.75;
        rho = 0.1; sigma = 0.7;
        lamda = rand(); 
        a = 0; b = Inf;
        xk=[xyz' pt']';

        loops=0;
        while (1)
            loops=loops+1;
            Je = [];
            Fe = [];
            Se_plus=0;
            for ii=data_idx-groupSz+1:data_idx
                [Se_sec, Je_sec, Fe_sec]=FuncGetGNParaDistanceMeas(nom.dhArr,nom.joints(ii,:),xyz+lamda*dk(1:3),pt+lamda*dk(4:6),nom.measuredDis(ii));
                Se_plus=Se_plus+Se_sec; % phi(lamda)
                Je=[Je;Je_sec];
                Fe=[Fe;Fe_sec];
            end
            gSplus=2*Je'*Fe; % phi'(lamda)
%             Se_plus
%             Se+rho*lamda*gS'*dk
%             gSplus'*dk
%             sigma*gS'*dk
            cc = Se+rho*lamda*gS'*dk;
            if ~(Se_plus<=Se+rho*lamda*gS'*dk)
                b = lamda;
                lamda = (lamda+a)/2;
%                 disp('wolf1')
                continue;
            end
            if ~(gSplus'*dk >= sigma*gS'*dk)
                a = lamda;
                lamda = min([2*lamda, (b+lamda)/2]);
%                 disp('wolf2')
                continue;
            end
            break;
        end
%         lamda
        dk=lamda*dk;
        
        xyz=xyz+dk(1:3);
        pt=pt+dk(4:6);
    end
end
est=[xyz' pt'];
end
end