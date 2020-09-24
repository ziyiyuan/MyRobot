function [xyz,rpy,pt]=FuncRoughCalibPositionMeas(nom)
%-----------------------------------------------------
% rough estimation of 6 envirenment parameters ,高斯牛顿搜索 得到工具参数以及世界坐标系参数；
%-----------------------------------------------------
% syms x y z tx ty tz real;
%-----------------------------------------------------
% Guass-Netow Method
% s=e'*e, e = f(x) - Pm = (Rw0*(R06*Pt+P06) + Pw0) - Pm
% ds/dx = 2*e*(de/dx) = 2*e*(df/dx);
% de/Pm = Pm' = eye(3);
% de/Rm = Rw0'*(R06*Pt+P06) = Rw0' * P0t;
% de/Pt = Rw0*R06*Pt' = Rw0*R16*eye(3);
% de/dhpara = Rw0*P0t';
% J = [df/Pm,df/Rm,df/Pt,df/dhpara];
% gs = 2*J'* e
% Hs  = 2*J'*J;
% line search --- wolfe
% parameter: c1(=0.1)<0.5, c1<c2=0.7(in [0.6,0.8])<1.
% phi(lamda)=s(xk+lamda*dk)
%-----------------------------------------------------
dim = 3; % 只考虑位置
groupSz = 3;

xyz = [0 0 0]'; %初始化坐标系参数；
rpy = [0,0,0]';
pt = [0 0 0]';

% xyz = nom.baseInLeica.pos;
% rpy = nom.baseInLeica.rpy;
% pt = nom.toolPosInFlange.pos;



iters = 0;
data_idx = 0;
while iters < 200
    iters = iters+1;
    for org_data_idx = 1:nom.joint_num
        data_idx = data_idx + 1;
        if data_idx > nom.joint_num
            data_idx = data_idx - nom.joint_num;
        end
        if mod(data_idx, groupSz)==1
            Je = [];
            Fe = [];
            Se = 0;
        end
        [Se_sec, Je_sec, Fe_sec]=FuncGetGNParaPositionMeas(nom.DH, nom.joints(data_idx,:),xyz,rpy,pt,nom.mesuredData(data_idx,1:dim)');
        
        Se=Se+Se_sec; % phi(0)
        Je=[Je;Je_sec];
        Fe=[Fe;Fe_sec];
        if mod(data_idx,groupSz)==0
            hS=2*(Je'*Je);
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
%             lamda = 0.3;
            a = 0; b = Inf;
            xk =[xyz' rpy' pt']';
            loops=0;
            while (1)
                loops = loops+1;
                Je = [];
                Fe = [];
                Se_plus=0;
                for ii = data_idx-groupSz+1 : data_idx
                    [Se_sec, Je_sec, Fe_sec] = FuncGetGNParaPositionMeas(nom.DH,nom.joints(ii,:),xyz+lamda*dk(1:3),rpy + lamda*dk(4:6),pt+lamda*dk(7:9),nom.mesuredData(ii,1:dim)');
                    Se_plus = Se_plus + Se_sec; % phi(lamda)
                    Je=[Je; Je_sec];
                    Fe=[Fe; Fe_sec];
                end
                gSplus = 2*Je'*Fe; % phi'(lamda)
                %             Se_plus
                %             Se+rho*lamda*gS'*dk
                %             gSplus'*dk
                %             sigma*gS'*dk
                
                if ~(Se_plus <= Se+rho*lamda*gS'*dk)
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
            dk = lamda*dk;
            
            xyz = xyz+dk(1:3);
            rpy = rpy + dk(4:6);
            pt = pt+dk(7:9);
        end
    end
    est = [xyz' rpy' pt'];
end
for k = 1:1:3
    while((rpy(k) > pi) || (rpy(k) < -pi))
        if((rpy(k) > pi))
            rpy(k) = rpy(k) - 2* pi;
        elseif((rpy(k) < -pi))
            rpy(k) = rpy(k) + 2* pi;
        else
            break
        end
    end
end
end