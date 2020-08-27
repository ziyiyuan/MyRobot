function D_ext = getDext(para_all,n,eps)
para_dy = [];
n_para = size(para_all,1)/6;
for j = 1:1:n
    % order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok ]
    para_L_i = para_all(n_para*(j-1) + 1 : n_para*(j-1) + 10); % Link para
    para_dy = [para_dy;para_L_i];
end
D = getD(para_dy,n,eps);
D_ext = D;
end

% function D_ext = getDext(para_all,n,eps,N)
% para_dy = [];
% para_If = [];
% nf = N.f;
% nI = N.i;
% nL = N.l;
% n_para = nf + nI + nL;
% 
% for j = 1:1:n
%     % order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok ]
%     para_dy_i = para_all(14*(j-1) + 1 : 14*(j-1) + 10);
%     para_If_i = para_all(14*(j-1) + 11 : 14*(j-1) + 14);
%     para_ext = [para_If_i(2);para_If_i(3);para_If_i(1)]; % fv, fc; Ia
%     para_dy = [para_dy;para_dy_i];
%     para_If = [para_If;para_ext];
% end
% D = getD(para_dy,n,eps);
% D_add = diag(para_If);
% D_ext = blkdiag(D,D_add);
% end