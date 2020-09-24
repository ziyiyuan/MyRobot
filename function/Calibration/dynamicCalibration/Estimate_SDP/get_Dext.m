function D_ext = get_Dext(para_all,dof,eps,identifyPara,addConstraints)

n_para = 0;
if identifyPara.linkModel
    n_para = n_para + 10; % link inertia number
end
if identifyPara.roterInertiaModel
    n_para = n_para + 1;
end
if identifyPara.frictionModel
    n_para = n_para + 2;
end
if identifyPara.offsetModel
    n_para = n_para + 1;
end

para_L = [];
para_add = [];
for j = 1:1:dof
    %input order [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,Ia, fv,fc,fok
    %]£»
    %     change order in
    % [[Ixx, Ixy, Ixz, Iyy, Iyz, Izz, Mx,My,Mz,M,fv,fc,Ia]]
    
    if identifyPara.linkModel
        para_L_i = para_all(n_para*(j-1) + 1 : n_para*(j-1) + 10); % Link para
        para_L = [para_L;para_L_i];
    end
    if identifyPara.frictionModel
        para_F_i = para_all(n_para*(j-1) + 12 : n_para*(j-1) + 13);
        para_add = [para_add; para_F_i];
    end
    if identifyPara.roterInertiaModel
        para_R_i = para_all(n_para*(j-1) + 11);
        para_add = [para_add; para_R_i];
    end
    if addConstraints.M.cons
        para_m_l = para_all(n_para * (j-1) + 10) - addConstraints.M.reff(j) + addConstraints.M.offset;
        para_m_u = -para_all(n_para * (j-1) + 10) + addConstraints.M.reff(j) + addConstraints.M.offset;
        para_add = [para_add; para_m_l;para_m_u];
    end  
end
D = get_D(para_L,dof,eps);
D_add = diag(para_add);
D_ext = blkdiag(D,D_add);
end
