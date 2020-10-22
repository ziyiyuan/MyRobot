function D = get_D(para,dof,eps) % n dof
% para =  [XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i)]'.....M(n)];
para_num = 10;
dim = 6;
for i = 1:dof
    a = para_num*(i-1)+1 : para_num*i;
    b = dim*(i-1)+1 : dim*i;
    D(b,b) = get_Dhat(para(a)) - eye(dim) * eps;
end
end