function D = getD(para,n,eps) % n dof

for i = 1:n
    a = 10*i-9 : 10*i;
    b = 6*i - 5 : 6*i;
    D(b,b) = getDhat(para(a)) - eye(6) * eps;
end
end