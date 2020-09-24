function D_hat = get_Dhat(para_i)
% [XX(i),XY(i),XZ(i),YY(i),YZ(i),ZZ(i),MX(i),MY(i),MZ(i),M(i)]'];
Ixx = para_i(1);
Ixy = para_i(2);
Ixz = para_i(3);
Iyy = para_i(4);
Iyz = para_i(5);
Izz = para_i(6);
mx = para_i(7);
my = para_i(8);
mz = para_i(9);
m = para_i(10);
I = [Ixx, Ixy, Ixz; Ixy, Iyy, Iyz; Ixz, Iyz, Izz ];
Sl = skew([mx,my,mz]);
D_hat = [I,Sl'; Sl, m*eye(3)];
end