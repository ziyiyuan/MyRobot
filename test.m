syms a1 b1 c1 a2 b2 c2 real
x1 = [a1,b1,c1]';
x2 = [a2,b2,c2]';
skew(x1)*skew(x2) - skew(x2)*skew(x1)
cross(x1,x2)
