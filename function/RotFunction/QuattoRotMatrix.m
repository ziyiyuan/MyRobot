function R = QuattoRotMatrix(quat)
normq = norm(quat);
a = quat(1)/normq;
b = quat(2)/normq;
c = quat(3)/normq;
d = quat(4)/normq;
R = [1-2*c^2-2*d^2, 2*b*c-2*a*d, 2*a*c+2*b*d
    2*b*c+2*a*d, 1-2*b^2-2*d^2, 2*c*d-2*a*b
    2*b*d-2*a*c, 2*a*b+2*c*d, 1-2*b^2-2*c^2];
end