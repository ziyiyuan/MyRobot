%homogeneous transformation use in fk
%from i-1 to i
%modified DH parameters
function T = transfer(alpha, a, d, theta)
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
T = [ct,   -st,   0,    a;
    st*ca, ct*ca, -sa,  -sa*d;
    st*sa, ct*sa, ca,   ca*d;
    0,  0,  0,  1];
end
