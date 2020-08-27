%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% transfer arry
% param order: thelta/alpha/a/d
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TArr = connectingRodTransfer(dh_vector,joint)
ct = cos(dh_vector(1)+joint);
st = sin(dh_vector(1)+joint);
ca = cos(dh_vector(2));
sa = sin(dh_vector(2));
a = dh_vector(3);
d = dh_vector(4);
TArr = [    ct      -st     0       a;
            st*ca   ct*ca   -sa     -sa*d;
            st*sa   ct*sa   ca      ca*d;
            0       0       0       1   ];



