
% Á¦´«µÝ¾ØÕó
function RR = TransferForce(T)
RR = [T(1:3,1:3) zeros(3,3); skew(T(1:3,4))*T(1:3,1:3) T(1:3,1:3)];
end