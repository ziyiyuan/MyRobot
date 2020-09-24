% q1 = [cos(pi/2),sin(pi/2),0,0]';
% w = [1,2,3]';
% 
% qd = QuatDot(q1,w)

function qd =  QuatDot(q1,w)
w1 = [0;w];
qd = 1/2 * quatMult(w1,q1)
end