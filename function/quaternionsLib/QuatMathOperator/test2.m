clc
clear all

syms wx wy wz q0 q1 q2 q3 wdx wdy wdz dt real
w = [wx;wy;wz];
wd = [wdx;wdy;wdz];
q = [q0;q1;q2;q3];

q1 = QuatLeftMultiplyQuat([0;w])

a1 = 1/2*(w*dt + 1/2 * wd * dt^2);
    
qa = QuatExp(a1)

qd = 1/2 * quatMult([0;w],q);
qdd = 1/2*(quatMult([0;wd],q) + quatMult([0;w],qd));
qa = q + qd*dt + 1/2 * qdd * dt*dt;
A1 = [diff(qa,q0),diff(qa,q1),diff(qa,q2),diff(qa,q3),...
    diff(qa,wx),diff(qa,wy),diff(qa,wz), ...
    diff(qa,wdx),diff(qa,wdy),diff(qa,wdz)];

% a1 = QuatExp(1/2*w*dt + 1/2*wd*dt^2);
% qb = quatMult([0;w],qd))

Q1 = QuatLeftMultiplyQuat([0;w])
diff(Q1,wx)



function M = diffQ(n)
if n == 1
    M = eye(4);
elseif n == 2
    M = [     0    -1     0     0
        1     0     0     0
        0     0     0    -1
        0     0     1     0];
    
elseif n == 3
    M = [     0     0    -1     0
        0     0     0     1
        1     0     0     0
        0    -1     0     0];
elseif n == 4
    M = [     0     0     0    -1
        0     0    -1     0
        0     1     0     0
        1     0     0     0];
end
end


function M = wskew(w)
M = QuatLeftMultiplyQuat([0;w])
end