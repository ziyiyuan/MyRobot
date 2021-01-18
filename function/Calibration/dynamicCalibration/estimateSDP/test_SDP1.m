%% test SDP out put form
clc
clear all
syms x1 x2
x = [x1,x2];
eqnX =  [x(1) - 1,           0,        0,        0,
    0, x(1) + x(2) - 2,        0,        0,
    0,           0, 5*x(2)- 3,     2*x(2),
    0,           0,     2*x(2), 6*x(2) - 4]
x0 = [0,0];
F0 = -subs(eqnX,x,x0)
F1 = subs(eqnX,x,[1,0]) + F0
F2 = subs(eqnX,x,[0,1]) + F0
block = [1,1,2];
y = [0,0];
record = [];
for i = 0:1:2
    y = [0,0];
    if i ~= 0
        y(i) = 1;
    end
    
    eqn =  [y(1) - 1,           0,        0,        0,
    0, y(1) + y(2) - 2,        0,        0,
    0,           0, 5*y(2)- 3,     2*y(2),
    0,           0,     2*y(2), 6*y(2) - 4];
    if i == 0
        eqn0 = -eqn;
        eqn = eqn0;
    else
        eqn = eqn + eqn0;
    end
    eqn
        se = 0;
        for m = 1:1:size(block,2)
            se = se + block(m)
            si = se - block(m) + 1
            for j = si :1:se
                for  k = j :1:se
                    if(abs(eqn(j,k)) > 1e-8)
                        a = eqn(j,k)
                        record = [record;i,m,j-si+1,k - si+1,eqn(j,k)];
                    else
                        eqn(j,k) = 0;
                    end
                end
            end
        end
    end