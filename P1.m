% 温升对末端位姿的影响
clc;clear all; close all; format short
%% initiall
global Robot
robotType = 'I5';
ParaCAD(robotType);
%%
% q_in = [1,1,1,1,1,1];
limit = 175*pi/180;
dt = 40;
dmt = 23 * 10^(-6);
dm = dt * dmt;
DH_t = Robot.DH;
DH_t(:,[2,3]) = DH_t(:,[2,3]) .* (1 + dm);

j = 0;
for i = 1:1:100000
    q_in = -limit + 2* limit * rand(6,1);
    q_all(:,i) = q_in;
    
    Tt = FK(q_in,Robot.DH);
    T2 = FK(q_in,DH_t);
    
    distance(i) = norm(T2(1:3,4) - Tt(1:3,4))*1000;
    
    q_sols = IK_all(Tt, DH_t);
    if size(q_sols,1) == 0
        continue;
    else
        j = j+1;
    end
    q_sol(:,j) = IK_one(q_sols, q_in);
    q_res(:,j) = q_sol(:,j) - q_in;
    
    if norm(q_res(:,j)) > 1
        flag = 1;
    end
    
    dq = q_res(:,j);
    dq([1,5,6]) = 0;
    q3 = dq + q_in;
    T3 = FK(q3,DH_t);
    e1(i) = norm(T3(1:3,4) - Tt(1:3,4))*1000;
    if e1(i) >  1e-8
        flag1 = 1;
    end
end
max(distance)
max(e1)


