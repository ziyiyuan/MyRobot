% 温升对末端位姿的影响
clc;clear all; close all; format short
%% initiall
robotType = 'I5';
Robot = get_cad_model_para(robotType)
%%
% q_in = [1,1,1,1,1,1];
limit = 175*pi/180;
dt = 40;
dmt = 23 * 10^(-6);
dm = dt * dmt;
DH_t = Robot.DH;
DH_t(:,[2,3]) = DH_t(:,[2,3]) .* (1 + dm);

q_in = [-1.5345   -0.9678    0.6017         0   -0.2271    0.8333]';
T = forward_kinematics(Robot.DH, q_in);

q_sols = IK_all(DH_t,T);
q_sol = IK_one(q_sols, q_in);
T1 = forward_kinematics(DH_t,q_sol);
norm(T-T1)


j = 0;k = 0;
for i = 1:1:100000
    q_in = -limit + 2* limit * rand(6,1);
    q_all(:,i) = q_in;
    
    Tt = forward_kinematics(Robot.DH, q_in);
    Tr = forward_kinematics(DH_t,q_in);
    
    distance(i) = norm(Tr(1:3,4) - Tt(1:3,4))*1000;
    
    q_sols = IK_all(DH_t,Tt);
    if size(q_sols,1) == 0
        continue;
    else
        j = j+1;
        q_sol_r(:,j) = IK_one(q_sols, q_in);
        q_res(:,j) = q_sol_r(:,j) - q_in;
        
        T2 = forward_kinematics(DH_t, q_sol_r(:,j));
        e2(j) = norm(T2 - Tt)*1000;
        
        if e2(j) >  1e-8
            flag2 = 1;
        end
        
        if norm(q_res(:,j)) > 1/180*pi
            flag = 1;
            continue;
        else
            k = k+1;
        end
    end
    
    dq = q_res(:,j);
    dq([1,5,6]) = 0;
    q3 = dq + q_in;
    T3 = forward_kinematics(DH_t, q3);
    e1(k) = norm(T3(1:3,4) - Tt(1:3,4))*1000;
    
    
    if e1(k) >  1
        flag1 = 1;
    end
end
max(distance)
max(e1)
max(e2)


