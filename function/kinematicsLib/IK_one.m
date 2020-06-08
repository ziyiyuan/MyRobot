function q_sol = IK_one(q_sols, q_ref)
% return one solution 
m = size(q_sols,1);			%
ARM_DOF =  size(q_sols,2);
errSum = zeros(m,1);
for i = 1:1:m
    for j = 1:1:ARM_DOF
        errSum(i) = errSum(i) + abs(q_sols(i, j) - q_ref(j));
    end
end

index = 1;
errMin = abs(errSum(1));
for i = 2:1:m
    if(errSum(i) < errMin)
        index = i;
        errMin = errSum(i);
    end
end

q_sol(1) = q_sols(index, 1);
q_sol(2) = q_sols(index, 2);
q_sol(3) = q_sols(index, 3);
q_sol(4) = q_sols(index, 4);
q_sol(5) = q_sols(index, 5);
q_sol(6) = q_sols(index, 6);

for i = 1:1:ARM_DOF
    while (q_sol(i) > pi)
        q_sol(i) = q_sol(i) - 2 * pi;
    end
    while (q_sol(i) <= -pi)
        q_sol(i) = q_sol(i) + 2 * pi;
    end
end
q_sol = q_sol';
end