function qSol = IK_choose_one_sol_norm(q_sols, q_ref)
% IK_choose_one_sol_norm  根据norm 值选出最接近上一个关节角的解
% 输入参数：
%   q_sols: 所有满足关节限制的解析解6*n；
%   q_ref: 参考关节角.6*1；
% 输出参数：
%   q_sol：选出的解；
% 调用说明：
%   q_sol = IK_choose_one_sol_norm(q_sols, q_ref)； 根据norm 值选出最接近上一个关节角的解；

% 版本号V1.0，编写于2020/8/27，修改于2020/9/1，作者：ziyi
if isempty(q_sols)
    qSol = [];
    return ;
else
    nSol = size(q_sols,2);			%
    index = 1;
    error = norm(q_sols(:,1) - q_ref);
    for i = 2:1:nSol
        err = norm(q_sols(:,i) - q_ref);
        if err < error
            index = i;
            error = err;
        end
    end
    qSol = q_sols(:,index);
end
end