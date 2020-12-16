function qSol = IK_choose_one_sol_norm(q_sols, q_ref)
% IK_choose_one_sol_norm  ����norm ֵѡ����ӽ���һ���ؽڽǵĽ�
% ���������
%   q_sols: ��������ؽ����ƵĽ�����6*n��
%   q_ref: �ο��ؽڽ�.6*1��
% ���������
%   q_sol��ѡ���Ľ⣻
% ����˵����
%   q_sol = IK_choose_one_sol_norm(q_sols, q_ref)�� ����norm ֵѡ����ӽ���һ���ؽڽǵĽ⣻

% �汾��V1.0����д��2020/8/27���޸���2020/9/1�����ߣ�ziyi
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