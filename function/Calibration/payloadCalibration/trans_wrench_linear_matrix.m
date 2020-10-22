% 力矩传递矩阵；
% get_rigid_body_wrench_linear_matrix 计算单个刚体的力矩矩阵
% 输入参数：
%   w:  刚体角速度（相对于力作用点）
%   wd: 刚体角加速度（相对于力作用点）
%   vd: 刚体加速度（相对于力作用点）
% 输出参数：
%   Wii：6*10 的矩阵
% 调用说明：
%   HA = trans_wrench_linear_matrix(T, Wii)

% 版本号V1.0，编写于2020/10/22，修改于2020/-/-，作者：ziyi

function HA = trans_wrench_linear_matrix(T, Wii)
RR = TransferForce(T); %TAi
HA = RR * Wii;
end