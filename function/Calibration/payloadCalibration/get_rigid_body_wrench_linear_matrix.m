% 计算单个刚体的力矩矩阵；
% get_rigid_body_wrench_linear_matrix 计算单个刚体的力矩矩阵
% 输入参数：
%   w:  刚体角速度（相对于力作用点）
%   wd: 刚体角加速度（相对于力作用点）
%   vd: 刚体加速度（相对于力作用点）
% 输出参数：
%   Wii：6*10 的矩阵
% 调用说明：
%   Wii = get_rigid_body_wrench_linear_matrix(w,wd,vd)

% 版本号V1.0，编写于2020/10/22，修改于2020/-/-，作者：ziyi

function Wii = get_rigid_body_wrench_linear_matrix(w,wd,vd)
% Fi = Wii * Para_i；para in order Ixx Ixy Ixz Iyy Iyz Izz mcx mcy mcz m
Wii = [zeros(3,6) skew(wd)+skew(w)*skew(w) vd;skewStar(wd)+skew(w)*skewStar(w) -skew(vd) zeros(3,1)]; %Wii
end