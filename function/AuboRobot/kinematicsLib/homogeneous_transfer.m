function Tadd = homogeneous_transfer(alpha, a, d, theta)
% homogeneous_transfer 从mordified DH 参数得到齐次变换矩阵; 只适用于 modified DH
% 输入参数：
%   alpha: rotx angle
%   a: trans in x axis;
%   d：trans in z axis;
%   theta：rotz angle
% 输出参数：
%   Tadd：齐次变换矩阵
% 调用说明：
%   Tadd = homogeneous_transfer(alpha, a, d, theta) :输出相对于上一个坐标系的齐次变换矩阵；

% 版本号V1.0，编写于2020/8/27，修改于2020/8/27，作者：ziyi

ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
Tadd = [ct,   -st,   0,    a;
    st*ca, ct*ca, -sa,  -sa*d;
    st*sa, ct*sa, ca,   ca*d;
    0,  0,  0,  1];
end
