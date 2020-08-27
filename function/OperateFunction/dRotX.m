function dR = dRotx(t, deg)
% *** used in 运动学参数辨识 ***
% 输入弧度；得到旋转矩阵 Rx 的导数；

if nargin > 1 && strcmp(deg, 'deg')
    t = t *pi/180;
end

ct = cos(t);
st = sin(t);
dR = [
    0   0    0
    0   -st  -ct
    0   ct   -st
    ];
end
