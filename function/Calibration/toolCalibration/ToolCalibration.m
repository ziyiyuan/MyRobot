function [T,error] = ToolCalibration(q)
% 输入 6*n 的关节角向量；
% 输出 工具标定点相对于法兰的齐次变换矩阵；
% 四点法得到工具末端位置；
% 指定 x 方向，以及 xoy 平面上的点，得到姿态
m = size(q,2);
if ( m < 6)
    error('need more joint angle ');
end
q_pos = q(:,1:(m - 2));
q_ori = [q(:,1),q(:,m-1:m)];
[p,error] = ToolCalibrationPos(q_pos);
R = ToolCalibrationOri(q_ori,p);
T = RpToTrans(R,p);
end

function [p,error] = ToolCalibrationPos(q)
m = size(q,2)
if (m < 4)
    error('need more joint angle >= 4 ');
end
A = [];
B = [];
for i = 1 : m-1
    [r1,p1] =TransToRp(FK(q(:,i)));
    [r2,p2] =TransToRp(FK(q(:,i+1)));
    A = [A;r1 - r2];
    B = [B;p2-p1];
end
p = inv(A'*A)*A' * B;
error = norm(A * p - B);
end

function R = ToolCalibrationOri(q, pt)
m = size(q,2)
if (m < 3)
    error('need more joint angle >= 3 ');
end
[r1,p1] =TransToRp(FK(q(:,1)));
[r2,p2] =TransToRp(FK(q(:,2)));
[r3,p3] =TransToRp(FK(q(:,3)));
v12 = r2 * pt + p2 - (r1 * pt + p1);% x in base ,
vf = r1' * v12; % base to flange
x = vf/norm(vf); % x in flange
v13 = r3 * pt + p3 - (r1 * pt + p1); %
vf = r1' * v13;
y = vf/norm(vf);
z = cross(x,y);
y = cross(z,x);
R = [x,y,z];
end