%% 注意 欧拉角全部采用XYZ 固定角； 即 R = rz * ry * rx;
clc
clear all
j = 0;
N = 1;
while j < N
    j = j+1;
rpy = -pi + rand(1,3)*2*pi;
R0 = EulerAngletoRotMatrixZYX(rpy(1),rpy(2),rpy(3))
e = [[]];
%%  RPY && matrix
[qx,qy,qz] = RotMatrixtoEulerAngleZYX(R0);
R1 = EulerAngletoRotMatrixZYX(qx,qy,qz)
% e{1}(1) = qx - rpy(1);
% e{1}(2) = qy - rpy(2);
% e{1}(3) = qz - rpy(3);
e{1}(4) = norm(R1 - R0);

%%  QUATIONN && Matrix
q1 = RotMatrixtoQuat(R0);
R2 = QuattoRotMatrix(q1);
e{2} = norm(R2 - R0);

%% angle _axis && quationn
[axis,angle] = QuattoAxisAngle(q1);
q2 = AxisAngletoQuat(axis,angle);
e{3} = norm(q1 - q2);

for i = 1:1:length(e)
    if norm(e{i}) >  1e-8
        error(['cal error' num2str(i)])
        break
    end
end

end

