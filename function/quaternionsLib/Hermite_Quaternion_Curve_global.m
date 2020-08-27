% simulation of  Hermite Quaternion Curve
% reference : A General Construction Scheme for Unit Quaternion Curves
% with Simple High Order Derivatives1
%
% 可以实现旋转轴的均匀变化，但是在离两端姿态的连接处出现不连续；未找到原因；
% 与论文的不同之处在于姿态是在全局坐标下表示的；

clc
clear all
close all
vx = [1;0;0];
q0 = [1;0;0;0];R0 = eye(3);
% 先绕 z 轴旋转 pi/2;
% 再绕 y 轴旋转 -pi/2;
axis1 = [0;1;0]; angle1 = pi/2;
axis2 = [1;0;0]; angle2 = pi/2;
Rd1 = AxisAngletoRotMatrix(axis1,angle1);
Rd2 = AxisAngletoRotMatrix(axis2,angle2);

R1 = Rd1
R2 = Rd2 * Rd1

q1 =  RotMatrixtoQuat(R1);
q2 =  RotMatrixtoQuat(R2);
w0 = [0;0;0];
% w0 = [0.5;0.5;0.5];
w1 = [0.5;0.5;0.5];
% w2 = [0.5;0.5;0.5];
w2 = [0;0;0];

Q = [q0,q1,q2]; %  q 位置
W = [w0,w1,w2]; % 速度
%% interpolation with 3d bezier 

qq = [[]];
dt = 0.01;
i = 0;
for j = 1:2
    qa = Q(:,j);
    qb = Q(:,j+1);
    wa = W(:,j);
    wb = W(:,j+1);
    
    q0 = qa;
    w1 = wa/3;
    w3 = wb/3;
    q_w2 = QuatLeftMultiplyQuat(QuatConjugate(QuatExp(wb/3))) ...  %% global
        * QuatLeftMultiplyQuat(qb)...
        * QuatLeftMultiplyQuat(QuatConjugate(qa)) ...
        * QuatConjugate(QuatExp(wa/3));
    w2 = QuatLog(q_w2);
    for t = 0:dt:1-dt
        i = i+1;
        b1 = 1 - (1 - t)^3;
        b2 = 3*t^2 - 2* t^3;
        b3 = t^3;
        qq(:,i) = QuatLeftMultiplyQuat(QuatExp(w3*b3))  ...
            * QuatLeftMultiplyQuat(QuatExp(w2*b2))...
            *  QuatLeftMultiplyQuat(QuatExp(w1*b1))...
            * q0;
    end
end

R = [[]];
for i = 1:size(qq,2)
    R{i} = QuattoRotMatrix(qq(:,i)) ;
    v_plot(:,i) = R{i} * vx;
end

Axis = [[]];
angle = [[]];

for i = 1:size(qq,2)-1
    dq = QuatLeftMultiplyQuat(qq(:,i+1)) * QuatConjugate(qq(:,i));%
    [Axis(:,i),angle(i)] = QuattoAxisAngle(dq) ;
end


%% plot
if 1
    figure(1)
    [x,y,z] = sphere(30);%30设置球面的经纬分面数
    x=0 + 1*x;           % 圆心:(0,0,0)   半径:1
    y=0+ 1*y;
    z=0 + 1*z;
    surf(x,y,z)
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal
    alpha(0.6)         %设置透明度
    shading flat       %去掉那些线
    view([0.5,-0.5,0.5]);
    
    for i = 1:size(qq,2)
        %%plot 单位球
        figure(1)
        hold on
        plot3([0,v_plot(1,1)],[0,v_plot(2,1)],[0,v_plot(3,1)],'r')
        plot3([0,v_plot(1,end)],[0,v_plot(2,end)],[0,v_plot(3,end)],'r')
        plot3([0,v_plot(1,i)],[0,v_plot(2,i)],[0,v_plot(3,i)],'b')
        scatter3(v_plot(1,i),v_plot(2,i),v_plot(3,i),'filled','b')
        hold off
        pause(0.08)
    end
end

figure(2)
plot(Axis(1,:))
hold on
plot(Axis(2,:))
hold on
plot(Axis(3,:))
title('rotation_axis')

figure(3)
plot(angle)
title('angle')

figure(4) 
plot3(Axis(1,:),Axis(2,:),Axis(3,:))






