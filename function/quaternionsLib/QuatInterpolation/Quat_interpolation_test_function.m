clc;clear all;close all

v_start = [1;1;0];
v_end = [0;0;2];

v_start = v_start/norm(v_start)
v_end = v_end/norm(v_end)

q_start = [0;v_start];
q_end = [0;v_end];

figure(1)
%% plot 单位球
[x,y,z] = sphere(30);%30是画出来的球面的经纬分面数...30的话就是30个经度, 30个纬度
x=0 + 1*x;           % 圆心:(4,2,0)   半径:1
y=0 + 1*y;
z=0 + 1*z;
surf(x,y,z)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
alpha(0.6)         %设置透明度
shading flat       %去掉那些线
view([0.5,-0.5,0.5]);
%%

for t = 0:0.01:1
%     qt = Quatlerp(q_start,q_end,t);
    qt = QuatNlerp(q_start,q_end,t)
    % qt = QuatSlerp(q_start,q_end,t);
    [axis1,angle1] = QuattoAxisAngle(qt)
    vt = qt(2:4,1);

    figure(1)
    hold on
    plot3([0,v_start(1)],[0,v_start(2)],[0,v_start(3)],'r')
    plot3([0,v_end(1)],[0,v_end(2)],[0,v_end(3)],'r')
    plot3([0,vt(1)],[0,vt(2)],[0,vt(3)],'b')
    scatter3(vt(1),vt(2),vt(3),'filled','b')
    hold off

    pause(0.05)
end
