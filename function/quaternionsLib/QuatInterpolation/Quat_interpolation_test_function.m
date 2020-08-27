clc;clear all;close all

v_start = [1;1;0];
v_end = [0;0;2];

v_start = v_start/norm(v_start)
v_end = v_end/norm(v_end)

q_start = [0;v_start];
q_end = [0;v_end];

figure(1)
%% plot ��λ��
[x,y,z] = sphere(30);%30�ǻ�����������ľ�γ������...30�Ļ�����30������, 30��γ��
x=0 + 1*x;           % Բ��:(4,2,0)   �뾶:1
y=0 + 1*y;
z=0 + 1*z;
surf(x,y,z)
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
alpha(0.6)         %����͸����
shading flat       %ȥ����Щ��
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
