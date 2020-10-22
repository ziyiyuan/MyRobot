clc
clear all
close all
N = 2000;
NUM = 1;
load('filter.mat')
qc = filter.qc(:,1:N);
V = var(qc(NUM,:)) % 计算当前数据的方差；
refdata = filter.motionTraj;% 输入的

dim = 1;
dt = 0.005;
I_d = eye(dim);

delta_a = 100;%加速度扰动

a2 = 10^(-5);
a3 = 10^(-5);
% a4 = 10^(-1);
delta_m = V;
delta_m = 0.5;

Q = diag([0.5*dt^2,dt,1])*delta_a;
% Q = diag([0.1,20,10000]);
R = delta_m * eye(dim);

A = [I_d,dt*I_d,dt^2*I_d/2;
    zeros(dim),I_d,dt*I_d;
    zeros(dim,2*dim),I_d];

C = [I_d,zeros(dim,2*dim)];

% x1 = [-0.7,1,0]';
x1 = [0,0,0]';
P1 = eye(dim*3);


if 0
    for i = 1:1:size(qc,2)
        x01 = x1(1);
        x02 = x1(2);
        x03 = x1(3);
        y1 = qc(NUM,i);
        p01_1 = P1(1,1);
        p01_2 = P1(1,2);
        p01_3 = P1(1,3);
        
        p02_1 = P1(2,1);
        p02_2 = P1(2,2);
        p02_3 = P1(2,3);
        
        p03_1 = P1(3,1);
        p03_2 = P1(3,2);
        p03_3 = P1(3,3);
        
        ss(1) = x01 + dt*x02 + (dt^2*x03)/2 - (((x03*dt^2)/2 + x02*dt + x01 - y1)*(p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(2) = x02 + dt*x03 - (((x03*dt^2)/2 + x02*dt + x01 - y1)*(p02_1 + (dt^2*(p02_3 + dt*p03_3))/2 + dt*p03_1 + dt*(p02_2 + dt*p03_2)))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(3) = x03 - (((p03_3*dt^2)/2 + p03_2*dt + p03_1)*((x03*dt^2)/2 + x02*dt + x01 - y1))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(4) = -((p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2)/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2) - 1)*(p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(5) = -((p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2)/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2) - 1)*(p01_2 + dt*p02_2 + dt*((p03_3*dt^2)/2 + p02_3*dt + p01_3) + (dt^2*p03_2)/2);
        ss(6) = -((p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2)/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2) - 1)*((p03_3*dt^2)/2 + p02_3*dt + p01_3);
        ss(7) = p02_1 + (dt^2*(p02_3 + dt*p03_3))/2 + dt*p03_1 + dt*(p02_2 + dt*p03_2) - ((p02_1 + (dt^2*(p02_3 + dt*p03_3))/2 + dt*p03_1 + dt*(p02_2 + dt*p03_2))*(p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(8) = p02_2 + delta_a*dt + dt*p03_2 + dt*(p02_3 + dt*p03_3) - ((p02_1 + (dt^2*(p02_3 + dt*p03_3))/2 + dt*p03_1 + dt*(p02_2 + dt*p03_2))*(p01_2 + dt*p02_2 + dt*((p03_3*dt^2)/2 + p02_3*dt + p01_3) + (dt^2*p03_2)/2))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(9) =  p02_3 + dt*p03_3 - (((p03_3*dt^2)/2 + p02_3*dt + p01_3)*(p02_1 + (dt^2*(p02_3 + dt*p03_3))/2 + dt*p03_1 + dt*(p02_2 + dt*p03_2)))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(10) = p03_1 + dt*p03_2 + (dt^2*p03_3)/2 - (((p03_3*dt^2)/2 + p03_2*dt + p03_1)*(p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(11) = p03_2 + dt*p03_3 - (((p03_3*dt^2)/2 + p03_2*dt + p03_1)*(p01_2 + dt*p02_2 + dt*((p03_3*dt^2)/2 + p02_3*dt + p01_3) + (dt^2*p03_2)/2))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        ss(12) = delta_a + p03_3 - (((p03_3*dt^2)/2 + p02_3*dt + p01_3)*((p03_3*dt^2)/2 + p03_2*dt + p03_1))/(delta_m + p01_1 + dt*p02_1 + dt*((p03_2*dt^2)/2 + p02_2*dt + p01_2) + (delta_a*dt^2)/2 + (dt^2*p03_1)/2 + (dt^2*((p03_3*dt^2)/2 + p02_3*dt + p01_3))/2);
        
        x1 = ss(1:3)';
        P1 = [ss(4),ss(5),ss(6);ss(7),ss(8),ss(9);ss(10),ss(11),ss(12)];
        res1(:,i) = x1;
    end
end

x1 = [-0.7,1,0]';
x1 = [0,0,0]';
P1 = eye(dim*3);
if 0
    for i = 1:1:size(qc,2)
        x01 = x1(1);
        x02 = x1(2);
        x03 = x1(3);
        y1 = qc(NUM,i);
        p011 = P1(1,1);
        p012 = P1(1,2);
        p013 = P1(1,3);
        
        p021 = P1(2,1);
        p022 = P1(2,2);
        p023 = P1(2,3);
        
        p031 = P1(3,1);
        p032 = P1(3,2);
        p033 = P1(3,3);
        
        tmp0 = dt*x02;
        tmp1 = 0.5*dt*dt;
        tmp2 = tmp1*x03;
        tmp3 = -tmp0 - tmp2 - x01 + y1;
        tmp4 = dt*p022 + p012 + p032*tmp1;
        tmp5 = p033*tmp1;
        tmp6 = dt*p023 + p013 + tmp5;
        tmp7 = delta_a*tmp1 + dt*p021 + dt*tmp4 + p011 + p031*tmp1 + tmp1*tmp6;
        tmp8 = 1/(delta_m + tmp7);
        tmp9 = tmp7*tmp8;
        tmp10 = dt*p031;
        tmp11 = dt*p032;
        tmp12 = dt*(p022 + tmp11);
        tmp13 = dt*p033;
        tmp14 = p023 + tmp13;
        tmp15 = tmp1*tmp14;
        tmp16 = p021 + tmp10 + tmp12 + tmp15;
        tmp17 = tmp3*tmp8;
        tmp18 = p031 + tmp11 + tmp5;
        tmp19 = 1.0 - tmp9;
        tmp20 = dt*tmp6 + tmp4;
        tmp21 = 1.0*delta_a;
        tmp22 = 1.0*tmp11;
        tmp23 = tmp16*tmp8;
        tmp24 = 1.0*tmp13;
        tmp25 = tmp18*tmp8;
        ss(1) = tmp0 + tmp2 + tmp3*tmp9 + x01;
        ss(2) = dt*x03 + tmp16*tmp17 + x02;
        ss(3) = tmp17*tmp18 + x03;
        ss(4) = tmp19*tmp7;
        ss(5) = tmp19*tmp20;
        ss(6) = tmp19*tmp6;
        ss(7) = 1.0*p021 + 1.0*tmp10 + 1.0*tmp12 + tmp15 - tmp16*tmp9;
        ss(8) = 1.0*dt*tmp14 + dt*tmp21 + 1.0*p022 - tmp20*tmp23 + tmp22;
        ss(9) = 1.0*p023 - tmp23*tmp6 + tmp24;
        ss(10) = 1.0*p031 - tmp18*tmp9 + tmp22 + tmp5;
        ss(11) = 1.0*p032 - tmp20*tmp25 + tmp24;
        ss(12) = 1.0*p033 + tmp21 - tmp25*tmp6;
        
        x1 = ss(1:3)';
        P1 = [ss(4),ss(5),ss(6);ss(7),ss(8),ss(9);ss(10),ss(11),ss(12)];
        res2(:,i) = x1;
    end
end

x1 = [qc(1,1),0,0]';
% x1 = [0,0,0]';
P1 = eye(dim*3);
% qc = [[0;0;0;0;0;0],qc];
robotType = 'I5';
toolPara = zeros(10,1);
Robot = get_cad_model_para(robotType);
if 1
    for i = 1:1:size(qc,2)
        x2 = A*x1;
        P2 = A*P1*A' + Q;
        K1 = P2*C'*inv(C*P2*C' + R);
        y1 = qc(NUM,i);
        x1 = x2 + K1*(y1 - C*x2);
        P1 = (eye(dim*3) - K1 * C)*P2;
        res(:,i) = x1;
    end
end

figure(1)
q = refdata.q';
qd = diff(q)/dt;
qdd = diff(q,2)/(dt^2);
plot(q(:,NUM))
hold on
plot(qd(:,NUM))
hold on
plot(qdd(:,NUM))
% legend('q','qd','qdd')
% title('reff data')
hold on

% figure(2)
plot(res')
hold on
plot(qc(NUM,:))
legend('q','qd','qdd','q1','q1d','q1dd','qc')

for i = 2:1:2000
    res(3,i) = res(3,i-1)*0.6 +  res(3,i)*0.4;
end
hold on
plot(res(3,:))


