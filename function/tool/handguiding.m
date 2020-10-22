clc
clear all
close all
N = 2000;
NUM = 2;
addpath( genpath( 'C:\Users\Sherry\Desktop\MyRobot\dataLib\handguidingPosition' ) )
data = load('handguidingData.txt');
% load('filter.mat')
qc = data(:,1:6)';
qcd = data(:,7:12)';
V = var(qc(NUM,:))
% refdata = filter.motionTraj;
qc = qc(NUM,:);
qcd = qcd(NUM,:);

dim = 1;
dt = 0.005;
I_d = eye(dim);

delta_a = 100;%加速度扰动
delta_m = 0.5;

Q = diag([0.5*dt^2,dt,1])*delta_a;
R = delta_m * eye(dim); 

A = [I_d,dt*I_d,dt^2*I_d/2;
    zeros(dim),I_d,dt*I_d;
    zeros(dim,2*dim),I_d];
C = [I_d,zeros(dim,2*dim)];

x1 = [0,0,0]';
P1 = eye(dim*3);
if 1
    for i = 1:1:size(qc,2)
        x2 = A*x1;
        P2 = A*P1*A' + Q;
        K1 = P2*C'*inv(C*P2*C' + R);
        y1 = qc(i);
        x1 = x2 + K1*(y1 - C*x2);
        P1 = (eye(dim*3) - K1 * C)*P2;
        res(:,i) = x1;
    end
end

% figure(1)
% q = refdata.q';
% qd = diff(q)/dt;
% qdd = diff(q,2)/(dt^2);
% plot(q(:,NUM))
% hold on
% plot(qd(:,NUM))
% hold on
% plot(qdd(:,NUM))
% % legend('q','qd','qdd')
% % title('reff data')
% hold on

% figure(2)
plot(res')
hold on
plot(qc)
hold on
plot(qcd)

legend('q','qd','qdd','qc','qcd')

% for i = 2:1:size(res,2)
%     res(3,i) = res(3,i-1)*0.6 +  res(3,i)*0.4;
% end
% hold on
% plot(res(3,:))


