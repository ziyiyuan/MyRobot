% 基于广义动量观测器的算法验证；
% 模型：二连杆机构
clc
clear all
close all
load('PVAJ.mat')

L1 = 0.5;
L2 = 0.5;
m1 = 20;
m2 = 10;
g = 9.81;
a1 = L2*L2*m2 + L1*L1*(m1 + m1);
a2 = L1*L2*m2;
a3 = L2^2*m2;
a4 = (m1 + m2) * L1 * g;
a5 = m2 * L2 * g;
%%
dt = 0.005;
%%
N = 314;
q_all = zeros(2,N);
qd_all = zeros(2,N);
qdd_all = zeros(2,N);
 
q_all = [PVAJ{1, 1}(:,1), PVAJ{1, 2}(:,1)]'./15;
qd_all = diff(q_all,1,2)./dt;
qdd_all = diff(qd_all,1,2)./dt;
% qd_all = [PVAJ{1, 1}(:,2), PVAJ{1, 2}(:,2)]';
% qdd_all = [PVAJ{1, 1}(:,3), PVAJ{1, 2}(:,3)]';


N = size(qdd_all,2);

T = N*dt;
num_p = 10;

%%
% traj
t = 0:dt:T;

w = 1;

q_all = [sin(w*t + pi/2);sin(w*t)];
qd_all = [w*cos(w*t);w*cos(w*t)];
qdd_all = [-w*w*sin(w*t);-w*w*sin(w*t)];


y = (-square(2*pi/(T/num_p)*t,50) + 1)/2;
tau1 = y(1:N)*7; %% 方形波

tau1 = zeros(1,N);
tau1([30,31,32,33,34,35,36,37,50,51,52,80]) = 7; % 脉冲波形
% 正弦波
% y = -sin(2*pi/(T/num_p)*t);

% tau1 = y(1:N)*7;

tau2 = y(1:N)*3;

figure(1)
plot(tau1)

K0 = 150;% 和采样频率有关系
res = [];
%%
for i = 1:1:N
    q = q_all(:,i);
    qd = qd_all(:,i);
    qdd = qdd_all(:,i);
    tau_f(:,i) = [tau1(i);tau2(i)];
    
    c1 = cos(q(1));
    c2 = cos(q(2));
    s1 = sin(q(1));
    s2 = sin(q(2));
    s12 = sin(q(1) + q(2));
    c12 = cos(q(1) + q(2));
    
    M = [a1 + 2*a2*c2 a3 + a2 * c2;a3 + a2 *c2, a3];
    G = [a4 * c1 + a5 * c12;  a5 *c12];
    Cqd = [-a2 *qd(2)*(qd(2) + 2*qd(1))*s2; a2 * qd(1)^2*s2];
    CTqd = [0; a2*qd(1)*(qd(1) + qd(2))*s2];
    
    %  电机力矩
    tau_m(:,i) = M*qdd + G + Cqd - tau_f(:,i);
    %  动量
    Pt = M *qd
    
    % 计算 beta = G - C' * qd =  G + C*qd - Md*qd
    if i == 1
        P_hat = zeros(2,1);
        Mk_1 = M;
        Rk_1 = zeros(2,1);
        P0 = Pt;
    end
    
    beta = G - CTqd;

    beta1 = Cqd + G - (M - Mk_1)*qd./dt;
    P_hat = P_hat + (tau_m(:,i) - beta + Rk_1).* dt

    
    R = K0*(Pt - P_hat - P0)
    res = [res,R];
    Mk_1 = M;
    Rk_1 = R;
    
    E(:,i) = res(:,i) - tau_f(:,i);
    
end
index = 1;
figure(2)
plot(res(index,:))
hold on
plot(tau_f(index,:))
hold on
plot(E(index,:),'--')
legend('est_f','input_f','error')

figure(3)
plot(tau_m(index,:))
legend('电机力矩')

figure(4)
plot(q_all(index,:))
hold on 
plot(qd_all(index,:))
hold on 
plot(qdd_all(index,:))
title('pva')
legend('p','v','a')








    
    
  