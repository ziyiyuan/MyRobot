
clc
clear all

L1 = 0.5;
L2 = 0.5;
m1 = 20;
m2 = 10;
g = 9.81;
a1 = L2*L2*m2 + L1*L1*(m1 + m1)
a2 = L1*L1*m2
a3 = L2^2*m2
a4 = (m1 + m2) * L1 * g
a5 = m2 * L2 * g


% c2 = cos(q(2));
% c12 = cos(q(1) + q(2));
% s2 = sin(q(2))

t = 0:0.1:25
T1 = 10 * sin(2*pi/25*t)
T2 = 2.5 * square(2*pi*t,50) + 2.5
figure(1)
plot(t,T1,'r--')
hold on
plot(t,T2)

% M = [a1 + 2*a2*c2 a3 + a2 * c2;a3 + a2 *c2, a3]
% alpha = [a4*c1 + a5*c12; a2*qd(1)*(qd(1) + qd(2))*s2 + a5 *c12]
% 
% P = M*qd;



    
    
  