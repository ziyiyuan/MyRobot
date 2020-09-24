clc
clear all
close all

Robot = get_cad_model_para()
%%
q_in = load('ikdata.txt');
quat = load('quat.txt');
dt = 0.05;
quat = quat';
for i = 1:1: size(q_in,1)-1
    %     T = forward_kinematics(Robot.DH,q_in(i,:)');
    %     quat(:,i) = RotMatrixtoQuat(T(1:3,1:3));
    delta_q = QuatLeftMultiplyQuat(QuatInverse(quat(:,i)))*quat(:,i+1);
    [axis,angle] = QuattoAxisAngle(delta_q);
    w(:,i) = real(axis*angle/dt);
end

xa = [quat(:,1)',zeros(1,3),zeros(1,3)];
q1 = quat(:,1);
w1 = zeros(3,1);
wd1 = zeros(3,1);

a1 = 10^(-6);
b1 = 10^(-6);
c1 = 10^(-4);

Q = diag([c1,c1,c1,c1,a1,a1,a1,b1,b1,b1]);
R = 10^(-10)*eye(4);

x1 = [quat(:,1);zeros(3,1);zeros(3,1)];
P1 = eye(10);

for i = 1:1:size(quat,2)-1
    q1 = x1(1:4);
    w1 = x1(5:7);
    wd1 = x1(8:10);
    
    a1 = 1/2*(w1*dt + 1/2 * wd1 * dt^2);
    qadd = QuatExp(a1);
    q2 = QuatLeftMultiplyQuat(qadd) * q1;
    qadd_m = QuatLeftMultiplyQuat(qadd);
    
    M11 =  [diffQ(1)*qadd,diffQ(2)*qadd,diffQ(3)*qadd,diffQ(4)*qadd];
    M12 =  [diffQ(2)*qadd_m*q1, diffQ(3)*qadd_m*q1,diffQ(4)*qadd_m*q1]*1/2*dt;
    M13 =  M12*1/2*dt;
    
    G = dq(x1,dt);
    D = [M11,M12,M13];
    e =  G - D
        D =  dq(x1,dt);
    E = [zeros(3,4),eye(3),eye(3).*dt];
    F = [zeros(3,7),eye(3)];
    
    A = [D;E;F];
    C = [eye(4,4),zeros(4,6)];
    
    x2 = A * x1;
    P2 = A * P1 * A' + Q;
    K1 = P2 * C' * inv(C*P2*C'+R);
    y2 = quat(:,i+1);
    x1 = x2 + K1*(y2 - C*x2);
    x1 = x1/norm(x1(1:4));
%     x3(i) = norm(x1(1:4));
    P1 = (eye(10) - K1*C)*P2;
    res(:,i) = x1;
end


close all
N = 4;
plot(res(N,:))
hold on
plot(quat(N,:))
legend('after','before')
%
close all
plot(res(5,:))
hold on
plot(w(1,:))
legend('after','before')

function A = dq(x,dt)

q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);
wx = x(5);
wy = x(6);
wz = x(7);
wdx = x(8);
wdy = x(9);
wdz  = x(10);

A1 = [ (- wx^2/8 - wy^2/8 - wz^2/8)*dt^2 + 1,            - (dt*wx)/2 - (dt^2*wdx)/4,            - (dt*wy)/2 - (dt^2*wdy)/4,            - (dt*wz)/2 - (dt^2*wdz)/4, - (dt*q1)/2 - (dt^2*q0*wx)/4, - (dt*q2)/2 - (dt^2*q0*wy)/4, - (dt*q3)/2 - (dt^2*q0*wz)/4, -(dt^2*q1)/4, -(dt^2*q2)/4, -(dt^2*q3)/4];
A2 = [              (wdx*dt^2)/4 + (wx*dt)/2, (- wx^2/8 - wy^2/8 - wz^2/8)*dt^2 + 1,            - (dt*wz)/2 - (dt^2*wdz)/4,              (wdy*dt^2)/4 + (wy*dt)/2,   (dt*q0)/2 - (dt^2*q1*wx)/4,   (dt*q3)/2 - (dt^2*q1*wy)/4, - (dt*q2)/2 - (dt^2*q1*wz)/4,  (dt^2*q0)/4,  (dt^2*q3)/4, -(dt^2*q2)/4];
A3 = [              (wdy*dt^2)/4 + (wy*dt)/2,              (wdz*dt^2)/4 + (wz*dt)/2, (- wx^2/8 - wy^2/8 - wz^2/8)*dt^2 + 1,            - (dt*wx)/2 - (dt^2*wdx)/4, - (dt*q3)/2 - (dt^2*q2*wx)/4,   (dt*q0)/2 - (dt^2*q2*wy)/4,   (dt*q1)/2 - (dt^2*q2*wz)/4, -(dt^2*q3)/4,  (dt^2*q0)/4,  (dt^2*q1)/4];
A4 = [              (wdz*dt^2)/4 + (wz*dt)/2,            - (dt*wy)/2 - (dt^2*wdy)/4,              (wdx*dt^2)/4 + (wx*dt)/2, (- wx^2/8 - wy^2/8 - wz^2/8)*dt^2 + 1,   (dt*q2)/2 - (dt^2*q3*wx)/4, - (dt*q1)/2 - (dt^2*q3*wy)/4,   (dt*q0)/2 - (dt^2*q3*wz)/4,  (dt^2*q2)/4, -(dt^2*q1)/4,  (dt^2*q0)/4];
A = [A1;A2;A3;A4];
end

function M = diffQ(n)
if n == 1
    M = eye(4);
elseif n == 2
    M = [     0    -1     0     0
        1     0     0     0
        0     0     0    -1
        0     0     1     0];
    
elseif n == 3
    M = [     0     0    -1     0
        0     0     0     1
        1     0     0     0
        0    -1     0     0];
elseif n == 4
    M = [     0     0     0    -1
        0     0    -1     0
        0     1     0     0
        1     0     0     0];
end
end




