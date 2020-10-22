function y = TrackingDifferentiator(T,h,x)
% 二阶线性微分跟踪器
% input: T = 0.02; % 滤波采样频率 T > h
%        h = 0.005;% 积分步长， 一般等于采样频率
%        x 为 N*1 维列向量。
% output: y = dx/dt
N = size(x,1);
x1  = zeros(N+1,1);
x2 = zeros(N+1,1);
y = zeros(N,1);
for k=1:1:N
    x1(k+1) = x1(k) + h*x2(k);
    x2(k+1) = x2(k) - h*((x1(k)-x(k))/T^2 + 2*x2(k)/T);
    y(k) = x2(k);
end
end