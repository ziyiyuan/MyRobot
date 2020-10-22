function y = ClassicalDifferentiator(T,h,x)
% 经典微分器
% input: T = 0.02; % 滤波采样频率 T > h
%        h = 0.005;% 积分步长， 一般等于采样频率
%        x 为 N*1 维列向量。
% output: y = dx/dt
N = size(x,1);
xDelay = zeros(N+1,1);
y = zeros(N,1);
for k=1:1:N
    xDelay(k+1) = xDelay(k) - h*(xDelay(k) - x(k))/T;
    y(k) = (x(k)-xDelay(k))/T;
end
end