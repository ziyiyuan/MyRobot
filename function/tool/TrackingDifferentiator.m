function y = TrackingDifferentiator(T,h,x)
% ��������΢�ָ�����
% input: T = 0.02; % �˲�����Ƶ�� T > h
%        h = 0.005;% ���ֲ����� һ����ڲ���Ƶ��
%        x Ϊ N*1 ά��������
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