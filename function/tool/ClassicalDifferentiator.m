function y = ClassicalDifferentiator(T,h,x)
% ����΢����
% input: T = 0.02; % �˲�����Ƶ�� T > h
%        h = 0.005;% ���ֲ����� һ����ڲ���Ƶ��
%        x Ϊ N*1 ά��������
% output: y = dx/dt
N = size(x,1);
xDelay = zeros(N+1,1);
y = zeros(N,1);
for k=1:1:N
    xDelay(k+1) = xDelay(k) - h*(xDelay(k) - x(k))/T;
    y(k) = (x(k)-xDelay(k))/T;
end
end