function [Col,beta] = get_mini_para_set_numerical(regression)
% 分析回归矩阵，数值分析得到最小参数集
% Col.i : 线性无关的列的哑标
% Col.c : 线性相关的列，和零空间列
%% matrix characteristics
% W*X = Y
% W1 * X1 + W2 * X2 = Y; W2 = L1 * beta;
% W1*(X1 + beta * X2 ) = Y;
% res = X1 + beta * X2;
%
% 1: 去掉无法辨识的参数对应的列 r_u_index；
COL = 1:1:size(regression,2);
% r_u_index = [];
% for i = size(regression,2):-1:1
%     e = norm(regression(:,i));
%     if e < 1e-8
%         r_u_index = [r_u_index,i];
%         regression(:,i) = [];
%     end
% end
% Col.u = COL(r_u_index);

Col.u = 0;
% COL(r_u_index) = [];
% QR 分解找到线性无关的列
[QQ,RR] = qr(regression);
% RR = qr(regression);// have different accuracy with [qq,rr] = qr()????
ee = size(RR,1)*eps*max(abs(diag(RR)));
C_1 = abs(diag(RR)) >  ee;%% 线性无关的列
Col.i = COL(C_1);
Col.c = COL(~C_1);
%
R1 = RR(C_1,C_1);% 上三角矩阵
R2 = RR(C_1,~C_1);
beta = inv(R1)*R2; % so beta depend on qr decompose
beta(abs(beta) < 1e-10) = 0;
end