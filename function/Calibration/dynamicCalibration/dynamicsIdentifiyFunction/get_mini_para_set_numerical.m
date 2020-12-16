function [Col,beta] = get_mini_para_set_numerical(regression)
% �����ع������ֵ�����õ���С������
% Col.i : �����޹ص��е��Ʊ�
% Col.c : ������ص��У�����ռ���
%% matrix characteristics
% W*X = Y
% W1 * X1 + W2 * X2 = Y; W2 = L1 * beta;
% W1*(X1 + beta * X2 ) = Y;
% res = X1 + beta * X2;
%
% 1: ȥ���޷���ʶ�Ĳ�����Ӧ���� r_u_index��
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
% QR �ֽ��ҵ������޹ص���
[QQ,RR] = qr(regression);
% RR = qr(regression);// have different accuracy with [qq,rr] = qr()????
ee = size(RR,1)*eps*max(abs(diag(RR)));
C_1 = abs(diag(RR)) >  ee;%% �����޹ص���
Col.i = COL(C_1);
Col.c = COL(~C_1);
%
R1 = RR(C_1,C_1);% �����Ǿ���
R2 = RR(C_1,~C_1);
beta = inv(R1)*R2; % so beta depend on qr decompose
beta(abs(beta) < 1e-10) = 0;
end