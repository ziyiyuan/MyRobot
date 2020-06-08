
function wrench = identificationModel(motionPara,identificationModel,varargin)
%  varargin = T01
%Returns  1: �������ʶģ��Ϊ�ڲ�ģ�ͣ����ÿ���ؽڵĹؽ����أ�
%         2: �����ʶģ��Ϊ�ⲿģ�ͣ����������ϵ�����������أ�
[tau,ft] = ID_NewtonEuler(motionPara);

if strcmp(identificationModel,'External') % ���base����ϵ�����������أ�
    T01 = varargin{1};
    R = T01(1:3,1:3);
    p = T01(1:3,4);
    wrench = ([R  zeros(3,3); skew(p)*R R])*ft(:,1);
elseif  strcmp(identificationModel,'Internal')
    wrench = tau;
else
    message('choose model');
end
end
