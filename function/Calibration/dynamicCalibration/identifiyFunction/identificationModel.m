function wrench = identificationModel(motionPara,identifyModel,varargin)
%  varargin = T01
%Returns  1: �������ʶģ��Ϊ�ڲ�ģ�ͣ����ÿ���ؽڵĹؽ����أ�
%         2: �����ʶģ��Ϊ�ⲿģ�ͣ����������ϵ�����������أ�
[tau,ft] = ID_NewtonEuler(motionPara); %% ����ÿ���ؽڴ������غ�����

if strcmp(identifyModel,'External') % ���base����ϵ�����������أ�
    if nargin > 2
        T01 = varargin{1};
        R = T01(1:3,1:3);
        p = T01(1:3,4);
        wrench = ([R  zeros(3,3); skew(p)*R R])*ft(:,1); % �ؽ�1 ��������ӳ�䵽base��
    else
        wrench = ft(:,1);
    end
elseif  strcmp(identifyModel,'Internal')
    wrench = tau;
else
    message('choose model');
end
end
