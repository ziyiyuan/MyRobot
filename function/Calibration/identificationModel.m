
function wrench = identificationModel(motionPara,identificationModel,varargin)
%  varargin = T01
%Returns  1: 如果，辨识模型为内部模型，输出每个关节的关节力矩；
%         2: 如果辨识模型为外部模型，输出基坐标系处的力和力矩；
[tau,ft] = ID_NewtonEuler(motionPara);

if strcmp(identificationModel,'External') % 输出base坐标系处的力和力矩；
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
