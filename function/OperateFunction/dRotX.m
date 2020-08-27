function dR = dRotx(t, deg)
% *** used in �˶�ѧ������ʶ ***
% ���뻡�ȣ��õ���ת���� Rx �ĵ�����

if nargin > 1 && strcmp(deg, 'deg')
    t = t *pi/180;
end

ct = cos(t);
st = sin(t);
dR = [
    0   0    0
    0   -st  -ct
    0   ct   -st
    ];
end
