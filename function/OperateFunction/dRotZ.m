function dR = dRotZ(t, deg)
% *** used in �˶�ѧ������ʶ ***
% ���뻡�ȣ��õ���ת���� Rz �ĵ�����
if nargin > 1 && strcmp(deg, 'deg')
    t = t *pi/180;
end

ct = cos(t);
st = sin(t);
dR = [
    -st  -ct  0
    ct   -st  0
    0    0   0];
end
