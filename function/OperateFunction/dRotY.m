function dR = dRoty(t, deg)
% *** used in �˶�ѧ������ʶ ***
% ���뻡�ȣ��õ���ת���� Ry �ĵ�����
    if nargin > 1 && strcmp(deg, 'deg')
        t = t *pi/180;
    end
    ct = cos(t);
    st = sin(t);
    dR = [
        -st  0   ct
        0   0   0
       -ct  0   -st];
end
