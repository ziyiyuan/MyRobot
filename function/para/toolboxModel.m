function aubo_i = toolboxModel()
%Creates a aubo i5 for Robotics Toolbox with DH parameters. 
%Returns robot struct.
load('Robot.mat');
DH = Robot.DH;% alpha a d theta

%% standard DH para of aubo_i
if 0
    a2 = 0.408;
    a3 = 0.376;
    d1 = 0.122;
    d2 = 0.1215;
    d5 = 0.1025;
    d6 = 0.094;
    L1 = Link('d', d1, 'a', 0, 'alpha', -pi/2, 'offset', pi);
    L2 = Link('d', d2, 'a', a2, 'alpha', pi, 'offset', -pi/2);
    L3 = Link('d', 0, 'a', a3, 'alpha',pi, 'offset', 0);
    L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', -pi/2);
    L5 = Link('d', d5, 'a', 0, 'alpha', pi/2, 'offset', 0);
    L6 = Link('d', d6, 'a', 0, 'alpha',0, 'offset', 0);
    aubo_i5= SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'AUBO_I5');
    qz = [0, 0, 0, 0, 0, 0];
    aubo_i5.plot(qz)
    A = aubo_i5.fkine(qz)
end

%% modified model// theta d a alpha revelute
for i = 1:1:6
S(i) = Link([ 0, DH(i,3), DH(i,2), DH(i,1), 0], 'modified');
end
aubo_i= SerialLink(S, 'name', 'AUBO_i');
aubo_i.offset =  DH(:,4)';
qz = [0, 0, 0, 0, 0, 0]; % ¡„Œª
aubo_i.plot(qz); 
B = aubo_i.fkine(qz);

end