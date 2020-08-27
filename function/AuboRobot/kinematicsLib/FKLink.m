function Tadd = FKLink(q,n)
%return  T01 0r T12 OR T23
global Robot 
DH = Robot.DH;

TT = [[]];
for i = 1:1:Robot.DOF
    TT{i} = transfer(DH(i,1),DH(i,2),DH(i,3),DH(i,4) + q(i));
end
Tadd = TT{n};
end