% 画 pos 点 处的位置和姿态向量； ori 用四元数表示；scaleFactor 为向量长度；
function plotOri(pos, ori, scaleFactor)
    qw = ori(1);
    qx = ori(2);
    qy = ori(3);
    qz = ori(4);
    eerot0 = 1.0 - 2.0*qy*qy - 2.0*qz*qz;
    eerot1 = 2.0*qx*qy - 2.0*qz*qw;
    eerot2 = 2.0*qx*qz + 2.0*qy*qw;
    eerot3 = 2.0*qx*qy + 2.0*qz*qw;
    eerot4 = 1.0 - 2.0*qx*qx - 2.0*qz*qz;
    eerot5 = 2.0*qy*qz - 2.0*qx*qw;
    eerot6 = 2.0*qx*qz - 2.0*qy*qw;
    eerot7 = 2.0*qy*qz + 2.0*qx*qw;
    eerot8 = 1.0 - 2.0*qx*qx - 2.0*qy*qy;
    hold on;
    line([pos(1) pos(1)+eerot0*scaleFactor],[pos(2),pos(2)+eerot3*scaleFactor],[pos(3) pos(3)+eerot6*scaleFactor],'color','r');
    hold on
    line([pos(1) pos(1)+eerot1*scaleFactor],[pos(2),pos(2)+eerot4*scaleFactor],[pos(3) pos(3)+eerot7*scaleFactor],'color','g');
    hold on
    line([pos(1) pos(1)+eerot2*scaleFactor],[pos(2),pos(2)+eerot5*scaleFactor],[pos(3) pos(3)+eerot8*scaleFactor],'color','b');
end

