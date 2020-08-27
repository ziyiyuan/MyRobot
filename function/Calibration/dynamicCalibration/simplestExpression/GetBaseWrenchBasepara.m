function wrench = GetBaseWrenchBasepara(motionPara)
% wrench: ��������ڻ�����ϵ�����������أ�
% motionPara: current position, velocity and acceleration
% robotType:
% gravity:
global Robot 
[index, DP] = GetRobotIdyMimParaSet('External'); %% �õ���С������
gravity = Robot.gravity;
% p{1} = [0 0 XZ1 0 YZ1 ZZ1 MX1 MY1 MZ1 M1]';
% p{2} = [XX2 XY2 XZ2 0 YZ2 ZZ2 MX2 MY2 0 0]';
% p{3} = [XX3 XY3 XZ3 0 YZ3 ZZ3 MX3 MY3 0 0]';
% p{4} = [XX4 XY4 XZ4 0 YZ4 ZZ4 MX4 MY4 0 0]';
% p{5} = [XX5 XY5 XZ5 0 YZ5 ZZ5 MX5 MY5 0 0]';
% p{6} = [XX6 XY6 XZ6 0 YZ6 ZZ6 MX6 MY6 0 0]';

XZ1 = DP(1);  YZ1 = DP(2);   ZZ1 = DP(3);  MX1 = DP(4);  MY1 = DP(5);   MZ1 = DP(6);  M1 = DP(7);                                                             %MZ1 = 0;
XX2 = DP(8);  XY2 = DP(9);   XZ2 = DP(10); YZ2 = DP(11); ZZ2 = DP(12);  MX2 = DP(13); MY2 = DP(14);  %MZ2 = 0;
XX3 = DP(15); XY3 = DP(16);  XZ3 = DP(17); YZ3 = DP(18); ZZ3 = DP(19);  MX3 = DP(20); MY3 = DP(21);  %MZ3 = 0;
XX4 = DP(22); XY4 = DP(23);  XZ4 = DP(24); YZ4 = DP(25); ZZ4 = DP(26);  MX4 = DP(27); MY4 = DP(28);  %MZ4 = 0;
XX5 = DP(29); XY5 = DP(30);  XZ5 = DP(31); YZ5 = DP(32); ZZ5 = DP(33);  MX5 = DP(34); MY5 = DP(35);  %MZ5 = 0;
XX6 = DP(36); XY6 = DP(37);  XZ6 = DP(38); YZ6 = DP(39); ZZ6 = DP(40);  MX6 = DP(41); MY6 = DP(42);  %MZ6 = 0;

a2 = Robot.Para.KP.a(3); a3 = Robot.Para.KP.a(4);
d1 = Robot.Para.KP.d(1); d2 = Robot.Para.KP.d(2); d5 = Robot.Para.KP.d(5); d6 = Robot.Para.KP.d(6);
q = motionPara.q;
qd = motionPara.qd;
qdd = motionPara.qdd;

s1 = sin(q(1)); s2 = sin(q(2)); s3 = sin(q(3)); s4 = sin(q(4)); s5 = sin(q(5)); s6 = sin(q(6));
c1 = cos(q(1)); c2 = cos(q(2)); c3 = cos(q(3)); c4 = cos(q(4)); c5 = cos(q(5)); c6 = cos(q(6));

tmp0 = -gravity(1)*s1;
tmp1 = -gravity(2)*c1;
tmp2 = tmp0 - tmp1;
tmp3 = qd(1)*s2;
tmp4 = qd(2)*tmp3;
tmp5 = c2*qdd(1) - tmp4;
tmp6 = c2*qd(1);
tmp7 = qd(2)*tmp6;
tmp8 = -qdd(1)*s2 - tmp7;
tmp9 = qd(1)^2;
tmp10 = c3*tmp6 + s3*tmp3;
tmp11 = -c3*tmp8 - qd(3)*tmp10 - s3*tmp5;
tmp12 = c3*tmp3 - s3*tmp6;
tmp13 = c4*tmp10 - s4*tmp12;
tmp14 = c3*tmp5 + qd(3)*tmp12 - s3*tmp8;
tmp15 = c4*tmp11 + qd(4)*tmp13 + s4*tmp14;
tmp16 = MY4*tmp15;
tmp17 = c4*tmp12 + s4*tmp10;
tmp18 = c4*tmp14 - qd(4)*tmp17 - s4*tmp11;
tmp19 = MX4*tmp18;
tmp20 = qd(2) - qd(3) + qd(4);
tmp21 = MX4*tmp17*tmp20;
tmp22 = tmp13*tmp20;
tmp23 = MY4*tmp22;
tmp24 = -qd(2) + qd(3);
tmp25 = tmp10*tmp24;
tmp26 = qdd(5) + tmp18;
tmp27 = c5*tmp17 - s5*tmp20;
tmp28 = qd(5) + tmp13;
tmp29 = c6*tmp27 + s6*tmp28;
tmp30 = c5*tmp20;
tmp31 = s5*tmp17;
tmp32 = -tmp30 - tmp31;
tmp33 = qdd(2) - qdd(3) + qdd(4);
tmp34 = c5*tmp15 + qd(5)*tmp32 - s5*tmp33;
tmp35 = c6*tmp26 - qd(6)*tmp29 - s6*tmp34;
tmp36 = tmp28^2;
tmp37 = c6*tmp28 - s6*tmp27;
tmp38 = c6*tmp34 + qd(6)*tmp37 + s6*tmp26;
tmp39 = -MX5*tmp32 + MY5*tmp27;
tmp40 = qd(6) + tmp30 + tmp31;
tmp41 = MX5*tmp26 - MX6*tmp29*tmp40 + MX6*tmp35 - MY5*tmp36 - MY6*tmp37*tmp40 - MY6*tmp38 - tmp27*tmp39;
tmp42 = c5*tmp41;
tmp43 = -MX6*tmp37 + MY6*tmp29;
tmp44 = tmp40^2;
tmp45 = c5*tmp33;
tmp46 = qd(5)*tmp27;
tmp47 = s5*tmp15;
tmp48 = qdd(6) + tmp45 + tmp46 + tmp47;
tmp49 = -MX6*tmp44 - MY6*tmp48 + tmp37*tmp43;
tmp50 = MX6*tmp48 - MY6*tmp44 - tmp29*tmp43;
tmp51 = c6*tmp49 - s6*tmp50;
tmp52 = -MX5*tmp36 - MY5*tmp26 + tmp32*tmp39 + tmp51;
tmp53 = s5*tmp52;
tmp54 = -tmp42 - tmp53;
tmp55 = MX3*tmp11 - MX3*tmp25 - MY3*tmp12*tmp24 - MY3*tmp14 + tmp16 - tmp19 + tmp21 + tmp23 + tmp54;
tmp56 = M1*tmp2 + MX1*qdd(1) + MX2*tmp7 - MX2*tmp8 - MY1*tmp9 - MY2*tmp4 + MY2*tmp5 + tmp55;
tmp57 = s1*tmp56;
tmp58 = gravity(1)*c1 + gravity(2)*s1;
tmp59 = qd(2)^2;
tmp60 = MX2*tmp3 + MY2*tmp6;
tmp61 = -qdd(2) + qdd(3);
tmp62 = tmp24^2;
tmp63 = -MX3*tmp12 + MY3*tmp10;
tmp64 = -MX4*tmp13 + MY4*tmp17;
tmp65 = tmp20^2;
tmp66 = c5*tmp52 - s5*tmp41;
tmp67 = -MX4*tmp65 - MY4*tmp33 + tmp13*tmp64 + tmp66;
tmp68 = -tmp45 - tmp46 - tmp47;
tmp69 = tmp28*tmp32;
tmp70 = c6*tmp50 + s6*tmp49;
tmp71 = MX4*tmp33 + MX5*tmp27*tmp28 - MX5*tmp68 - MY4*tmp65 + MY5*tmp34 + MY5*tmp69 - tmp17*tmp64 + tmp70;
tmp72 = c4*tmp67 - s4*tmp71;
tmp73 = MX3*tmp61 - MY3*tmp62 - tmp10*tmp63 + tmp72;
tmp74 = -MX3*tmp62 - MY3*tmp61 + c4*tmp71 + s4*tmp67 + tmp12*tmp63;
tmp75 = -c3*tmp73 - s3*tmp74;
tmp76 = MX2*qdd(2) - MY2*tmp59 - tmp6*tmp60 + tmp75;
tmp77 = -MX2*tmp59 - MY2*qdd(2) + c3*tmp74 - s3*tmp73 - tmp3*tmp60;
tmp78 = c2*tmp76 + s2*tmp77;
tmp79 = M1*tmp58 - MX1*tmp9 - MY1*qdd(1) + tmp78;
tmp80 = c1*tmp79;
tmp81 = c1*tmp56;
tmp82 = s1*tmp79;
tmp83 = c2*tmp77 - s2*tmp76;
tmp84 = -d2*qdd(1) + tmp58;
tmp85 = gravity(3)*s2 + c2*tmp84;
tmp86 = a2*tmp9;
tmp87 = -gravity(3)*c2 + s2*tmp84;
tmp88 = -a2*tmp59 - s2^2*tmp86 + tmp87;
tmp89 = a2*qdd(2) - c2*s2*tmp86 + tmp85;
tmp90 = c3*tmp88 - s3*tmp89;
tmp91 = -a3*tmp12^2 - a3*tmp62 + tmp90;
tmp92 = -c3*tmp89 - s3*tmp88;
tmp93 = a3*tmp10*tmp12 + a3*tmp61 + tmp92;
tmp94 = c4*tmp91 - s4*tmp93;
tmp95 = XY4*tmp17 + YZ4*tmp20;
tmp96 = XX3*tmp10 + XY3*tmp12 + XZ3*tmp24;
tmp97 = c4*tmp93 + s4*tmp91;
tmp98 = d5*tmp13*tmp17 - d5*tmp33 + tmp97;
tmp99 = a2*tmp8;
tmp100 = a2*tmp7;
tmp101 = d2*tmp9;
tmp102 = -tmp101 + tmp2;
tmp103 = a3*tmp11 - a3*tmp25 + tmp100 + tmp102 - tmp99;
tmp104 = d5*tmp15 + d5*tmp22 + tmp103;
tmp105 = c5*tmp98 - s5*tmp104;
tmp106 = d6*tmp26 - d6*tmp27*tmp32 + tmp105;
tmp107 = -d5*tmp17^2 - d5*tmp65 + tmp94;
tmp108 = -d6*tmp34 - d6*tmp69 + tmp107;
tmp109 = XX6*tmp29 + XY6*tmp37 + XZ6*tmp40;
tmp110 = XX5*tmp27 + XY5*tmp32 + XZ5*tmp28;
tmp111 = XY6*tmp29 + YZ6*tmp40;
tmp112 = XZ5*tmp27 + YZ5*tmp32 + ZZ5*tmp28;
tmp113 = -MX5*tmp107 - MX6*(c6*tmp108 - s6*tmp106) + MY6*(c6*tmp106 + s6*tmp108) + XY5*tmp34 - XZ6*tmp38 + YZ5*tmp26 - YZ6*tmp35 - ZZ6*tmp48 + tmp109*tmp37 + tmp110*tmp28 - tmp111*tmp29 - tmp112*tmp27;
tmp114 = c5*tmp104;
tmp115 = s5*tmp98;
tmp116 = -d6*tmp27^2 - d6*tmp36 + tmp114 + tmp115;
tmp117 = XZ6*tmp29 + YZ6*tmp37 + ZZ6*tmp40;
tmp118 = MY6*tmp116 + XX6*tmp38 + XY6*tmp35 + XZ6*tmp48 - tmp111*tmp40 + tmp117*tmp37;
tmp119 = -MX6*tmp116 + XY6*tmp38 + YZ6*tmp48 + tmp109*tmp40 - tmp117*tmp29;
tmp120 = XY5*tmp27 + YZ5*tmp28;
tmp121 = MY5*tmp107 + XX5*tmp34 + XY5*tmp68 + XZ5*tmp26 + c6*tmp118 - d6*tmp70 - s6*tmp119 + tmp112*tmp32 - tmp120*tmp28;
tmp122 = XY3*tmp10 + YZ3*tmp24;
tmp123 = XX4*tmp17 + XY4*tmp13 + XZ4*tmp20;
tmp124 = XY2*tmp6 + YZ2*qd(2);
tmp125 = XX2*tmp6 - XY2*tmp3 + XZ2*qd(2);
tmp126 = gravity(3)*MX1 + MX2*tmp85 - MX3*tmp92 + MX4*tmp94 - MY2*tmp87 + MY3*tmp90 - MY4*tmp97 + MZ1*tmp58 + XZ1*tmp9 + XZ2*tmp5 - XZ3*tmp14 + XZ4*tmp15 + YZ1*qdd(1) + YZ2*tmp8 - YZ3*tmp11 + YZ4*tmp18 + ZZ2*qdd(2) - ZZ3*tmp61 + ZZ4*tmp33 + a2*tmp75 - a3*tmp72 - c5*tmp113 - d5*tmp66 - s5*tmp121 - tmp10*tmp122 + tmp12*tmp96 - tmp123*tmp13 + tmp124*tmp6 + tmp125*tmp3 + tmp17*tmp95;
tmp127 = XZ4*tmp17 + YZ4*tmp13 + ZZ4*tmp20;
tmp128 = MY4*tmp103 + XX4*tmp15 + XY4*tmp18 + XZ4*tmp33 + c5*tmp121 + d5*tmp54 - s5*tmp113 + tmp127*tmp13 - tmp20*tmp95;
tmp129 = -tmp0 + tmp1 - tmp100 + tmp101 + tmp99;
tmp130 = -MX4*tmp103 + MX5*(-tmp114 - tmp115) - MY5*tmp105 + XY4*tmp15 + XZ5*tmp34 + YZ4*tmp33 + YZ5*tmp68 + ZZ5*tmp26 + c6*tmp119 + d6*tmp51 + s6*tmp118 - tmp110*tmp32 + tmp120*tmp27 + tmp123*tmp20 - tmp127*tmp17;
tmp131 = XZ3*tmp10 + YZ3*tmp12 + ZZ3*tmp24;
tmp132 = -MX3*tmp129 + XY3*tmp14 + YZ3*tmp61 - a3*(-tmp16 + tmp19 - tmp21 - tmp23 + tmp42 + tmp53) + c4*tmp128 - s4*tmp130 - tmp10*tmp131 + tmp24*tmp96;
tmp133 = MY3*tmp129 + XX3*tmp14 + XY3*tmp11 + XZ3*tmp61 + c4*tmp130 + s4*tmp128 + tmp12*tmp131 - tmp122*tmp24;
tmp134 = XZ2*tmp6 - YZ2*tmp3 + ZZ2*qd(2);
tmp135 = -MX2*tmp102 + XY2*tmp5 + YZ2*qdd(2) - a2*tmp55 - c3*tmp132 + qd(2)*tmp125 - s3*tmp133 - tmp134*tmp6;
tmp136 = MY2*tmp102 + XX2*tmp5 + XY2*tmp8 + XZ2*qdd(2) + c3*tmp133 - qd(2)*tmp124 - s3*tmp132 - tmp134*tmp3;
tmp137 = -gravity(3)*MY1 - MZ1*tmp2 + XZ1*qdd(1) - YZ1*tmp9 + c2*tmp135 + d2*tmp83 + s2*tmp136;
wrench(1) = tmp57 - tmp80;
wrench(2) = -tmp81 - tmp82;
wrench(3) = -gravity(3)*M1 + tmp83;
wrench(4) = -c1*tmp137 + d1*tmp81 + d1*tmp82 + s1*tmp126;
wrench(5) = -c1*tmp126 + d1*tmp57 - d1*tmp80 - s1*tmp137;
wrench(6) = MX1*tmp2 - MY1*tmp58 + ZZ1*qdd(1) + c2*tmp136 - d2*tmp78 - s2*tmp135;
end
