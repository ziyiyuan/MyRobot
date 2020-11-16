
% 计算工具末端的速度和角速度，返回【v,w】
% global robotType
% robotType = 'I5';
% %%
% 
% Robot = get_cad_model_para(robotType);
% q = [1;1;1;1;1;1]
% qd = [1;2;3;4;5;6]
% 
% Pt = [0;0.1;0.2]
% T = forward_kinematics(q, Robot.DH, 6)
% J = cal_jacobian(q,Robot.DH)
% v6 = J{6}* qd
% v6(1:3) + cross(v6(4:6),Pt)
% v6(1:3) + cross(v6(4:6),T(1:3,1:3)*Pt)
% Pt0 = T(1:3,4) + T(1:3,1:3)*Pt
% 
% tt = cal_velocity(q,qd, Robot,Pt)



function tt = cal_velocity(q,qd, Robot,Pt)
c1 = cos(q(1));c2 = cos(q(2));c3 = cos(q(3));c4 = cos(q(4));c5 = cos(q(5));c6 = cos(q(6));
s1 = sin(q(1));s2 = sin(q(2));s3 = sin(q(3));s4 = sin(q(4));s5 = sin(q(5));s6 = sin(q(6));
a2 = Robot.Para.KP.a(3);
a3 = Robot.Para.KP.a(4);
d1 = Robot.Para.KP.d(1);
d2 = Robot.Para.KP.d(2);
d5 = Robot.Para.KP.d(5);
d6 = Robot.Para.KP.d(6);
tcx = Pt(1);
tcy = Pt(2);
tcz = Pt(3);

qd1 = qd(1);
qd2 = qd(2);
qd3 = qd(3);
qd4 = qd(4);
qd5 = qd(5);
qd6 = qd(6);

tmp0 = c1;
tmp1 = c5;
tmp2 = tmp0*tmp1;
tmp3 = s1;
tmp4 = c2;
tmp5 = c3;
tmp6 = tmp4*tmp5;
tmp7 = s2;
tmp8 = s3;
tmp9 = tmp7*tmp8;
tmp10 = tmp3*tmp6 + tmp3*tmp9;
tmp11 = c4;
tmp12 = tmp4*tmp8;
tmp13 = tmp5*tmp7;
tmp14 = tmp12*tmp3 - tmp13*tmp3;
tmp15 = s4;
tmp16 = tmp10*tmp11 + tmp14*tmp15;
tmp17 = s5;
tmp18 = tmp16*tmp17;
tmp19 = d6*(-tmp18 + tmp2);
tmp20 = a3*tmp14;
tmp21 = -tmp10*tmp15 + tmp11*tmp14;
tmp22 = d5*tmp21;
tmp23 = tmp0*tmp17 + tmp1*tmp16;
tmp24 = c6;
tmp25 = s6;
tmp26 = tcx*(tmp21*tmp25 + tmp23*tmp24);
tmp27 = tcy*(tmp21*tmp24 - tmp23*tmp25);
tmp28 = tmp18 - tmp2;
tmp29 = tcz*tmp28;
tmp30 = a2*tmp7;
tmp31 = tmp3*tmp30;
tmp32 = tmp6 + tmp9;
tmp33 = -tmp12 + tmp13;
tmp34 = tmp11*tmp33 + tmp15*tmp32;
tmp35 = tmp17*tmp34;
tmp36 = tmp11*tmp32 - tmp15*tmp33;
tmp37 = tmp1*tmp34;
tmp38 = tcx*(tmp24*tmp37 + tmp25*tmp36) + tcy*(tmp24*tmp36 - tmp25*tmp37) + tcz*tmp35;
tmp39 = d6*tmp35 + tmp38;
tmp40 = tmp26 + tmp27 + tmp29;
tmp41 = -tmp19 + tmp40;
tmp42 = d5*tmp36 + tmp39;
tmp43 = a3*tmp32 + tmp42;
tmp44 = qd3*tmp0;
tmp45 = a2*tmp4 + tmp43;
tmp46 = qd2*tmp0;
tmp47 = qd4*tmp0;
tmp48 = tmp0*tmp12 - tmp0*tmp13;
tmp49 = tmp0*tmp6 + tmp0*tmp9;
tmp50 = tmp11*tmp48 - tmp15*tmp49;
tmp51 = tmp11*tmp49 + tmp15*tmp48;
tmp52 = tmp17*tmp51;
tmp53 = tmp1*tmp3;
tmp54 = tmp1*tmp51 - tmp17*tmp3;
tmp55 = tmp52 + tmp53;
tmp56 = tcx*(tmp24*tmp54 + tmp25*tmp50) + tcy*(tmp24*tmp50 - tmp25*tmp54) + tcz*tmp55;
tmp57 = -d6*(-tmp52 - tmp53) + tmp56;
tmp58 = d5*tmp50 + tmp57;
tmp59 = a3*tmp48 + tmp58;
tmp60 = -tmp0*tmp30 + tmp59;
tmp61 = qd3*tmp3;
tmp62 = qd2*tmp3;
tmp63 = qd4*tmp3;
tmp64 = tmp22 + tmp41;
tmp65 = tmp20 + tmp64;
tt(1) = qd1*(d2*tmp0 + tmp19 - tmp20 - tmp22 - tmp26 - tmp27 - tmp29 + tmp31) + qd5*(tmp21*tmp39 - tmp36*tmp41) + qd6*(tmp28*tmp38 - tmp35*tmp40) - tmp42*tmp47 + tmp43*tmp44 - tmp45*tmp46;
tt(2) = qd1*(d2*tmp3 + tmp60) + qd5*(tmp36*tmp57 - tmp39*tmp50) + qd6*(tmp35*tmp56 - tmp38*tmp55) - tmp42*tmp63 + tmp43*tmp61 - tmp45*tmp62;
tt(3) = qd2*(tmp0*tmp60 + tmp3*(-tmp31 + tmp65)) + qd3*(-tmp0*tmp59 - tmp3*tmp65) + qd4*(tmp0*tmp58 + tmp3*tmp64) + qd5*(-tmp21*tmp57 + tmp41*tmp50) + qd6*(-tmp28*tmp56 + tmp40*tmp55);
tt(4) = qd5*tmp50 + qd6*tmp55 - tmp61 + tmp62 + tmp63;
tt(5) = qd5*tmp21 + qd6*tmp28 + tmp44 - tmp46 - tmp47;
tt(6) = qd1 + qd5*tmp36 + qd6*tmp35;
end
