function T4 = TRANS(a2, a3, d1, d2, p_offset, q)
T4 = zeros(16,1);
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

T4(1) = cos(q1 - q2 + q3 - q4)/2 + cos(q1 + q2 - q3 + q4)/2;
T4(2) = sin(q1 - q2 + q3 - q4)/2 - sin(q1 + q2 - q3 + q4)/2;
T4(3) = sin(q1);
T4(4) = d2*sin(q1) - a2*cos(q1)*sin(q2) + a3*cos(q1)*cos(q2)*sin(q3) - a3*cos(q1)*cos(q3)*sin(q2) + p_offset*cos(q1)*cos(q2)*cos(q3)*sin(q4) - p_offset*cos(q1)*cos(q2)*cos(q4)*sin(q3) + p_offset*cos(q1)*cos(q3)*cos(q4)*sin(q2) + p_offset*cos(q1)*sin(q2)*sin(q3)*sin(q4);
T4(5) = sin(q1 - q2 + q3 - q4)/2 + sin(q1 + q2 - q3 + q4)/2;
T4(6) = cos(q1 + q2 - q3 + q4)/2 - cos(q1 - q2 + q3 - q4)/2;
T4(7) = -cos(q1);
T4(8) = a3*cos(q2)*sin(q1)*sin(q3) - a2*sin(q1)*sin(q2) - d2*cos(q1) - a3*cos(q3)*sin(q1)*sin(q2) + p_offset*cos(q2)*cos(q3)*sin(q1)*sin(q4) - p_offset*cos(q2)*cos(q4)*sin(q1)*sin(q3) + p_offset*cos(q3)*cos(q4)*sin(q1)*sin(q2) + p_offset*sin(q1)*sin(q2)*sin(q3)*sin(q4);
T4(9) = sin(q2 - q3 + q4);
T4(10) = cos(q2 - q3 + q4);
T4(11) = 0;
T4(12) = d1 - p_offset*cos(q2 - q3 + q4) + a2*cos(q2) + a3*cos(q2 - q3);
T4(13) = 0;
T4(14) = 0;
T4(15) = 0;
T4(16) = 1;
end