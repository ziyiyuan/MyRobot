function  q_sols = IK_all(T,DH)
% return all solution; ×î¶à8×é

q_sols = [];    %[8 * 6];
num_sols = 1;
ZERO_THRESH = 0.00000001;

a2 = DH(3,2);
a3 = DH(4,2);
d1 = DH(1,3);
d2 = DH(2,3);
d5 = DH(5,3);
d6 = DH(6,3);

nx = T(1,1); ox = T(1,2); ax = T(1,3); px = T(1,4);
ny = T(2,1); oy = T(2,2); ay = T(2,3); py = T(2,4);
nz = T(3,1); oz = T(3,2); az = T(3,3); pz = T(3,4);


%% ////////////////////////////// shoulder rotate jo (q1) //////////////////////////////
q1 =[]; %[2];
cx = -px + d6 * ax;
cy = -py + d6 * ay;
xy = sqrt(cx * cx + cy * cy);
RS = cx * cx + cy * cy - d2 * d2;
if RS < 0
    return ;
else
    R = sqrt(RS);
    %            q1[0] = atan2(cy, cx) - atan2(d2, R);
    q1(1) = antiSinCos(cy, cx) - antiSinCos(d2, R);
    %            if(q1[0] > pi) q1[0] -= 2 * pi;
    %            if(q1[0] <= -pi) q1[0] += 2 * pi;
    %            q1[1] = atan2(cy, cx) - atan2(d2, -R);
    q1(2) = antiSinCos(cy, cx) - antiSinCos(d2, -R);
    %            if(q1[1] > pi) q1[1] -= 2 * pi;
    %            if(q1[1] <= -pi) q1[1] += 2 * pi;
end


%% ////////////////////////////// wrist 2 jo (q5) //////////////////////////////
q5 = []; %[2][2];

for i = 1:1:2
    c1 =  cos(q1(i));
    s1 =  sin(q1(i));
    S52 = (1 - ay * ay) * c1 * c1 + (1 - ax * ax) * s1 * s1 - 2 * s1 * c1 * (nx * ny + ox * oy);
    s5 = sqrt(S52);
    %             s5 = -(px * c1 - d6 * (ax * c1 + ay * s1) + py * s1);
    c5 = -ay * c1 + ax * s1;
    %            q5(i,1) = atan2(s5, c5);
    %            q5(i,2) = atan2(-s5, c5);
    q5(i,1) = antiSinCos(s5, c5);
    q5(i,2) = antiSinCos(-s5, c5);
    
    if(abs(q5(i,1)) < ZERO_THRESH)
        q5(i,1) = 0.0;
    end
    if(abs(q5(i,2)) < ZERO_THRESH)
        q5(i,2) = 0.0;
    end
end

%% axis 6 3 2 4
for i = 1:1:2
    for j = 1:1:2
        c1 = cos(q1(i)); s1 = sin(q1(i));
        c5 = cos(q5(i,j)); s5 = sin(q5(i,j));
        %% ////////////////////////////// wrist 3 jo (q6) //////////////////////////////
        if(abs(s5) < ZERO_THRESH)
            %% singularity
            error('singularity');
        else
            s6 = (-oy * c1 + ox * s1) / s5;
            c6 = (-nx * s1 + ny * c1) / s5;
            %                q6 = atan2(s6, c6);
            q6 = antiSinCos(s6, c6);
            if(abs(q6) < ZERO_THRESH)
                q6 = 0.0;
            end
        end
        
        %% ////////////////////////////////////////////////////////////////////////////////
        
        q2 = []; q3 = []; q4 = []; %[2];
        %% ///////////////////////////// RRR jos (q2,q3,q4) ////////////////////////////
        c6 = cos(q6); s6 = sin(q6);
        p14x = -(s6*(nx*c1 + ny*s1) + c6*(ox*c1 + oy*s1));
        p14y = (oz * c6 + nz * s6);
        
        %             p13x = px * c1 - d6 * (ax * c1 + ay * s1) + d5 * p14x + py * s1;
        p13x = -d5*p14x + d6*(ax*c1 + ay*s1) - px*c1 - py*s1;
        p13y = pz - d1 - az * d6 - d5 * p14y;
        
        c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3);
        if(abs(abs(c3) - 1.0) < ZERO_THRESH)
            c3 = sign(c3);
        elseif(abs(c3) > 1.0)
            % TODO NO SOLUTION
            continue;
        end
        arccos = acos(c3);
        q3(1) = arccos;
        q3(2) = 2.0 * pi - arccos;
        if(q3(2) > pi)
            q3(2) = q3(2) - 2 * pi;
        end
        s3 = sin(arccos);
        B = a3 * s3;
        A = (a2 + a3 * c3);
        AB = A * A + B * B;
        %            q2[0] = atan2((B * p13x - A * p13y) / AB, (A * p13x + B * p13y) / AB);
        q2(1) = antiSinCos((A * p13x + B * p13y) / AB, (-B * p13x + A * p13y) / AB);
        B = -B;
        %            q2[1] = atan2((B * p13x - A * p13y) / AB, (A * p13x + B * p13y) / AB);
        q2(2) = antiSinCos((A * p13x + B * p13y) / AB, (-B * p13x + A * p13y) / AB);
        c23_0 = cos(q2(1) - q3(1));
        s23_0 = sin(q2(1) - q3(1));
        c23_1 = cos(q2(2) - q3(2));
        s23_1 = sin(q2(2) - q3(2));
        
        %           q4[0] = atan2(c23_0 * p14x - s23_0 * p14y, p14y * c23_0 + p14x * s23_0);
        %            q4[1] = atan2(c23_1 * p14x - s23_1 * p14y, p14y * c23_1 + p14x * s23_1);
        
        q4(1) = antiSinCos(c23_0 * p14x - s23_0 * p14y, p14y * c23_0 + p14x * s23_0);
        q4(2) = antiSinCos(c23_1 * p14x - s23_1 * p14y, p14y * c23_1 + p14x * s23_1);
        
        
        %% ////////////////////////////////////////////////////////////////////////////////
        for k = 1:1:2
            if(abs(q2(k)) < ZERO_THRESH)
                q2(k) = 0.0;
            end
            
            if(abs(q4(k)) < ZERO_THRESH)
                q4(k) = 0.0;
            end
            
            q_sols(num_sols , 1) = q1(i);     q_sols(num_sols , 2) = q2(k);
            q_sols(num_sols , 3) = q3(k);    q_sols(num_sols , 4) = q4(k);
            q_sols(num_sols , 5) = q5(i,j);   q_sols(num_sols , 6) = q6;
            num_sols = num_sols + 1;
        end
    end
end
end