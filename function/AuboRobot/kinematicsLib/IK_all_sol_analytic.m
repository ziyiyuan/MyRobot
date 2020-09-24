function  qAllSols = IK_all_sol_analytic(T, DHMatrix, qLimit)
% IK_all_sol_analytic  计算所有存在的逆解解析解，最多8组解；在关节角限制范围内；目前只支持[-pi,pi]
% 输入参数：
%   T: 目标位置和姿态，6*1
%   DHMatrix: DH参数矩阵，顺序为 [alpha, a, d, theta]，6*4
%   qLimit:关节角限制，6*1
% 输出参数：
%   qSols：解的数组，6*n;
% 调用说明：
%   qSols = IK_all_sol_analytic(T, DHMatrix, qLimit)； 输出所有在关节限制内存在的解析解；
%   qSols = IK_all_sol_analytic(T，DHMatrix) :输出所有在[-175,175]关节限制内存在的解析解；

% 版本号V1.0，编写于2020/8/27，修改于2020/9/1，作者：ziyi

if nargin > 3
    error('输入变量过多！');
elseif nargin == 2
    qLimit = ones(6,1)*pi; % 默认情况下为[-pi,pi]
end

qAllSols = [];
num_sols = 0;
ZERO_THRESH = 0.00000001;

a2 = DHMatrix(3,2);
a3 = DHMatrix(4,2);
d1 = DHMatrix(1,3);
d2 = DHMatrix(2,3);
d5 = DHMatrix(5,3);
d6 = DHMatrix(6,3);

nx = T(1,1); ox = T(1,2); ax = T(1,3); px = T(1,4);
ny = T(2,1); oy = T(2,2); ay = T(2,3); py = T(2,4);
nz = T(3,1); oz = T(3,2); az = T(3,3); pz = T(3,4);

%% ////////////////////////////// q1 //////////////////////////////
q1 =[]; %[2];
A1 = d6*ay - py;
B1 = d6*ax - px;
R1 = A1^2 + B1^2 - d2^2;
if R1 < 0
    disp('no solution!')
    return ;
else
    R12 = sqrt(R1);
    q1(1) = antiSinCos(A1, B1) - antiSinCos(d2, R12);%值域可能超过（-pi,pi],需要约束在区间内
    q1(2) = antiSinCos(A1, B1) - antiSinCos(d2, -R12);
    for i = 1:1:2
        while(q1(i) > pi)
            q1(i) = q1(i) - 2 * pi;
        end
        while(q1(i) < -pi)
            q1(i) = q1(i) + 2 * pi;
        end
    end
end

%% ////////////////////////////// q5 //////////////////////////////
q5 = []; %[2][2];
for i = 1:1:2
    c1 =  cos(q1(i));
    s1 =  sin(q1(i));
    B5 = -ay * c1 + ax * s1;
    M5 = (-ny * c1 + nx * s1);
    N5 = (-oy * c1 + ox * s1);
    R5 = sqrt(M5 * M5 + N5 * N5);
    
    q5(i,1) = antiSinCos(R5, B5);
    q5(i,2) = antiSinCos(-R5, B5);
end

%% axis 6 3 2 4
for i = 1:1:2
    for j = 1:1:2
        c1 = cos(q1(i)); s1 = sin(q1(i));
        s5 = sin(q5(i,j));
        %% ////////////////////////////// q6 //////////////////////////////
        if(abs(s5) < ZERO_THRESH)
            %% singularity
            disp('joint 5 singularity')
            break;
        else
            A6 = (-oy * c1 + ox * s1);
            B6 = (ny * c1 - nx * s1);
            q6 = antiSinCos(A6 * s5, B6 * s5);
        end
        
        %% ////////////////////////////////////////////////////////////////////////////////
        
        q2 = []; q3 = []; q4 = []; %[2];
        %% ///////////////////////////// RRR jos (q2,q3,q4) ////////////////////////////
        s6 = cos(q6); c6 = sin(q6);
        
        pp1 = c1*(ax*d6 - px + d5*ox*s6 + d5*nx*c6) +  s1*(ay*d6 - py + d5*oy*s6 + d5*ny*c6);
        pp2 = -d1 - az*d6 + pz - d5*oz*s6 - d5*nz*c6;
        B3 = (pp1^2 + pp2^2 - a2^2 - a3^2)/(2*a2*a3 );
        
        if((1 - B3 * B3) < ZERO_THRESH)
%             disp('lose solution')
            continue;
        else
            A3 = sqrt(1 - B3 * B3);
            q3(1) = antiSinCos(A3, B3);
            q3(2) = antiSinCos(-A3, B3);
        end
        
        A4 = -c1 * (ox * s6 + nx * c6) - s1 * (oy * s6 + ny * c6);
        B4 = oz * s6 + nz * c6;
        
        for k = 1:1:2
            c3 = cos(q3(k)); s3 = sin(q3(k));
            p1 = a2 + a3*c3;
            p2 = a3*s3;
            A2 = pp1 * p1 + pp2 * p2;
            B2 = pp2 * p1 - pp1 * p2;
            
            q2(k) = antiSinCos(A2, B2);
            
            s2 = sin(q2(k)); c2 = cos(q2(k));
            A41 = pp1 - a2 * s2;
            B41 = pp2 - a2 * c2;
            q4(k) = antiSinCos(A4, B4) - antiSinCos(A41, B41);
            
            while(q4(k) > pi)
                q4(k) = q4(k) - 2 * pi;
            end
            while(q4(k) < -pi)
                q4(k) = q4(k) + 2 * pi;
            end
            
            num_sols = num_sols + 1;
            qAllSols(1,num_sols) = q1(i);     qAllSols(2, num_sols) = q2(k);
            qAllSols(3,num_sols) = q3(k);    qAllSols(4,num_sols) = q4(k);
            qAllSols(5,num_sols) = q5(i,j);   qAllSols(6,num_sols) = q6;

        end
    end
end
index = [];
% choose sols under joint limits
for i = 1:1:size(qAllSols,2)
    for j = 1:1:size(qAllSols,1)
        if abs(qAllSols(j,i)) > qLimit(j)
            index = [index,i];
            break;
        end
    end
end
qAllSols(:,index) = [];
end