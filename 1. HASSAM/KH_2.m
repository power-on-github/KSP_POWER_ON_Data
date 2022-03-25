%% Forward Kinematics
la = 87; lb = 65.5; lc = 65.5; ld = 62; le = 73;
L = [la lb lc ld le];
A = [0.175781250000000   10.5468750000000   22.0605468750000   -9.84375000000000   10.6347656250000   -9.37500000000000]; %각 링크 사이의 각도
A = deg2rad(A)
ForT = Ry(A(1)) * Ty(L(1)) * Rz(A(2)) * Ty(L(2)) * Rz(A(3)) * Tx(L(3)) * Ty(-L(4)) * Ry(-A(4)) * Rz(A(5)) * Rx(A(6)) * Tx(L(5));

ForT
%% Inverse Kinematics
la = 87; lb = 65.5; lc = 65.5; ld = 62; le = 73;
L = [la lb lc ld le];
A = [0 0 0 0 0 0]; %각 링크 사이의 각도, 최종적으로 구하고자 하는 값
T = [0.715756686946492,-0.697528126304066,0.0338655888199334,128.847379609153;0.677286355827341,0.705171880128572,0.209799455876450,183.904226233044;-0.170222082296926,-0.127228662221505,0.977157771400210,-12.6612098317925;0,0,0,1]; %end effecter의 변환행렬, 즉 목표하는 orientation
%T = [-1 0 0 -3 ; 0 -1 0 2 ; 0 0 1 0 ; 0 0 0 1];

%theta 1, 2, 3 구하기

a_ = T ;
a = a_(1:3,4); %순서대로 ax, ay, az

C1 = [0 ; 0 ; 0]; %순서대로 1x, 1y, 1z
C2 = [0 ; 0 ; 0]; %순서대로 2x, 2y, 2z
    
C1(1) = sqrt(a(1)^2 + a(3)^2);  % +- 2개의 해
C1(2) = a(2);
C1(3) = 0;

C2(2) = (a(1)^2 + (a(2)-L(1))^2 + a(3)^2 + L(2)^2 - L(3)^2 - L(4)^2) / (2*L(2));
C2(1) = -sqrt((a(1)^2 + a(3)^2 + (a(2)-L(1))^2 - C2(2)^2));    % +- 2개의 해
C2(3) = 0;

m1_1 = [C1(1) C1(3) ; C1(3) -C1(1)];
m1_2 = [a(1) ; a(3)];
t1_mat = (1/(C1(1)^2 + C1(3)^2))* m1_1 * m1_2;

for i = 1:2
    if (t1_mat(i) >= 1)
        t1_mat(i) = 1; 
    elseif (t1_mat(i) <= -1)
        t1_mat(i) = -1; 
    end      
end

m2_1 = [C2(1) C2(2) ; -C2(2) C2(1)];
m2_2 = [C1(1) ; C1(2) - L(1)];
t2_mat = (1/(C2(1)^2 + C2(2)^2))* m2_1 * m2_2;

for i = 1:2
    if (t2_mat(i) >= 1)
        t2_mat(i) = 1; 
    elseif (t2_mat(i) <= -1)
        t2_mat(i) = -1; 
    end      
end

m3_1 = [L(3) -L(4) ; L(4) L(3)];
m3_2 = [C2(1) ; C2(2) - L(2)];
t3_mat = (1/(L(3)^2 + L(4)^2))* m3_1 * m3_2;

for i = 1:2
    if (t3_mat(i) >= 1)
        t3_mat(i) = 1; 
    elseif (t3_mat(i) <= -1)
        t3_mat(i) = -1; 
    end      
end

A(1) = asin(t1_mat(2)); % theta 1
A(2) = asin(t2_mat(2)); % theta 2
A(3) = asin(t3_mat(2)); % theta 3

%theta 4, 5, 6 구하기

R = Ry(A(1))*Ty(L(1))*Rz(A(2))*Ty(L(2))*Rz(A(3))*Tx(L(3))*Ty(-L(4));
B = inv(R)*T;

A(4) = atan2(B(3,1) , B(1,1)); %theta 4
A(5) = asin(B(2,1)); %theta 5
A(6) = asin(-sin(A(4))*B(1,2) + cos(A(4))*B(3,2)); %theta 6

A
rad2deg(A)
%% function 정의 

function T=Tx(x) % x만큼 x축으로 이동하는 동차변환
T=[1 0 0 x; 0 1 0 0; 0 0 1 0; 0 0 0 1];
end 

function T=Ty(y) % y만큼 y축으로 이동
T=[1 0 0 0; 0 1 0 y; 0 0 1 0; 0 0 0 1];
end 

function T=Tz(z) % z만큼 z축으로 이동
T=[1 0 0 0; 0 1 0 0; 0 0 1 z; 0 0 0 1];
end 

function T=Rx(qx) % qx만큼 x축으로 회전하는 동차변환
T=[1 0 0 0; 0 cos(qx) -sin(qx) 0;
0 sin(qx) cos(qx) 0; 0 0 0 1];
end 

function T=Ry(qy) % qy만큼 y축으로 회전
T=[cos(qy) 0 sin(qy) 0; 0 1 0 0;
-sin(qy) 0 cos(qy) 0; 0 0 0 1];
end 

function T=Rz(qz) % qz만큼 z축으로 회전
T=[cos(qz) -sin(qz) 0 0; sin(qz) cos(qz) 0 0;
0 0 1 0; 0 0 0 1];
end 