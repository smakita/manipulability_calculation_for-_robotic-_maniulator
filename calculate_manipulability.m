    clear
% DH parameters of 6DOF manipulator
dha=[0;-0.24365;-0.21325;0;0;0];
dhalpha=[pi/2;0;0;pi/2;-pi/2;0];
dhd=[0.1519;0;0;0.11235;0.08535;0.0819];
%dhth=[2.236,-1.873,-0.666,-0.400,0.919,3.410]; %pattern B
%dhth=[-1.765,0.191,-0.431,-4.695,-5.142,-2.886]; %pattern B'
%dhth=[2.261,-0.976,-0.280,-4.408,4.984,-0.277]; %pattern C
dhth=[-1.775,0.221,-0.233,1.538,1.337,2.865]; %pattern C'


% 同次変換行列の初期化 / Initialization of homogeneous transformation matrix
alpha = sym('lp',[1 6]); %alpha
theta = sym('th',[1 6]); %theta
a = sym('a',[1 6]);
d = sym('d',[1 6]);
zero = sym('0');
one = sym('1');

% for UR3
% a(1) = sym('0');
% a(2) = str2sym('-A2');
% a(3) = str2sym('-A3');
% a(4) = sym('0');
% a(5) = sym('0');
% a(6) = sym('0');
% d(1) = str2sym('D1');
% d(2) = sym('0');
% d(3) = sym('0');
% d(4) = str2sym('D4');
% d(5) = str2sym('D5');
% d(6) = str2sym('D6');
Ttotal = [one,zero,zero,zero;zero,one,zero,zero;zero,zero,one,zero;zero,zero,zero,one]
for i=1:6
    a(i) = dha(i);
    alpha(i) = dhalpha(i);
    d(i) = dhd(i);
%    theta(i) = dth(i);
    Ta = [one,zero,zero,a(i);zero,one,zero,zero;zero,zero,one,zero;zero,zero,zero,one];
    Talpha=[one,zero,zero,zero;zero,cos(alpha(i)),-sin(alpha(i)),zero;zero,sin(alpha(i)),cos(alpha(i)),zero;zero,zero,zero,one];
    Td = [one,zero,zero,zero;zero,one,zero,zero;zero,zero,one,d(i);zero,zero,zero,one];
    Ttheta=[cos(theta(i)),-sin(theta(i)),zero,zero;sin(theta(i)),cos(theta(i)),zero,zero;zero,zero,one,zero;zero,zero,zero,one];
    Ttotal = Ttotal*Td*Ttheta*Ta*Talpha;
end
L = [zero;zero;zero;one]
Ttotal = simplify(Ttotal)
L = Ttotal*L;
L = [L(1);L(2);L(3)]
Lval = subs(L,theta,dhth) %ベクトルL内のthetaに dhthの数値を代入
pos = [double(Lval(1));double(Lval(2));double(Lval(3))] %手先位置が正しいかどうかの確認のため

J=jacobian(L, theta) %ヤコビ行列のシンボリック計算
Jval = subs(J,theta,dhth) 
w = sqrt(det(Jval*Jval'))
manipulability = double(w)
