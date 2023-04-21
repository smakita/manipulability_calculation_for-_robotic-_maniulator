clear
% DH parameters of 6DOF manipulator
dha=[0;-0.24365;-0.21325;0;0;0];
dhalpha=[pi/2;0;0;pi/2;-pi/2;0];
dhd=[0.1519;0;0;0.11235;0.08535;0.0819];
dth=[0;0;0;0;0;0];


% 同次変換行列の初期化 / Initialization of homogeneous transformation matrix
alpha = sym('lp',[1 6]); %alpha
theta = sym('th',[1 6]); %theta
%theta = [theta1;theta2;theta3;theta4;theta5;theta6];
% syms theta;
a = sym('a',[1 6]);
d = sym('d',[1 6]);
zero = sym('0');
one = sym('1');

% for jissen robot seigyo
a(1) = sym('0');
a(2) = sym('0');
a(3) = str2sym('L2');
alpha(1) = 0;
alpha(2) = pi/2;
alpha(3) = 0;
d(1) = sym('0');
d(2) = str2sym('-D2');
d(3) = str2sym('D3');

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
for i=1:3
%     a(i) = dha(i);
%     alpha(i) = dhalpha(i);
%     d(i) = dhd(i);
%     theta(i) = dth(i);
    Ta = [one,zero,zero,a(i);zero,one,zero,zero;zero,zero,one,zero;zero,zero,zero,one];
    Talpha=[one,zero,zero,zero;zero,cos(alpha(i)),-sin(alpha(i)),zero;zero,sin(alpha(i)),cos(alpha(i)),zero;zero,zero,zero,one];
    Td = [one,zero,zero,zero;zero,one,zero,zero;zero,zero,one,d(i);zero,zero,zero,one];
    Ttheta=[cos(theta(i)),-sin(theta(i)),zero,zero;sin(theta(i)),cos(theta(i)),zero,zero;zero,zero,one,zero;zero,zero,zero,one];
    Ttotal = Ttotal*Ta*Talpha*Td*Ttheta;
end
L = [zero;zero;zero;one]
Ttotal = simplify(Ttotal)
L = Ttotal*L

% disp(Ttotal)
% 
% Ta = [one,zero,zero,a;zero,one,zero,zero;zero,zero,one,zero;zero,zero,zero,one];
% Talpha=[one,zero,zero,zero;zero,cos(alpha),-sin(alpha),zero;zero,sin(alpha),cos(alpha),zero;zero,zero,zero,one];
% Td = [one,zero,zero,zero;zero,one,zero,zero;zero,zero,one,d;zero,zero,zero,one];
% %Ttheta=[cos(theta),-sin(theta),zero,zero;sin(theta),cos(theta),zero,zero;zero,zero,one,zero;zero,zero,zero,one];
% disp(Ta);
% disp(Talpha);
% disp(Td);
% disp(Ttheta);
% 
% Ttotal = Ta*Talpha*Td*Ttheta
% pos=[Ttotal(1,4),Ttotal(2,4),Ttotal(3,4)]
% J = jacobian(pos,theta)

% 同次変換行列
% function f = Ta(x)
%   T=eye(4,4);
%   T(1,4) = x;
%   f = T;
% end
% 
% function f = Talpha(x)
%   T=eye(4,4);
%   T(2,2) =  cos(x);
%   T(2,3) = -sin(x);
%   T(3,2) =  sin(x);
%   T(3,3) =  cos(x);
%   f = T;
% end
% 
% function f = Td(x)
%   T=eye(4,4);
%   T(3,4) = x;
%   f = T;
% end
% 
% function f = Ttheta(x)
%   T=eye(4,4);
%   T(1,1) =  cos(x);
%   T(1,2) = -sin(x);
%   T(2,1) =  sin(x);
%   T(2,2) =  cos(x);
%   f = T;
% end
