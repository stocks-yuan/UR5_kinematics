function T = T_para(theta,d,a,alpha)
%input: [rad]

%modify DH matrix
% T=[cos(theta),-sin(theta),0,a;
%     sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-d*sin(alpha);
%     sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),d*cos(alpha);
%     0,0,0,1];

%standard DH matrix
T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
    sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
    0,sin(alpha),cos(alpha),d;
    0,0,0,1];
end
