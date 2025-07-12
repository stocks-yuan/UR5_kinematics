% startup_rvc
clear;clc;
%UR5 standard_DH parameter
a=[0,-0.42500,-0.39225,0,0,0];
d=[0.089159,0,0,0.10915,0.09465,0.08230];
alpha=[pi/2,0,0,pi/2,-pi/2,0];


% %UR5 modify_DH parameter
% a=[0,0,-0.42500,-0.39225,0,0];
% d=[0.089159,0,0,0.10915,0.09465,0.08230];
% alpha=[0,pi/2,0,0,pi/2,-pi/2];

% 建立UR5机械臂模型
L1 = Link('d', d(1),  'a', a(1), 'alpha', alpha(1),  'standard');
L2 = Link('d', d(2),  'a', a(2), 'alpha', alpha(2),  'standard');
L3 = Link('d', d(3),  'a', a(3), 'alpha', alpha(3),  'standard');
L4 = Link('d', d(4),  'a', a(4), 'alpha', alpha(4),  'standard');
L5 = Link('d', d(5),  'a', a(5), 'alpha', alpha(5),  'standard');
L6 = Link('d', d(6),  'a', a(6), 'alpha', alpha(6),  'standard');
tool_robot = SerialLink([L1,L2,L3,L4,L5,L6], 'name', 'UR5');
tool_robot.display();
view(3);
tool_robot.teach();

% q0 = [-84.84,-84.60,-108.11,-78.76,91.39,-1.78];
% q0 = deg2rad(q0);  
theta =[-38.65,-147.24,-116.13,-18.80,81.84,63.71];
theta_rad = deg2rad(theta);  

T = forward_kinematics(theta_rad,d,a,alpha);
theta2 = inverse_kinematics(T);
%%%%%%将各个关节角的值转换到[-pi,pi]
for i=1:8
    for j=1:6
        if theta2(i,j)<-pi
          theta2(i,j)=theta2(i,j)+2*pi;
        elseif theta2(i,j)>pi 
          theta2(i,j)=theta2(i,j)-2*pi;
        else
           theta2(i,j)=theta2(i,j);
        end
    end
end

disp(theta2)