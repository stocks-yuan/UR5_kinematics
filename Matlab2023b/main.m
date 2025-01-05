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

q0 = [-84.84,-84.60,-108.11,-78.76,91.39,-1.78];
q0 = deg2rad(q0);  
theta = [93.14,-62.68,108.27,-135.56,-66.46,15.59];
theta_rad = deg2rad(theta);  
% theta_rad = [1,1,1,1,0.000,1];
T = forward_kinematics(theta_rad,d,a,alpha);
theta2 = inverse_kinematics(T);

qq1 = forward_kinematics(theta2(1,:),d,a,alpha);
qq2 = forward_kinematics(theta2(2,:),d,a,alpha);
qq3 = forward_kinematics(theta2(3,:),d,a,alpha);
qq4 = forward_kinematics(theta2(4,:),d,a,alpha);
qq5 = forward_kinematics(theta2(5,:),d,a,alpha);
qq6 = forward_kinematics(theta2(6,:),d,a,alpha);
