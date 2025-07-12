% UR5 机械臂 DH 参数定义
% DH 参数：[theta d a alpha]
DH_params = [
    0   0.08916   0      pi/2;
    0   0        -0.425  0;
    0   0        -0.39225 0;
    0   0.10915   0      pi/2;
    0   0.09465   0     -pi/2;
    0   0.0823    0      0
];

N = 30000;  % 随机采样数量
q_limits = [-360, 360]; % 关节角范围（度）

% 初始化存储位置
x = zeros(1, N);
y = zeros(1, N);
z = zeros(1, N);

% 蒙特卡罗采样
for i = 1:N
    q = (q_limits(2) - q_limits(1)) * rand(1, 6) + q_limits(1); % 随机生成六个关节角
    q = deg2rad(q); % 转换为弧度
    
    % 计算正向运动学
    T = eye(4);
    for j = 1:6
        theta = q(j) + DH_params(j, 1);
        d = DH_params(j, 2);
        a = DH_params(j, 3);
        alpha = DH_params(j, 4);
        
        Tj = [
            cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)  a*cos(theta);
            sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha)  a*sin(theta);
            0           sin(alpha)             cos(alpha)             d;
            0           0                      0                      1
        ];
        
        T = T * Tj;
    end
    
    % 提取末端执行器位置
    x(i) = T(1, 4);
    y(i) = T(2, 4);
    z(i) = T(3, 4);
end

% 在一个窗口中渲染四个图像
tiledlayout(2,2);

% XY平面
nexttile;
scatter(x, y, 1, 'r');
xlabel('X'); ylabel('Y');
title('UR5 XY平面工作域');
grid on;
axis equal;

% XZ平面
nexttile;
scatter(x, z, 1, 'g');
xlabel('X'); ylabel('Z');
title('UR5 XZ平面工作域');
grid on;
axis equal;

% YZ平面
nexttile;
scatter(y, z, 1, 'm');
xlabel('Y'); ylabel('Z');
title('UR5 YZ平面工作域');
grid on;
axis equal;

% 三维工作域
nexttile;
scatter3(x, y, z, 1, 'b');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('UR5 机械臂工作域分析 (蒙特卡罗法)');
grid on;
axis equal;

% 输出工作域范围
x_range = [min(x), max(x)];
y_range = [min(y), max(y)];
z_range = [min(z), max(z)];

fprintf('X 方向工作域范围: [%f, %f]\n', x_range(1), x_range(2));
fprintf('Y 方向工作域范围: [%f, %f]\n', y_range(1), y_range(2));
fprintf('Z 方向工作域范围: [%f, %f]\n', z_range(1), z_range(2));
