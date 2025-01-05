syms theta1 theta2 theta3 theta4 theta5 theta6 d1 d4 d5 d6 a2 a3
t01 = [cos(theta1), 0, sin(theta1), 0;
       sin(theta1), 0, -cos(theta1), 0;
       0, 1, 0, d1;
       0, 0, 0, 1];

t12 = [cos(theta2), -sin(theta2), 0, a2 * cos(theta2);
       sin(theta2), cos(theta2), 0, a2 * sin(theta2);
       0, 0, 1, 0;
       0, 0, 0, 1];

t23 = [cos(theta3), -sin(theta3), 0, a3 * cos(theta3);
       sin(theta3), cos(theta3), 0, a3 * sin(theta3);
       0, 0, 1, 0;
       0, 0, 0, 1];

t34 = [cos(theta4), 0, sin(theta4), 0;
       sin(theta4), 0, -cos(theta4), 0;
       0, 1, 0, d4;
       0, 0, 0, 1];

t45 = [cos(theta5), 0, -sin(theta5), 0;
       sin(theta5), 0, cos(theta5), 0;
       0, -1, 0, d5;
       0, 0, 0, 1];

t56 = [cos(theta6), -sin(theta6), 0, 0;
       sin(theta6), cos(theta6), 0, 0;
       0, 0, 1, d6;
       0, 0, 0, 1];

T = simplify(t01 * t12 * t23 * t34 * t45 * t56);

% 转换为 LaTeX 格式
latex_T = latex(T);

% 输出结果
disp(latex_T);


