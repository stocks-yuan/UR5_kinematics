function [Q] = IK(T,K)

d1=89.459;
a2=-425 ;
a3=-392.25 ;
d4=109.15;
d5=94.65;
d6=82.3;
k1=K(1);
k2=K(2);
k3=K(3);


px=T(1,4); py=T(2,4); pz=T(3,4);
ax=T(1,3); ay=T(2,3); az=T(3,3);
ox=T(1,2); oy=T(2,2); oz=T(3,2);
nx=T(1,1); ny=T(2,1); nz=T(3,1);


A1=px - ax*d6;
B1=py - ay*d6;
q1=atan2(d4,k1*sqrt(A1^2+B1^2-d4^2))+atan2(B1,A1);

% %  + (ny*cos(q1) - nx*sin(q1))*sin(q6)+(oy*cos(q1)- ox*sin(q1))*cos(q6)

A6=ny*cos(q1) - nx*sin(q1);
B6=oy*cos(q1) - ox*sin(q1);
q6=atan2(0,k2)-atan2(B6,A6);

%%%%%%%%%%%%%%%
p1= cos(q1)*(px - ax*d6 + d5*ox*cos(q6) + d5*nx*sin(q6)) + sin(q1)*(py - ay*d6 + d5*oy*cos(q6) + d5*ny*sin(q6));
p2= pz - d1 - az*d6 + d5*oz*cos(q6) + d5*nz*sin(q6);


c3=(p1^2+p2^2-a2^2 - a3^2)/(2*a2*a3);
s3=k3*sqrt(1-c3^2);
q3=atan2(s3,c3);

% % % a3^2 - 2*cos(q2 + q3)*a3*p1 - 2*sin(q2 + q3)*a3*p2 + p1^2 + p2^2-a2^2
A23=2*a3*p1;
B23=2*a3*p2;
C23=a3^2 + p1^2 + p2^2-a2^2;
q23=atan2(k3*sqrt(A23^2+B23^2-C23^2),C23)+atan2(B23,A23);

%%%%%%%%%%%%%%%
q2=q23-q3;


% % % ax*cos(q5)*sin(q1) - ay*cos(q1)*cos(q5) - ny*cos(q1)*cos(q6)*sin(q5) + nx*cos(q6)*sin(q1)*sin(q5) + oy*cos(q1)*sin(q5)*sin(q6) - ox*sin(q1)*sin(q5)*sin(q6)
%%%%%%%%%%%%%%%
% % (ax*sin(q1) - ay*cos(q1))*cos(q5)+( - ny*cos(q1)*cos(q6) + nx*cos(q6)*sin(q1) + oy*cos(q1)*sin(q6) - ox*sin(q1)*sin(q6))*sin(q5)
A5=( - ny*cos(q1)*cos(q6) + nx*cos(q6)*sin(q1) + oy*cos(q1)*sin(q6) - ox*sin(q1)*sin(q6));
B5= ax*sin(q1) - ay*cos(q1);
q5=atan2(0,1)+atan2(A5,B5);



%%%%%%%%%%%%%
A4= nz*cos(q5)*cos(q6) - az*sin(q5) - oz*cos(q5)*sin(q6);
B4=-cos(q1)*(ax*sin(q5) - nx*cos(q5)*cos(q6) + ox*cos(q5)*sin(q6)) - sin(q1)*(ay*sin(q5) - ny*cos(q5)*cos(q6) + oy*cos(q5)*sin(q6));
q234=atan2(A4,B4);

q4=q234-q23;

Q=[q1 q2 q3 q4 q5 q6];
end
