clear
clc

K=[1  1  1
   1  1 -1
   1 -1  1 
   1 -1 -1
   -1  1  1
   -1  1 -1
   -1 -1  1 
   -1 -1 -1];
theta = [-38.65,-147.24,-116.13,-18.80,81.84,63.71];
theta_rad = deg2rad(theta);   

theta_rad
   
Td=FW(theta_rad)

for i=1:8
Q=IK(Td,K(i,:));

%%%%%%将各个关节角的值转换到[-pi,pi]
for j=1:6
    if Q(j)<-pi
      Q(j)=Q(j)+2*pi;
    elseif Q(j)>pi 
      Q(j)=Q(j)-2*pi;
    else
       Q(j)=Q(j);
    end
end

T1=FW(Q);
delta(i)=norm(T1-Td);
AQ(i,:)=Q;

end

AQ

