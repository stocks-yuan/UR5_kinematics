function [T] = FW(q)

alfa=pi/2;
d1=89.159;
a2=-425 ;
a3=-392.25 ;
d4=109.15;
d5=94.65;
d6=82.3;
T1=stdtrans(q(1),     d1,  0, alfa);
T2=stdtrans(q(2),     0,  a2, 0);
T3=stdtrans(q(3),     0,  a3, 0);
T4=stdtrans(q(4),     d4,  0, alfa);
T5=stdtrans(q(5),     d5,  0, -alfa);
T6=stdtrans(q(6),     d6,  0, 0);

T=T1*T2*T3*T4*T5*T6;
end
