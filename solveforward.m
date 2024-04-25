syms d2  A4  a1 a2 a3 theta1 theta2 theta3 nx ny nz ox oy oz ax ay az px py pz  U1

theta0 = 11;
pi = 3.14;
%这是用来求正运动学中各个dh矩阵的


%p01=[0 0 77 theta1]; theta1 = 0 
p1 = [0 0 77 0];
A1=forwardkine(p1);
%A1=simplify(A1);

disp('A1 = :')
disp(A1)


%p12=[0 pi/2 0 theta2-theta0];
p2 = [0 pi/2 0 theta2-theta0];
A2=forwardkine(p2);
disp('A2 = :')
disp(A2)
%A2=simplify(A2)

%p23=[130 0 0 theta3+theta0];
p3=[130 0 0 theta3+theta0];
A3=forwardkine(p3);
disp('A3 = :')
disp(A3)
%A3=simplify(A3)

%p34=[124 0 0 theta4];
p4=[124 0 0 theta3+theta0];
A4=forwardkine(p4);
%A4=simplify(A4)

%p45=[126 0 0 0];
p5=[126 0 0 0];
A5=forwardkine(p5);
disp('A5 = :')
disp(A5)
%A5=simplify(A5)

A6=A1*A2*A3*A4*A5;
%A6=simplify(A6)

disp('A6 = :')
disp(A6)

a1=inv(A1);        %求逆
a2=inv(A2);
a3=inv(A3);
a4=inv(A4);       
a5=inv(A5);

%a1=simplify(a1)
%a2=simplify(a2)
%a3=simplify(a3)
%a4=simplify(a4)
%a5=simplify(a5)


U=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];  %设出要求的矩阵U

U1=a4*a3*a2*a1*U;
%U1=simplify(U1)

