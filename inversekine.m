function [ L2, L3,th2,th3] = inversekine( px, py )
ToDeg = 180/pi;
%ToRad = pi/180;

px = 8.768;
py = 21.568;

L2 = 12.8;
L3 = 12.4;


th3=acos((px^2+py^2-L2^2-L3^2)/(2*L2*L3));        %逆运动学方程
th2=atan(py/px)-atan((L3*sin(th3))/(L2+L3*cos(th3)));
%d2=py*cos(th1)-px*sin(th1)-100*sin(th3);

fprintf('th2=%4.2f \n',th2*ToDeg);   %观察输出结果
%fprintf('d2=%4.2f \n',d2);
fprintf('th3=%4.2f \n',th3*ToDeg);

end



