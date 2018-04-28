%% Calibration Curve
%  Garret Hilton, Austin Rosenbaum, Meshal Albiaz
%  EELE 488 Montana State Univerity 
%
%  Second Order Polynomial Calibration curve 
%  inputs:
%    leftT PWM to turn to the far left 
%    rightT PWM to turn to the far right
%    middle PWM to drive straight
%
%    Initial Conditions:
%        y'(63) = 0 
%        y(0) = leftT
%        y(127) = rightT
%    Equation:
%    y(x) = for x <  64 y(x) = A*x^2  + B*x  + C 
%           for x >= 64 y(x) = A2*x^2 + B2*x + D 
% 

leftT = 1250;
rigthT = 1850;
middle = 1550;
tt = 0:1:127;

C = leftT;
D = rigthT;

A = C / (63.5*63.5) - middle/(63.5*63.5);

B = (-A)*63.5 - (C / 63.5) + middle/(63.5);

A2 = D / (63.5*63.5) - middle/(63.5*63.5);

B2 = -(A2)*63.5 - (D / 63.5) + middle/(63.5);

y = zeros(1, length(tt));
for kk = 0:(length(tt)-1)
    if kk <= 63
        y(kk+1) = A*kk*kk + B*kk + C;
    else
        y(kk+1) = A2*kk*kk + B2*kk + D;
    end
    
end
xV = [63.5, 63.5];
yV = [1200, 1900];
points = [leftT, middle, rigthT];
fprintf('\ny1(x) = %.3fx^2 + %.3fx + %d\n', A, B, C)
eq1 = sprintf('\ny1(x) = %.3fx^2 + %.3fx + %d\n', A, B, C);
fprintf('\ny2(x) = %.3fx^2 + %.3fx + %d\n', -A, -B, D)
eq2 = sprintf('\ny2(x) = %.3fx^2 + %.3fx + %d\n', -A, -B, D);
xPoints = [0, 63, 127];
plot(tt, y, 'b', xPoints, points, 'r o', xV, yV, 'g--')
text(5, 1600, eq1);
text (68, 1450, eq2);
title('Two Second Order Steering Curves')
ylabel('PWM to Servo')
xlabel ('Center of Line Position')

%%
center = 1500;
left = 1300;
right = 1800;

slope = (center - left)/64;
b1 = center - slope*64;
slope2 = (right - center)/64;
b2 = right - slope2*128;
tt = 1:1:128;
%y = slope*tt + b1;
%y2 = slope2*tt + b2;

y = zeros(1, 128);
for kk = 1:128
    if kk <= 63
        y(kk) = slope*kk + b1;
    else
        y(kk) = slope2*kk + b2;
    end
    
end

plot(tt, y)
