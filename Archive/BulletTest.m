%Bullet Import test

% clear all
% clc
close all

A = csvread('Data.txt');

 time = cumsum(A(:,1));
 y = A(:,2);
 rot = A(:,3);
 selectedRot = [];
 selectedTime = [];
 
 for i = 2: size(y,1)-1
 if (y(i)<y(i+1))&&(y(i)<y(i-1))
     selectedRot(end+1) = rot(i);
     selectedTime(end+1) = time(i);
 end
 end
 
 
 
% posX = A(:,2)-A(1,2);
% posY = A(:,3);
% 
% subplot(2,1,1)
% plot(time,posX)
% subplot(2,1,2)
% plot(time,posY);
hold on
plot(time,A(:,3),'*');
%plot(selectedRot)
plot(selectedTime,ones(size(selectedTime,1)),'*')

spokes = 6;
slope = 20*pi/180;
alpha = pi*2/spokes;
g = 9.81
L = 0.5;
stepcount = 20;


beta1 = alpha/2 + slope;
beta2 = alpha/2 - slope;

w = zeros(stepcount,1);
w(1) = 0;
for i = 2:stepcount
w(i+1) = sqrt(cos(alpha)^2*(w(i)^2 + 2*g/L*(1-cos(beta1)))-2*g/L*(1-cos(beta2)));

end

plot(w)

hold off
