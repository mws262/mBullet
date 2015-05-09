%Bullet Import Data:

% clear all
% clc
close all

A = csvread('Data.txt');

 time = cumsum(A(:,1));
 
 %PENCIL
 x = A(:,2)-A(1,2);
 y = A(:,3)-A(1,3);
 
 PE = A(:,4)-A(1,4);
 KE = A(:,5);
 
 subplot(2,1,1)
 title('Tipping Pencil');
 plot(time, x, time, y)
 legend('X Tip Pos','Y Tip Pos');
 xlabel('Time')
 ylabel('Position from nominal');
 xlim([0.1 2]);
 
 subplot(2,1,2)
 plot(time, KE, time, PE, time, KE+PE);
 legend('KE', 'PE', 'Total');
 xlabel('Time');
 ylabel('Energy');
 xlim([0.1 2]);

%DOUBLE PENDULUM
% actual = load('Double_Pendulum/DoublePend1.mat');
% 
% 
% angle1 = zeros(size(A,1),1);
% angle2 = zeros(size(A,1),1);
% 
% for i = 1:size(A,1)
% quat = [A(i,2),A(i,3),A(i,4),A(i,5)];
% [roll,pitch,yaw] = quat2angle(quat);
% angle1(i) = -mod(roll,2*pi);
% end
%  
% for i = 1:size(A,1)
% quat = [A(i,8),A(i,9),A(i,10),A(i,11)];
% [roll,pitch,yaw] = quat2angle(quat);
% angle2(i) = -mod(roll,2*pi);
% end
% 
% %Shift data start
% time = time-0.0;
% 
% %Line them up.
% angle1 = angle1-angle1(1)+interp1(X(:,1),X(:,2),time(1));
% angle2 = angle2-angle2(1)+interp1(X(:,1),X(:,4),time(1));
% 
% 
% %Find deviation more than 0.5
% for i = 1:size(time,1)
%     
%     if(abs((angle1(i)-X((find(X(:,1)<=time(i),1,'last')),2)))>0.5)
%         xbreak = time(i)
%         %X((find(X(:,1)<=time(i),1,'last')),2)
%         ybreak = angle1(i);
%         break;
%     end
% end
% 
% 
% subplot(2,1,1)
% hold on
% plot(time, angle1,X(:,1),X(:,2));
% plot(xbreak,ybreak,'*')
% hold off
% title('Link1');
% xlim([0 10]);
% 
% subplot(2,1,2)
% plot(time,angle2,X(:,1),X(:,4));
% title('Link2');
% xlim([0 10]);

% figure
% plot(time, A(:,6)+A(:,12)+A(:,7)+A(:,13));
% title('energy');
% xlim([0 10]);

