%Bullet Import test - double pend

% clear all
% clc
close all

A = csvread('Data.txt');
m = 1;
I = 1/3*(2^2+0.2^2)*m;

 time = cumsum(A(:,1));
 Ek = A(:,2);
 ang1 = A(:,3);
 ang2 = A(:,4);

 Ep1 = -sin(ang1)*1;
 Ep2 = -sin(ang1)*2 + -sin(ang2)*1;
 
 
 plot(Ep2+Ep1+Ek)
 figure(plot)
 
 
 
 