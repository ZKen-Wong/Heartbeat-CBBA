% auvsim.m
%
% Main program for AUV simulation
%
% GU Euan McGookin 2000
clear                                        % clear all variables

auvinit													 % set model data

N     = 2000;                                % number of samples
h     = 0.05;        % sample time            (s)

xout(1,:) = x'; 			                 % data storage
uout(1,:) = ui'; 			                 % data storage

i=0;

for t = h:h:N*h                              % START OF MAIN LOOP
   
  i=i+1;
   
  xout(i,:) = x'; 			                 % state data storage

  uout(i,:) = ui'; 			                 % input data storage
 
  ui     = [ 0*pi/180 0*pi/180 0*pi/180 0*pi/180 0*pi/180 n ]'; % input vector

  xdot = auv(x,ui);                          % xdot(i) = f(x(i),u(i))

  x    = x + h*xdot ;                        % euler integration x(i+1)
  
end                                          % END OF MAIN LOOP   

auvplot2					                       % plots simulation


