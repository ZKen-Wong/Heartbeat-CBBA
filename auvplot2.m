t = (0:h:(N-1)*h);                        % Calculate time values for plots.

figure(1)
clf
subplot(5,2,1)
plot(t,xout(:,1)),xlabel('time [s]'),ylabel('u, surge[m/s]'),grid
subplot(5,2,2)
plot(t,xout(:,2)),xlabel('time [s]'),ylabel('v, sway [m/s]'),grid
subplot(5,2,3)
plot(t,xout(:,3)),xlabel('time [s]'),ylabel('w, heave [m/s]'),grid
subplot(5,2,4)
plot(t,xout(:,4)*180/pi),xlabel('time [s]'),ylabel('p, roll [deg/s]'),grid
subplot(5,2,5)
plot(t,xout(:,5)*180/pi),xlabel('time [s]'),ylabel('q, pitch [deg/s]'),grid
subplot(5,2,6)
plot(t,xout(:,6)*180/pi),xlabel('time [s]'),ylabel('r, yaw [deg/s]'),grid
subplot(5,2,7)
plot(t,xout(:,10)*180/pi),xlabel('time [s]'),ylabel('roll, [deg]'),grid
subplot(5,2,8)
plot(t,xout(:,11)*180/pi),xlabel('time [s]'),ylabel('pitch [deg]'),grid
subplot(5,2,9)
plot(t,xout(:,12)*180/pi),xlabel('time [s]'),ylabel('yaw [degs]'),grid

figure(2)
clf
subplot(3,2,1)
plot(t,uout(:,1)*180/pi),xlabel('time [s]'),ylabel('dr [degs]'),grid
subplot(3,2,2)
plot(t,uout(:,2)*180/pi),xlabel('time [s]'),ylabel('ds [degs]'),grid
subplot(3,2,3)
plot(t,uout(:,3)*180/pi),xlabel('time [s]'),ylabel('db [degs]'),grid
subplot(3,2,4)
plot(t,uout(:,4)*180/pi),xlabel('time [s]'),ylabel('dbp [degs]'),grid
subplot(3,2,5)
plot(t,uout(:,5)*180/pi),xlabel('time [s]'),ylabel('dps [degs]'),grid
subplot(3,2,6)
plot(t,uout(:,6)),xlabel('time [s]'),ylabel('n [rpm]'),grid

figure(3)
clf
plot(xout(:,7),xout(:,8)),xlabel('xpos [m]'),ylabel('ypos [m]'),grid
