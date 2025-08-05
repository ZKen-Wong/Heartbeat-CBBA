% auvinit.m   datafile for auv.m. Contains initial state 
%              vector, initial input and main dimensions.
%
% NTH 1994 Trygve Lauvdal

% Default parameters for the Underwater Vehicle :
%
% Max rudder angle (all)   : 20 deg
% Max shaft velocity       : 1500 rpm
% Length of vehicle        : 5.3 m

% Initial states,inputs and sample time (to be edited by user):

u     = 1.9;           % surge velocity         (m/s)
v     = 0;           % sway velocity          (m/s)
w     = 0;           % heave velocity         (m/s)
p     = 0;           % roll velocity          (rad/s)
q     = 0;           % pitch velocity         (rad/s)
r     = 0;           % yaw velocity           (rad/s)
phi   = 0;           % roll angle             (rad)
theta = 0;           % pitch angle            (rad)
psi   = 0*pi/180;           % yaw angle              (rad)
xpos  = 0;           % surge position         (m)
ypos  = 0;           % sway position          (m)
zpos  = 0;           % heave position         (m)

delta_r  = 0;        % rudder angle                   (rad)
delta_s  = 0;        % port and starboard stern plane (rad)
delta_b  = 0;        % top and bottom bow plane       (rad)
delta_bp = 0;        % port bow plane                 (rad)
delta_bs = 0;        % starboard bow plane            (rad)
n        = 1500;        % propeller shaft speed          (rpm)  

x     = [ u v w p q r xpos ypos zpos phi theta psi]';     % state vector
ui     = [ delta_r delta_s delta_b delta_bp delta_bs n ]'; % input vector

