%% Function for ODE solver of Requried trajectory
function dqdt = auv_Track_trajectory(t2,q)

%disp('reqd');
%r = [q(1),q(2),q(3),q(4)];
%disp(r);

a=10;
b=0.5;

dqdt = zeros(4,1); 
dqdt(1) =    (a*b*cos(b*t2));   %to find xreq
dqdt(2) =   -(a*b*sin(b*t2));	%to find yreq
dqdt(3) = -(a*b*b*sin(b*t2));   %to find x_dotreq
dqdt(4) = -(a*b*b*cos(b*t2));   %to find y_dotreq

