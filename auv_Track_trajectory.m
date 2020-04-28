%% Function for ODE solver of Requried trajectory
function dqdt = auv_Track_trajectory(t2,q)

dqdt = zeros(4,1); 
dqdt(1) = 0.1*cos(0.01*t2);     %to find xreq
dqdt(2) = -0.1*sin(0.01*t2);	%to find yreq
dqdt(3) = 0;                    %to find x_dotreq
dqdt(4) = 0;                    %to find y_dotreq
