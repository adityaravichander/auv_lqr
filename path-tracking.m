% Trajectory planning and Tracking control of underactuated AUV
clear all;
clc;
clf;

% AUV parameters
auv = zeros(6, 100);   % [ x, y, psi, u, v, r ]
auv(:, 1) = [11, 5, 0, 0, 0, 0];
disp('auv');
disp(auv(:, 1));

%%Required Path - circle
cx = 0;
cy = 0;
R = 10;
Reqd = zeros(6, 100);  % [ x_req, y_req, psi_req, u_req, v_req, r_req ]
Fin = [0,0];
%Error parameters
Error = zeros(6, 100); % [ x_error, y_error, psi_error, u_error, v_error, r_error ]

%    i=1;

for i = 1:50

    % Calculate Required values
    disp(i);
    per_dist = sqrt((auv(1, i)^2) + (auv(2, i)^2)) - R; % perpendicular distance between AUV and path
    theta = atan((auv(2, i) - cy) / (auv(1, i) - cx));  % angle between point and center of circle

    Error(1, i) = per_dist * (cos(theta));              % x_error
    Error(2, i) = per_dist * (sin(theta));              % y_error
    Reqd(1, i) = auv(1, i) - Error(1, i);               % x_req
    Reqd(2, i) = auv(2, i) - Error(2, i);               % y_req
    Reqd(3, i) = 1.57 + theta;                          % psi_req
    Error(3, i) = auv(3, i) - Reqd(3, i);               % psi_error
    XYE = [Error(1, i); Error(2, i)];                   % array of x_error and y_error
    R1 = [cos(auv(3, i)), -sin(auv(3, i));              % Rotation matrix
          sin(auv(3, i)), cos(auv(3, i))];
    UVE = -(transpose(R1) * XYE);                       % array of u_error and v_error
    Error(4, i) = UVE(1);                               % u_error 
    Error(5, i) = UVE(2);                               % v_error
    Reqd(4, i) = auv(4, i) - Error(4, i);               % u_req
    Reqd(5, i) = auv(5, i) - Error(5, i);               % v_req

    if (i == 1)
        Error(6, i) = Error(3, i);      % + > top
    else
        Error(6, i) = (Error(3, i) - Error(3, i - 1)); % r_error  -> circle
    end
    Reqd(6, i) = auv(6, i) - Error(6, i);               % r_req
    
    % Display reqd position and position error
    disp('reqdpos');
    disp(Reqd(1:2,i));
    disp('errorpos');
    disp(Error(1:2,i));
    
    % Control action by LQR
    time_period = [0 1];
    V0 = [auv(1, i), auv(2, i), auv(3, i), auv(4, i), auv(5, i), auv(6, i), Reqd(4, i), Reqd(5, i), Reqd(6, i), Reqd(1, i), Reqd(2, i), Reqd(3, i), Fin(1), Fin(2)];
    [t1, p] = ode45('Trackfn', time_period, V0);
    
    % Plot auv's path and required path
    figure(1)
    plot(p(:, 1), p(:, 2), 'b')                         % plots auv's path
    hold on
    plot(cx + R * cos([1:360] .* pi / 180), cy + R * sin([1:360] .* pi / 180), 'r'); % plots required path

    % Update auv parameters
    auv(:, i + 1) = [p(end, 1), p(end, 2), p(end, 3), p(end, 4), p(end, 5), p(end, 6)];

    % Display auv position
    disp('auvpos');
    disp(auv(1:2,i+1));
end


