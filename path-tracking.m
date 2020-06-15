% Trajectory planning and Tracking control of underactuated AUV

% Constants
global m
global d
mass= 185;                    % Mass kg
Iz = 50;                      % Rotational inertia kg-m^2
Xu = -30;                     % added mass kg
Yv = -90;                     % added mass kg
Nr = -30;                     % added mass kg
m = [mass-Xu, mass-Yv, Iz-Nr] % combined inertia and added mass terms
d = [70, 100, 50];            % linear drag [surge, sway, yaw]

% Path values
cx = 0; % x coordinate of center of circle
cy = 0; % y coordinate of center of circle
R = 10; % Radius of circle
Reqd = zeros(6, 200);  % [ x_req, y_req, psi_req, u_req, v_req, r_req ] Required values in global frame

% AUV values 
auv = zeros(6, 100);   % [ x, y, psi, u, v, r ] Auv values in global frame
auv(:, 1) = [10.5, 5, 0, 0, 0, 0];
disp('Auv');
disp(auv(:, 1)); 
global Fin             
Fin = [0, 0];          % Force input
Error = zeros(6, 200); % Error values [ x_error, y_error, psi_error, u_error, v_error, r_error ] 

%% Path Tracking

for i = 1:200
    disp(i);

    % Calculate perpendicular distance to circle
    per_dist = ((sqrt((auv(1,i)^2) + (auv(2,i)^2))) - R);  % shortest distance between AUV and path
    disp('per_dist');
    disp(per_dist);
    
    % Theta - angle between auv and center of circle 
    theta = angwrapfn(atan2((auv(2,i) - cy),(auv(1,i) - cx)));  % psi_required in range ( 0 to 6.28 )        
    disp('theta');
    disp(theta);
    
    % PSI Error and Reqd values 
    Reqd(3,i) = angwrapfn(1.57 + theta);    % psi_required - direction of tangent to circle
    Error(3,i) = auv(3,i) - Reqd(3,i);      % psi_error
    
    % Position Error and Reqd values 
    Error(1:2,i) = [ per_dist*(cos(theta)); per_dist*(sin(theta)) ];  % position error
    Reqd(1,i) = auv(1,i) - Error(1);        % x_required
    Reqd(2,i) = auv(2,i) - Error(2);        % y_required

    % Velocity Error and Reqd Values 
    ts = 0.01;                              % sample time
    if (i==1)
        xydot_GE = Error(1:2,i)/ts;         % velocity error in global frame
        Error(6,i) = Error(3,i)/ts;         % r_error
    else
        xydot_GE = [((Error(1,i) - Error(1,i-1))/ts);  ((Error(2,i) - Error(2,i-1))/ts)]; % velocity error in global frame
        Error(6,i) = ((Error(3,i) - Error(3,i-1))/ts); % r_error
    end 	

    % Rotation matrix 1
    R1 = [ cos(Reqd(3,i)),  sin(Reqd(3,i)); 
           sin(Reqd(3,i)), -cos(Reqd(3,i))];
    % Rotation matrix 2   
    R2 = [-cos(Reqd(3,i)) - (sin(Reqd(3,i))*Error(3,i)) , -sin(Reqd(3,i)) + (cos(Reqd(3,i))*Error(3,i)) ; 
          -sin(Reqd(3,i)) + (cos(Reqd(3,i))*Error(3,i)) ,  cos(Reqd(3,i)) + (sin(Reqd(3,i))*Error(3,i))];
       
    Reqd(4:5,i) = (R2)\(xydot_GE - (R1*auv(3:4,i))); % ureq and vreq 
    Error(4:5,i) = auv(4:5,i) - Reqd(4:5,i);         % u_error and v_error   
    Reqd(6,i) = auv(6,i) - Error(6,i);               % r_req
    
    % Display
    disp('reqd');
    disp(Reqd(1:6,i));
    %disp('errorpos');
    %disp(Error(1:6,i));
    
    % State model
    A = [                    (-d(1)/m(1)),       ((m(2)*Reqd(6,i))/m(1)),  ((m(2)*Reqd(5,i))/m(1));               
                 ((-m(1)*Reqd(6,i))/m(2)),                  (-d(2)/m(2)), ((-m(1)*Reqd(4,i))/m(2));
            (((m(1)-m(2))*Reqd(5,i))/m(3)), (((m(1)-m(2))*Reqd(4,i))/m(3)),          (-d(3)/m(3))];
        
    B = [ 1/m(1),       0;
              0,        0;
              0,  1/m(3)];
    
    % Tuning matrices    
    Q = 1*eye(3);
    Rt = 1*eye(2);
    
    % Control action by LQR
    K = lqr(A,B,Q,Rt);     % gain from LQR
    F = -(K*Error(4:6,i)); % returns error in Fin     
    Fin = Fin-F;           % Updating Fin values
    
    % ODE solver to update auv values
    time_period = [0 ts];                           % time period for ODE45
    V0 = auv(:,i);                                  % input to ODE45
    [t1, p] = ode45('Trackfn', time_period, V0);    % ODE45 
    auv(:,i+1) = p(end,:);                          % Update auv parameters            
    auv(3,i+1) = angwrapfn(auv(3,i+1));             % psi in ( 0 to 6.28 )

    % Plots
    figure(1)
    plot(p(:,1), p(:,2), 'b')   % plots auv's path
    hold on
    plot(cx + (R * cos([1:360] .* pi / 180)), cy + (R * sin([1:360] .* pi / 180)), 'r'); % plots required path

    % Display 
    disp('Fin');
    disp(Fin);
    disp('Auv');
    disp(auv(1:6,i+1));
    
end



