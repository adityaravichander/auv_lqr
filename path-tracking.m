% Trajectory planning and Tracking control of underactuated AUV

% AUV values 
auv = zeros(6, 100);   % [ x, y, psi, u, v, r ] Auv values in global frame
auv(:, 1) = [11, 5, 0, 0, 0, 0];
disp('Auv');
disp(auv(:, 1));

% Path values
cx = 0; % x coordinate of center of circle
cy = 0; % y coordinate of center of circle
R = 10; % Radius of circle

% Global values 
global Fin
Fin = [0, 0];
global Reqd 
Reqd = zeros(6, 200);  % [ x_req, y_req, psi_req, u_req, v_req, r_req ] Required values in global frame
global Error
Error = zeros(6, 200); % body frame [ x_error, y_error, psi_error, u_error, v_error, r_error ] 
global i
xyGE = zeros(2, 200);  % global frame [ x_error, y_error]

%% Path Tracking

for i = 1:200

    % Calculate perpendicular distance to circle
    disp(i);
    per_dist = (sqrt((auv(1,i)^2) + (auv(2,i)^2))) - R;  % shortest distance between AUV and path
    disp('per_dist');
    disp(per_dist);
    
    % Theta - angle between auv and center of circle 
    theta = angwrapfn(atan2((auv(2,i) - cy),(auv(1,i) - cx)));  % psi_required in range ( 0 to 6.28 )        
    disp('theta')
    disp(theta);
    
    % PSI Error and Reqd values 
    Reqd(3,i) = angwrapfn(1.57 + theta);   % psi_required - direction of tangent to circle
    Error(3,i) = auv(3,i) - Reqd(3,i);     % psi_error
    
    % Position Error and Reqd values 
    xyGE(1:2,i) = [ per_dist * (cos(theta)); per_dist * (sin(theta)) ];  % position error global frame
    Reqd(1,i) = auv(1,i) - xyGE(1);        % x_required
    Reqd(2,i) = auv(2,i) - xyGE(2);        % y_required
    R1 = [ cos(auv(3,i)), sin(auv(3,i));   % Rotation matrix for global to body frame
          -sin(auv(3,i)), cos(auv(3,i))];
    Error(1:2,i) = (R1 * xyGE(1:2,i));     % position error in body frame

    % Velocity Error and Reqd Values 
    ts = 0.01;                              % sample time
    if (i==1)
        xydot_GE = xyGE(1:2,i)/ts;         % velocity error in global frame
        Error(6,i) = Error(3,i)/ts;        % r_error
    else
        xydot_GE = [((xyGE(1,i) - xyGE(1,i-1))/ts);  ((xyGE(2,i) - xyGE(2,i-1))/ts)]; % velocity error in global frame
        Error(6,i) = ((Error(3,i) - Error(3,i-1))/ts);   % r_error
    end 	
    Error(4:5,i) = (R1 * xydot_GE);	   % velocity error in body frame 
    Reqd(4,i) = auv(4,i) - Error(4,i);     % u_req
    Reqd(5,i) = auv(5,i) - Error(5,i);     % v_req
    Reqd(6,i) = auv(6,i) - Error(6,i);     % r_req
    
    % Display
    disp('reqd');
    disp(Reqd(1:6,i));
    %disp('errorpos');
    %disp(Error(1:6,i));
    
    % Control action by LQR
    time_period = [0 0.01];
    opts = odeset('RelTol',1e-2,'AbsTol',1e-5);
    V0 = auv(:,i);
    [t1, p] = ode45('Trackfn', time_period, V0, opts);
    
    % Plots
    figure(1)
    plot(p(:,1), p(:,2), 'b')   % plots auv's path
    hold on
    plot(cx + R * cos([1:360] .* pi / 180), cy + R * sin([1:360] .* pi / 180), 'r'); % plots required path

    % Update auv parameters
    auv(:,i+1) = p(end,:);             
    auv(3,i+1) = angwrapfn(auv(3,i+1)); % psi in ( 0 to 6.28 )

    % Display 
    disp('Fin');
    disp(Fin);
    disp('Auv');
    disp(auv(1:6,i+1));
    
end


