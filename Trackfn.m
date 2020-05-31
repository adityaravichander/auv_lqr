function dpdt = Trackfn(t,p)
    
    % Constants
    mass= 185; % Mass kg
    Iz = 50; % Rotational inertia kg-m^2
    Xu = -30; % added mass kg
    Yv = -90; % added mass kg
    Nr = -30; % added mass kg
    d11 = 70; % surge linear drag
    d22 = 100;% sway linear drag
    d33 = 50; % yaw linear drag
    
    % combined inertia and added mass terms
    m11 = mass - Xu; %kg 
    m22 = mass - Yv; %kg
    m33 = Iz- Nr;    %kg-m^2
    
    % Global variables
    global Fin
    global i
    global Reqd
    global Error
    
    dpdt = zeros(6,1);   
    
    A = [       0, -Reqd(6,i),  0,                           1,                           0,                      0;
        Reqd(6,i),           0, 0,                           0,                           1,                      0;  
                0,           0, 0,                           0,                           0,                      1;
                0,           0, 0,                  (-d11/m11),       ((m22*Reqd(6,i))/m11),  ((m22*Reqd(5,i))/m11);               
                0,           0, 0,      ((-m11*Reqd(6,i))/m22),                  (-d22/m22), ((-m11*Reqd(4,i))/m22);
                0,           0, 0, (((m11-m22)*Reqd(5,i))/m33), (((m11-m22)*Reqd(4,i))/m33),            (-d33/m33)];
        
    B = [   0,      0;
            0,      0;
            0,      0;
        1/m11,      0;
            0,      0;
            0,  1/m33];
    
    % Tuning matrices    
    Q = 1*eye(6);
    R = 1*eye(2);
    
    % LQR gain and Control Input
    K = lqr(A,B,Q,R);    % gain from LQR
    F = -(K*Error(:,i)); % returns error in Fin 
        
    % Updating Fin values
    Fin(1) = Fin(1)-F(1);
    Fin(2) = Fin(2)-F(2);
    
    % ODE solver 
    dpdt(4) = ((m22*p(5)*p(6))/m11) - ((-d11*p(4))/m11) + (Fin(1)/m11);       % to find u
    dpdt(5) = -((m11*p(4)*p(6))/m22) - ((-d22*p(5))/m22);                     % to find v
    dpdt(6) = (((m11-m22)*p(4)*p(5))/m33) - ((-d33*p(6))/m33) + (Fin(2)/m33); % to find r
    dpdt(3) = (p(6));                                                         % to find psi
    dpdt(1) = (p(4)*cos(p(3))) - (p(5)*sin(p(3)));                            % to find x
    dpdt(2) = (p(4)*sin(p(3))) + (p(5)*cos(p(3)));                            % to find y
    
    %% Controllability and Observability tests  
    
        %  C = [ 1 1 1 1 1 1 ];
        %  disp('C');
        %  disp(C);
        
        %  poles = eig(A);
        %  disp('poles');
        %  disp(poles);
        
        %  rankc = rank(ctrb(A,B));
        %  disp('rankC');
        %  disp(rankc);
        
        %  ranko = rank(obsv(A,C));
        %  disp('rankO');
        %  disp(ranko);
        %  printmatrix = [ x_e;
        %                  y_e;
        %                  psi_e;    
        %                  u_e;
        %                  v_e;
        %                  r_e;];
        %  disp(printmatrix);    
        