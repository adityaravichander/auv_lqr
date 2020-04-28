%% Trajectory planning and Tracking control of underactuated AUV

%% Initialise variables
    %constants
    m11 = 215;
    m22 = 265;
    m33 = 80;
    d11 = 70;
    d22 = 100;
    d33 = 50; 
    
    %initial instant vehicle values in body frame
    x = 15;
    y = 15;
    psi = 0;
    u = 0;
    v = 0;
    r = 0;   
    
    %required value and States of LQR
    xreq = 10;  
    yreq = 10;      
    x_dotreq = 0.1;
    y_dotreq = 0;
    Inst = zeros(6,18);
 
 %% State variables and Control action   
    for i=1:18
            
        % Updating values for state variables
        x_e   = x-xreq;                                                     % error in x coordinate of positon
        y_e   = y-yreq;                                                     % error in y coordinate of positon
        XE = [ x_e; y_e];
        R1 = [ cos(psi), -sin(psi);
            sin(psi), cos(psi)];
        UVE = -(transpose(R1)*XE); 
        u_e = UVE(1);                                                       % error in surge velocity 
        v_e = UVE(2);                                                       % error in sway velocity 
        ureq = u-u_e;
        vreq = v-v_e;    
        psireq = atan(y_dotreq/x_dotreq) - atan(vreq/ureq);
        psi_e = psi-psireq;
        r_e = psi_e;                                                        % error in yaw    
        rreq = r-r_e;
          
        
        % Control action by LQR
        time_period1 = [0 100];
        V0 = [ x_e, y_e, psi_e, u_e, v_e, r_e, ureq, vreq, rreq]; 
        [t1,p] = ode45('Trackfn',time_period1,V0); 
        
        
        % Updating instantaneous variables
        x   = p(end,1) + xreq;
        y   = p(end,2) + yreq;
        psi = p(end,3) + psireq;
        u   = p(end,4) + ureq;
        v   = p(end,5) + vreq;
        r   = p(end,6) + rreq;
        
      
        Inst(1,i) = x;
        Inst(2,i) = y;
        Inst(3,i) = psi;
        Inst(4,i) = u;
        Inst(5,i) = v;
        Inst(6,i) = r;
        
        % Displaying values of state variables
        disp('iteration value');
        disp(i);
        
       
        printmatrix = [ x_e;
                        y_e;
                        psi_e;    
                        u_e;
                        v_e;
                        r_e;];
        disp(printmatrix);
        
        
        % Updating the required values of variables 
%         time_period2 = [0 2];
%         R0 = [ xreq, yreq, x_dotreq, y_dotreq];
%         [t2,q] = ode45('auv_Track_trajectory',time_period2,R0);               
%         xreq     = q(end,1);
%         yreq     = q(end,2);
%         x_dotreq = q(end,3);
%         y_dotreq = q(end,4);
%         

    end 


        figure(1)
        plot(Inst(1,i),i); 

%% Plotting Results
	
   % Plotting graphs
   % figure(1)
   % plot(p(:,1),q(:,1));
   
    
        
    
      
    
