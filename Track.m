%% Trajectory planning and Tracking control of underactuated AUV

%% Initialise variables
    %constants
    m11 = 215;
    m22 = 265;
    m33 = 80;
    d11 = 70;
    d22 = 100;
    d33 = 50;    
    
    %initial required values
    xreq = 10;  
    yreq = 10;      
    x_dotreq = 0.1;
    y_dotreq = 0;
    
    %initial instant vehicle values    
    Inst = zeros(6,20);
    Inst(1:6,1) = [ 15,15,0,0,0,0 ];  % [ x, y, psi, u, v, r ]
    m = zeros(1,21);
    m(1,1) = 1;
  
    % Initial values for state variables
    Error = zeros(6,20);  % [ xe, ye, psie, ue, ve, re ]
    Error(1,1)   = Inst(1,1)-xreq;                                          
    Error(2,1)   = Inst(2,1)-yreq;                          
    XE = [ Error(1,1); Error(2,1)];
    R1 = [ cos(Inst(3,1)), -sin(Inst(3,1));
        sin(Inst(3,1)), cos(Inst(3,1))];
    UVE = -(transpose(R1)*XE); 
    Error(4,1) = UVE(1);                  
    Error(5,1) = UVE(2);                            
    ureq = Inst(4,1)-Error(4,1);
    vreq = Inst(5,1)-Error(5,1);    
    psireq = atan(y_dotreq/x_dotreq) - atan(vreq/ureq);
    Error(3,1) = Inst(3,1)-psireq;
    Error(6,1) = Error(3,1);                               
    rreq = Inst(6,1)-Error(6,1);   
     
 %% State variables and Control action   
 
        i=2;
        % Control action by LQR
        time_period1 = [1 500];
        n = 1;
        V0 = [ Error(1,i-1),Error(2,i-1),Error(3,i-1),Error(4,i-1),Error(5,i-1),Error(6,i-1),ureq,vreq,rreq,n]; 
        [t1,p] = ode45('Trackfn',time_period1,V0);        
        
        Error(1:6,i) = [ p(end,1),p(end,2),p(end,3),p(end,4),p(end,5),p(end,6)];        

        Inst(1:6,i) = [ Error(1,i)+xreq,Error(2,i)+yreq,Error(3,i)+psireq,Error(4,i)+ureq,Error(5,i)+vreq,Error(6,i)+rreq ];
        m(1,i) = i;
        
        % Displaying values 
       % disp('iteration value');
       % disp(i);
        %disp(Error(1:6,i));

    %% PLOTTING RESULTS
%       figure(1)
%       plot(Inst(1,1:i),Inst(2,1:i)); 
%               figure(2)
%         plot(m(1,1:i),Error(1,1:i));
       
%         figure(3)
%         plot(tm,p(tm,2));

        
        % Updating the required values of variables 
%         time_period2 = [0 2];
%         R0 = [ xreq, yreq, x_dotreq, y_dotreq];
%         [t2,q] = ode45('auv_Track_trajectory',time_period2,R0);               
%         xreq     = q(end,1);
%         yreq     = q(end,2);
%         x_dotreq = q(end,3);
%         y_dotreq = q(end,4);
%         

