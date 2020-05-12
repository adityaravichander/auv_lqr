% underactuated AUV -- Trajectory planning and Tracking control

% Initialise variables

    %constants
    m11 = 215;
    m22 = 265;
    m33 = 80;
    d11 = 70;
    d22 = 100;
    d33 = 50;    
    
    %initialise required values
    Reqd = zeros(8,30);               % [ xreq, yreq, x_dotreq, y_dotreq, psireq, ureq, vreq, rreq ]
    Reqd(:,1) = [ 0,10,5,0,0,0,0,0];     
    
    %initialise and print instantaneous values
    Inst = zeros(6,30);		      % [ x, y, psi, u, v, r ]
    Inst(:,1) = [ 0.5,10.5,0,0,0,0 ];    
    disp('instantaneous values');
    disp(Inst(:,1));
    Error = zeros(6,12);              % [ xe, ye, psie, ue, ve, re ]
    n=0;
    
% Control loop

for i = 1:90
    %i=1;   

    %% Initial values for state variables
    Error(1,i)   = Inst(1,i)-Reqd(1,i);                                          
    Error(2,i)   = Inst(2,i)-Reqd(2,i);                          
    XE = [ Error(1,i); Error(2,i)];
    R1 = [ cos(Inst(3,i)), -sin(Inst(3,i));
        sin(Inst(3,i)), cos(Inst(3,i))];
    UVE = -(transpose(R1)*XE); 
    Error(4,i) = UVE(1);                  
    Error(5,i) = UVE(2);                            
    Reqd(6,i) = Inst(4,i)-Error(4,i);
    Reqd(7,i) = Inst(5,i)-Error(5,i);    
    Reqd(5,i) = atan(Reqd(4,i)/Reqd(3,i)) - atan(Reqd(7,i)/Reqd(6,i));
    Error(3,i) = Inst(3,i)-Reqd(5,i);
    Error(6,i) = Error(3,i);                               
    Reqd(8,i) = Inst(6,i)-Error(6,i);

    % print required position and error
    %disp('error');
    %disp(Error(:,i+1));         
    disp('reqd');
    disp(Reqd(:,i));    

    % Control action by LQR
    time_period1 = [1 15];
    V0 = [ Error(1,i),Error(2,i),Error(3,i),Error(4,i),Error(5,i),Error(6,i),Reqd(6,i),Reqd(7,i),Reqd(8,i)]; 
    [t1,p] = ode45('Trackfn',time_period1,V0);     

    % plot error and update error
    figure(1)
    plot(p(:,1),p(:,2),'g');   % error-green     
    Error(:,i+1) = [ p(end,1),p(end,2),p(end,3),p(end,4),p(end,5),p(end,6)]; 

    % update and print instantaneous values
    Inst(:,i+1) = [ Error(1,i+1)+Reqd(1,i),Error(2,i+1)+Reqd(2,i),Error(3,i+1)+Reqd(5,i),Error(4,i+1)+Reqd(6,i),Error(5,i+1)+Reqd(7,i),Error(6,i+1)+Reqd(8,i) ];
    disp('instantaneous values');
    disp(Inst(:,i+1));    

    %% update required values
    time_period2 = [n n+0.15];
    V1 = [ Reqd(1,i), Reqd(2,i), Reqd(3,i), Reqd(4,i) ];
    [t2,q] = ode45('auv_Track_trajectory',time_period2,V1);    
    Reqd(1:4,i+1) = [ q(end,1),q(end,2),q(end,3),q(end,4)];
    n=n+0.15;    

    % plot instantaneous position and required trajectory together
    figure(2)
    plot(Inst(1,1:i+1),Inst(2,1:i+1),'b');  % instant position-blue
    hold on
    plot(Reqd(1,1:i+1),Reqd(2,1:i+1),'r');  % required position-red
    
end        
      
       
