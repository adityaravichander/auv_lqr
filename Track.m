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
    Reqd = zeros(8,30);               
    Reqd(1:8,1) = [ 10,10,0.1,0,0,0,0,0]; % [ xreq, yreq, x_dotreq, y_dotreq, psireq, ureq, vreq, rreq ]    
    
    %initial instant vehicle values    
    Inst = zeros(6,30);
    Inst(1:6,1) = [ 15,15,0,0,0,0 ];  	  % [ x, y, psi, u, v, r ]
    m = zeros(1,30);
    m(1,1) = 1;
    
%% main LOOP    

for i = 1:10
    
    % values for state variables
    Error = zeros(6,30);                  % [ xe, ye, psie, ue, ve, re ]
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
    
    %print reqd values
    disp('reqd');
    disp(Reqd(1:8,i));
    
    % Control action by LQR
    time_period1 = [1 1000];
    V0 = [ Error(1,i),Error(2,i),Error(3,i),Error(4,i),Error(5,i),Error(6,i),Reqd(6,i),Reqd(7,i),Reqd(8,i)]; 
    [t1,p] = ode45('Trackfn',time_period1,V0);        

    %print error and instantaneous values
    Error(1:6,i+1) = [ p(end,1),p(end,2),p(end,3),p(end,4),p(end,5),p(end,6)];     
    Inst(1:6,i+1) = [ Error(1,i+1)+Reqd(1,i),Error(2,i+1)+Reqd(2,i),Error(3,i+1)+Reqd(5,i),Error(4,i+1)+Reqd(6,i),Error(5,i+1)+Reqd(7,i),Error(6,i+1)+Reqd(8,i) ];
    m(1,i) = i;
    disp('error');
    disp(Error(1:6,i+1));
    disp('instant position');
    disp(Inst(1:6,i+1));
    
    % update required values
    time_period2 = [1 10];
    V1 = [ Reqd(1,i), Reqd(2,i), Reqd(3,i), Reqd(4,i) ];
    [t2,q] = ode45('auv_Track_trajectory',time_period2,V1);
    Reqd(1:4,i+1) = [ q(end,1),q(end,2),q(end,3),q(end,4)];

end        

%%    plotting results

	%plot instantaneous values
       figure(1)
       plot(Inst(1,1:i),Inst(2,1:i));  

	%plot required trajectory
       figure(2)
       plot(Reqd(1,1:i),Reqd(2,1:i));
       
