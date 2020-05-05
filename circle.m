% Trajectory planning and Tracking control of underactuated AUV
clear all;
clc;
clf;

%%Required Path - circle
C = [0 0];
cx = C(1);
cy = C(2);
R = 10;
%plot(C(1)+R*cos([1:360].*pi/180),C(2)+R*sin([1:360].*pi/180),'r'); hold on;
Reqd = zeros(6,30); % [xr,yr,psir,ur,vr,rr]

%% Initialise constants
    m11 = 215;
    m22 = 265;
    m33 = 80;
    d11 = 70;
    d22 = 100;
    d33 = 50;    

% AUV initialisation 
auv = zeros(6,10);
auv(:,1) = [1, 12, 0, 0, 0, 0]; % [x,y,psi,u,v,r]
x   = auv(1,1);
y   = auv(2,1);
psi = auv(3,1);
u   = auv(4,1);
v   = auv(5,1);
r   = auv(6,1);

disp('auv');
disp(auv(:,1));
%Error initialisation
Error = zeros(6,12);  % [ xe, ye, psie, ue, ve, re ]


% loop for updating 
%for i=1:
   i=1; 
    % Calculate required values and error
    disp(i);
    per_dist   = sqrt((x*x)+(y*y))-R;  % distance between AUV and path   
    theta = atan((y-cy)/(x-cx));

    Error(1,i) = per_dist*(cos(theta));        %xe
    Error(2,i) = per_dist*(sin(theta));        %ye
    Reqd(1,i)  = auv(1,i)   - Error(1,i);      %xr
    Reqd(2,i)  = auv(2,i)   - Error(2,i);      %yr
    Reqd(3,i)  = 1.57       + theta;           %psir
    Error(3,i) = auv(3,i)   - Reqd(3,i);       %psie
    XE         = [ Error(1,i); Error(2,i)];
    R1         = [ cos(auv(3,i)), -sin(auv(3,i));
                   sin(auv(3,i)), cos(auv(3,i))];
    UVE        = -(transpose(R1)*XE);
    Error(4,i) = UVE(1);       		       %ue	           
    Error(5,i) = UVE(2);		       %ve 
    Reqd(4,i)  = auv(4,i)   - Error(4,i);      %ur
    Reqd(5,i)  = auv(5,i)   - Error(5,i);      %vr

    if(i==1)
	Error(6,i) = Error(3,i);               
    else
	Error(6,i) = Error(3,i) - Error(3,i-1);%re
    end 
    Reqd(6,i)  = auv(6,i)   - Error(6,i);      %rr
    
    % Display
    disp('reqd');
    disp(Reqd(:,i));

    %Control action by LQR
    time_period1 = [0 20];
    V0 = [ Error(1,i),Error(2,i),Error(3,i),Error(4,i),Error(5,i),Error(6,i),Reqd(4,i),Reqd(5,i),Reqd(6,i),Reqd(1,i),Reqd(2,i),Reqd(3,i)]; 
    [t1,p] = ode45('circlefn',time_period1,V0);  
    %figure(1)
    %plot(p(:,1),p(:,2),'g');   % error-green 
    Error(:,i+1) = [ p(end,1),p(end,2),p(end,3),p(end,4),p(end,5),p(end,6)];     
    auv(:,i+1) = [ Error(1,i+1)+Reqd(1,i),Error(2,i+1)+Reqd(2,i),Error(3,i+1)+Reqd(3,i),Error(4,i+1)+Reqd(4,i),Error(5,i+1)+Reqd(5,i),Error(6,i+1)+Reqd(6,i) ];
    disp('auv');
    disp(auv(:,i+1)); 
       
    disp('error');
    disp(p(:,1:2));
     



