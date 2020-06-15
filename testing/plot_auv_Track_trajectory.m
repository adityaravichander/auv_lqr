%% provisional

Reqd = zeros(4,10);        
Reqd(:,1) = [ 0,10,5,0]; 
n=0;
for i = 1:90
    % print and plot
    disp(i);
    disp('reqd');
    disp(Reqd(1:2,i));  
    %plot(q(:,1),q(:,2))
    
    % update
    time_period2 = [n n+0.15];
    V1 = [ Reqd(1,i), Reqd(2,i), Reqd(3,i), Reqd(4,i)];
    [t2,q] = ode45('auv_Track_trajectory',time_period2,V1);
    Reqd(:,i+1) = [q(end,1),q(end,2),q(end,3),q(end,4)];
    n=n+0.15;
end
    figure(1)
    plot(Reqd(1,1:i),Reqd(2,1:i));  


%% standard method
%     time_period2 = [0 0.15];
%     [t2,q] = ode45('auv_Track_trajectory',time_period2,[0;10;5;0]);
%     %plot(q(:,1),q(:,2))
%     disp('x');
%     disp(q(end,1));
%     disp('y');
%     disp(q(end,2));

