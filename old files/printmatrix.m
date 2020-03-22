
xreq = 0;
yreq = 10;
x_dotreq = 0.1;
y_dotreq = 0;

time_period2 = [0 2];
R0 = [ xreq, yreq, x_dotreq, y_dotreq];
[t2,q] = ode45('auv_Track_trajectory',time_period2,R0);


figure(1)
plot(q(:,1),q(:,2))

