function dpdt = auv_Trackfn(t,p)

m11 = 215;
m22 = 265;
m33 = 80;
d11 = 70;
d22 = 100;
d33 = 50; 

x_e   = p(1);
y_e   = p(2);
psi_e = p(3);
u_e   = p(4);
v_e   = p(5);
r_e   = p(6);

ureq = p(7);
vreq = p(8);
rreq = p(9);

X     = [ x_e; y_e; psi_e; u_e; v_e; r_e]; 

A = [    0, -rreq, 0,                              1,                        0,              -y_e;
      rreq,     0, 0,                              0,                        1,               x_e;  
         0,     0, 0,                              0,                        0,                 1;
         0,     0, 0,                     (-d11/m11), ((m22*(r_e + rreq))/m11),  ((m22*vreq)/m11);               
         0,     0, 0,      ((-m11*(r_e + rreq))/m22),               (-d22/m22), ((-m11*ureq)/m22);
         0,     0, 0, (((m11-m22)*(v_e + vreq))/m33),   (((m11-m22)*ureq)/m33),       (-d33/m33)];
  
B = [    0,      0;
         0,      0;
         0,      1;
     1/m11, -1/m11;
         0,      0;
     1/m33, -1/m33];

Q = [ 1.5,   0,   0,   0,   0,   0;
        0, 1.5,   0,   0,   0,   0;
        0,   0, 1.5,   0,   0,   0;
        0,   0,   0, 1.5,   0,   0;
        0,   0,   0,   0, 1.5,   0;
        0,   0,   0,   0,   0, 1.5];
     
R = [ 1, 0;
      0, 1];
  
dpdt = zeros(9,1); 
e = 0.5;

distance = ((p(1)*p(1))+(p(2)*p(2)))^(0.5);

if(distance>e)
    K = lqr(A,B,Q,R);
    F = -(K*X);     
    X_dot = A*X + B*F;      
    dpdt(1) = X_dot(1); 
    dpdt(2) = X_dot(2);      
    dpdt(3) = X_dot(3);
    dpdt(4) = X_dot(4);
    dpdt(5) = X_dot(5);  
    dpdt(6) = X_dot(6);  

end

dpdt(7) = 0;
dpdt(8) = 0;
dpdt(9) = 0;















