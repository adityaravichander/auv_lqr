function dpdt = Trackfn(t,p)

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

dpdt = zeros(9,1);    

xe_mod = abs(x_e);
ye_mod = abs(y_e);
e=0.5;

if((xe_mod>=e)||(ye_mod>=e))
    X     = [ x_e; y_e; psi_e; u_e; v_e; r_e]; 

    A = [0, -rreq,0,                    -1,                      0,                 0;
        rreq,  0, 0,                      0,                     -1,                 0;  
        0,     0, 0,                      0,                      0,                -1;
        0,     0, 0,             (-d11/m11),       ((m22*rreq)/m11),  ((m22*vreq)/m11);               
        0,     0, 0,      ((-m11*rreq)/m22),             (-d22/m22), ((-m11*ureq)/m22);
        0,     0, 0, (((m11-m22)*vreq)/m33), (((m11-m22)*ureq)/m33),       (-d33/m33)];
    
    B = [   0,      0;
            0,      0;
            0,      0;
        1/m11,      0;
            0,      0;
            0,  1/m33];
    
    Q = [ 5,0,0,0,0,0;
        0,5,0,0,0,0;
        0,0,5,0,0,0;
        0,0,0,5,0,0;
        0,0,0,0,5,0;
        0,0,0,0,0,5];
    
    R = [ 1, 0;
        0, 1];
    
    % C = [ 1 1 1 1 1 1 ];
    % % disp('C');
    % % disp(C);
    
    % poles = eig(A);
    % % disp('poles');
    % % disp(poles);
    
    % rankc = rank(ctrb(A,B));
    % % disp('rankC');
    % % disp(rankc);
    
    % ranko = rank(obsv(A,C));
    % % disp('rankO');
    % % disp(ranko);
    printmatrix = [ x_e;
                        y_e;
                        psi_e;    
                        u_e;
                        v_e;
                        r_e;];
        disp(printmatrix);
    
    K = lqr(A,B,Q,R);
    F = -(K*X);     
    X_dot = A*X + B*F;      
    dpdt(1) = X_dot(1); 
    dpdt(2) = X_dot(2);      
    dpdt(3) = X_dot(3);
    dpdt(4) = X_dot(4);
    dpdt(5) = X_dot(5);  
    dpdt(6) = X_dot(6);
    dpdt(7) = 0;
    dpdt(8) = 0;
    dpdt(9) = 0;   

else
    dpdt(1) = 0; 
    dpdt(2) = 0;      
    dpdt(3) = 0;
    dpdt(4) = 0;
    dpdt(5) = 0;  
    dpdt(6) = 0;
    dpdt(7) = 0;
    dpdt(8) = 0;
    dpdt(9) = 0;
end








