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

% AUV parameters
x   = p(1);
y   = p(2);
psi = p(3);
u   = p(4);
v   = p(5);
r   = p(6);

% Required parameters
ureq   = p(7);
vreq   = p(8);
rreq   = p(9);   
xreq   = p(10);
yreq   = p(11);
psireq = p(12);

dpdt = zeros(12,1);    

X = [ (x-xreq); (y-yreq); (psi-psireq); (u-ureq); (v-vreq); (r-rreq)]; % Errors - states of LQR
xe_mod = abs(X(1));
ye_mod = abs(X(2));
e=0.01;



if((xe_mod>=e)||(ye_mod>=e))
    A = [0, -rreq,0,                      1,                      0,                 0;
        rreq,  0, 0,                      0,                      1,                 0;  
        0,     0, 0,                      0,                      0,                 1;
        0,     0, 0,             (-d11/m11),       ((m22*rreq)/m11),  ((m22*vreq)/m11);               
        0,     0, 0,      ((-m11*rreq)/m22),             (-d22/m22), ((-m11*ureq)/m22);
        0,     0, 0, (((m11-m22)*vreq)/m33), (((m11-m22)*ureq)/m33),       (-d33/m33)];
    
    B = [   0,      0;
            0,      0;
            0,      0;
        1/m11,      0;
            0,      0;
            0,  1/m33];

    q = 1;
    Q =[q,0,0,0,0,0;
        0,q,0,0,0,0;
        0,0,q,0,0,0;
        0,0,0,q,0,0;
        0,0,0,0,q,0;
        0,0,0,0,0,q];
    
    R = [1,0;
         0,1];

    K = lqr(A,B,Q,R);
    F = -(K*X);  
    
    p(13) = p(13)-F(1);
    p(14) = p(14)-F(2);
    dpdt(13)= p(13);
    dpdt(14)= p(14);
else 
    dpdt(13)= p(13);
    dpdt(14)= p(14);
end


% ODE solver 
dpdt(4) = ((m22*v*r)/m11) - ((-d11*u)/m11) + (p(13)/m11);
dpdt(5) = -((m11*u*r)/m22) - ((-d22*v)/m22);  
dpdt(6) = (((m11-m22)*u*v)/m33) - ((-d33*r)/m33) + (p(14)/m33);       
dpdt(3) = (r);
dpdt(1) = (u*cos(psi)) - (v*sin(psi)); 
dpdt(2) = (u*sin(psi)) + (v*cos(psi));      


dpdt(7) = 0;
dpdt(8) = 0;
dpdt(9) = 0;
dpdt(10)= 0;
dpdt(11)= 0;
dpdt(12)= 0;


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

