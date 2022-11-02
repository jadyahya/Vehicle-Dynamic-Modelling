%% Response to steering input with no roll steer
V_x = 10:10:120; % in mph
m = 3000; % in lbm
C_af = 279.5*57.3; %lbf/rad
C_ar = 242*57.3; %lbf/rad
Iz = 40000; %lbm*ft^2 
Ixx = 15000; %lbm*ft^2 
lf = 3.5; %ft 
lr = 4.5; %ft
h1 = 0.5; %ft 
hcg = 1.2; %ft

V_x_SI =V_x*0.44704; %in m/s
m_SI = 3000*0.453592; %in kg
C_af_SI = C_af*4.448; %in N/rad
C_ar_SI =C_ar*4.448; %in N/rad
Iz_SI = Iz/23.73036; %in kg*m^2
Ixx_SI = Ixx/23.73036; %in kg*m^2
lf_SI =3.5*0.305; % in m
lr_SI =4.5*0.305; %in m
h1_SI = h1*0.305; %in m
hcg_SI = hcg*0.305; %in m
hr_SI=hcg_SI-h1_SI;
ms_SI = 2700*0.453592;
 


epsf = 0;
epsr = 0;
syms phi_delta beta_delta r_delta vx

A11 = -2*(C_af_SI+C_ar_SI)/m_SI/vx;
A12 = -(1+2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/m_SI/vx^2); 
A13 = 2*(C_ar_SI*epsr+C_af_SI*epsf)/m_SI/vx; 

A21 = -2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/Iz_SI; 
A22 = -2*(C_af_SI*lf_SI^2+C_ar_SI*lr_SI^2)/Iz_SI/vx; 
A23 = 2*(C_af_SI*epsf*lf_SI-C_ar_SI*epsr*lr_SI)/Iz_SI; 

A41 = -2*h1_SI*(C_ar_SI+C_af_SI)/(Ixx_SI+m_SI*(h1_SI^2)); 
A42 = -2*h1_SI*(-C_ar_SI*lr_SI+C_af_SI*lf_SI)/(Ixx_SI+m_SI*(h1_SI^2))/vx;
A43 = -(2*(C_ar_SI*epsr+C_af_SI*epsf)*hr_SI+m_SI*9.81*h1_SI)/(Ixx_SI+m_SI*(h1_SI^2)); 

A =[A11,A12,A13,0;A21,A22,A23,0;0,0,0,1;A41,A42,A43,0];
    
e1 = A11*beta_delta +A12*r_delta +2*C_af_SI/m_SI/vx ==0;
e2 = A21*beta_delta +A22*r_delta + 2*C_af_SI*lf_SI/Iz_SI == 0;
e3 = A41*beta_delta +A42*r_delta+A43*phi_delta+2*C_af_SI*h1_SI/(Ixx_SI+m_SI*h1_SI^2) == 0;


S = solve(e1,e2,e3,beta_delta,r_delta,phi_delta)

b_d = subs(S.beta_delta,vx,V_x_SI);
r_d = subs(S.r_delta,vx,V_x_SI);
p_d = subs(S.phi_delta,vx,V_x_SI);

figure(1)
plot(V_x,r_d)
title('r/delta vs speed')
grid on
xlabel('Speed in mph')
ylabel('Yaw rate over Steering angle')
hold on

%% Response to steering input with roll steer
epsf = 0;
epsr = -0.03;
syms phi_delta beta_delta r_delta vx

A11 = -2*(C_af_SI+C_ar_SI)/m_SI/vx; 
A12 = -(1+2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/m_SI/vx^2);
A13 = 2*(C_ar_SI*epsr+C_af_SI*epsf)/m_SI/vx; 

A21 = -2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/Iz_SI; 
A22 = -2*(C_af_SI*lf_SI^2+C_ar_SI*lr_SI^2)/Iz_SI/vx; 
A23 = 2*(C_af_SI*epsf*lf_SI-C_ar_SI*epsr*lr_SI)/Iz_SI; 

A41 = -2*h1_SI*(C_ar_SI+C_af_SI)/(Ixx_SI+m_SI*(h1_SI^2)); 
A42 = -2*h1_SI*(-C_ar_SI*lr_SI+C_af_SI*lf_SI)/(Ixx_SI+m_SI*(h1_SI^2))/vx; 
A43 = -(2*(C_ar_SI*epsr+C_af_SI*epsf)*h1_SI+m_SI*9.81*h1_SI)/(Ixx_SI+m_SI*(h1_SI^2)); 

A =[A11,A12,A13,0;A21,A22,A23,0;0,0,0,1;A41,A42,A43,0];
    
e1 = A11*beta_delta +A12*r_delta+A13*phi_delta +2*C_af_SI/m_SI/vx ==0;
e2 = A21*beta_delta +A22*r_delta+A23*phi_delta +2*C_af_SI*lf_SI/Iz_SI == 0;
e3 = A41*beta_delta +A42*r_delta+A43*phi_delta +2*C_af_SI*h1_SI/(Ixx_SI+m_SI*h1_SI^2) == 0;

S = solve(e1,e2,e3,beta_delta,r_delta,phi_delta);

b_d = subs(S.beta_delta,vx,V_x_SI);
r_d = subs(S.r_delta,vx,V_x_SI);
p_d = subs(S.phi_delta,vx,V_x_SI);

figure(1)
plot(V_x,r_d)
title('r/delta vs speed')
grid on
xlabel('Speed in mph')
ylabel('Yaw rate over Steering angle')
legend('No Roll','With Roll')
hold off

R= 400; %ft
R_SI = 400*0.305; % in m
epsf = 0;


% Derived parameters:
l_SI= lf_SI+lr_SI;
mr_SI = m_SI*lf_SI*l_SI;
mf_SI = m_SI*lr_SI*l_SI;
Kv = mf_SI/2/C_af_SI-mr_SI/2/C_ar_SI;
l=[];
for i = 1:4
    delta = (l_SI/R_SI+Kv*V_x_SI(i)^2/R_SI)*57.3;
    A1= double(subs(A,vx,V_x_SI(i)));
    B =[2*C_af_SI/m_SI/V_x_SI(i),2*lf_SI*C_af_SI/Iz_SI,0,2*h1_SI*C_af_SI/(Ixx_SI+m_SI*h1_SI^2)];
    C=eye(4);
    D=zeros(4,1);
   
    sys = ss(A1,B',C,D);
    [f,t,x]=step(delta*sys);
    l = [l,f(end,2)];
    hold on
end
figure(2)
plot(V_x(1:4),l/delta)
title('r/delta vs speed')
grid on
xlabel('Speed in mph')
ylabel('Yaw rate over Steering angle')


for i = 1:4
    delta = (l_SI/R_SI+Kv*V_x_SI(i)^2/R_SI);
    A1= double(subs(A,vx,V_x_SI(i)));
    B =[2*C_af_SI/m_SI/V_x_SI(i),2*lf_SI*C_af_SI/Iz_SI,0,2*hcg_SI*C_af_SI/(Ixx_SI+m_SI*h1_SI)];
    C=eye(4);
    D=zeros(4,1);
    figure(3)
    sys = ss(A1,B',C,D);
    step(delta*sys,100);
    hold on
end
legend('Speed = 10 mph', 'Speed = 20 mph','Speed = 30 mph', 'Speed = 40 mph')

syms y(t)
y(t) = piecewise((t>=0) & (t <= 0.0625),720*t,(t<=3.0625) & (t>=0.0625),45,(t>=3.0625) & (t<=3.1875),-720*(t-3.0625)+45,(t>=3.1875) & (t<=6.1875),-45,(t>=6.1875) & (t<=6.25),720*(t-6.1875)-45,(t<=9.25) & (t>=6.25),0);
xvalues = 0:0.01:9.25;
input_ackerman = double(y(xvalues))/15/57.3; %radians
for i =1:4
    A1= double(subs(A,vx,V_x_SI(i)));
    B =[2*C_af_SI/m_SI/V_x_SI(i),2*lf_SI*C_af_SI/Iz_SI,0,2*h1_SI*C_af_SI/(Ixx_SI+m_SI*h1_SI^2)];
    C=eye(4);
    D=zeros(4,1);
   
    sys = ss(A1,B',C,D);
    figure (4)
    title('Response to Steering Input')
    YSIM=lsim(sys, input_ackerman,xvalues); %radians
    plot(xvalues,YSIM(:,1))
    ylabel('side slip angle (rad)')
    xlabel('Time (s)')
    hold on
     figure(5)
     title('Response to Steering Input')
    plot(xvalues,YSIM(:,2))
    ylabel('yaw rate (rad)')
    xlabel('Time (s)')
    hold on
     figure(6)
          title('Response to Steering Input')
     ylabel('roll angle (rad)')
     xlabel('Time (s)')
    plot(xvalues,YSIM(:,3))
    hold on
    
end
grid off
legend ('Vx = 10 mph','Vx = 20 mph','Vx = 30 mph','Vx = 40 mph')