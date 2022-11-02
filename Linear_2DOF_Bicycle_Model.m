%% Slip Angle/steering angle & yaw rate/steering angle as a function of speed
V_x = 10:10:120; % in mph
m = 3000; % in lbm
C_af = 279.5*57.3; %lbf/rad
C_ar = 242*57.3; %lbf/rad
Iz = 40000; %lbm*ft^2 
lf = 3.5; %ft 
lr = 4.5; %ft

V_x_SI =V_x*0.44704; %in m/s
m_SI = 3000*0.453592; %in kg
C_af_SI = C_af*4.448; %in N/rad
C_ar_SI =C_ar*4.448; %in N/rad
Iz_SI = Iz/23.73036; %in kg*m^2
lf_SI =3.5*0.305; % in m
lr_SI =4.5*0.305; %in m

syms beta_delta r_delta vx
e1 = 2*(C_af_SI+C_ar_SI)/(m_SI*vx)*beta_delta +(2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/(m_SI*vx^2)+1)*r_delta -2*C_af_SI/m_SI/vx ==0;
e2 = 2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/Iz_SI*beta_delta +2*(C_af_SI*lf_SI^2+C_ar_SI*lr_SI^2)/Iz_SI/vx*r_delta - 2*C_af_SI*lf_SI/Iz_SI == 0;

S = solve(e1,e2,beta_delta,r_delta);

b_d = subs(S.beta_delta,vx,V_x_SI);
r_d = subs(S.r_delta,vx,V_x_SI);

figure(1)
plot(V_x,b_d)
title('beta/delta vs speed')
grid on
xlabel('Speed in mph')
ylabel('Slip angle over Steering angle')

figure(2)
plot(V_x,r_d)
title('r/delta vs speed')
grid on
xlabel('Speed in mph')
ylabel('Yaw rate over Steering angle')

%% Response to driver inputs
syms y(t)
y(t) = piecewise((t>=0) & (t <= 0.0625),720*t,(t<=3.0625) & (t>=0.0625),45,(t>=3.0625) & (t<=3.1875),-720*(t-3.0625)+45,(t>=3.1875) & (t<=6.1875),-45,(t>=6.1875) & (t<=6.25),720*(t-6.1875)-45,(t<=9.25) & (t>=6.25),0)
fplot(y,[0,9.25])
hold on
grid on 
fplot(y/15,[0,9.25])
xlabel('Time (sec)')
ylabel('Steering angle(degree)')
title('Steering angles vs time')
legend('Handwheel Angle','Ackerman Angle')

%% Response to steering input at different speeds
xvalues = 0:0.01:9.25;
input_ackerman = double(y(xvalues))/15/57.3; %radians
for i =1:length(V_x_SI)
    A = [ -2*(C_af_SI+C_ar_SI)/(m_SI*V_x_SI(i)), -(2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/(m_SI*V_x_SI(i)^2)+1); -2*(C_af_SI*lf_SI-C_ar_SI*lr_SI)/Iz_SI, -2*(C_af_SI*lf_SI^2+C_ar_SI*lr_SI^2)/(Iz_SI*V_x_SI(i))];
    B = [2*C_af_SI/m_SI/V_x_SI(i); 2*C_af_SI*lf_SI/Iz_SI];
    sys = ss(A,B, eye(2), zeros(2,1));
    lsim(sys, input_ackerman,xvalues) %radians
    hold on
end
grid on
legend ('Vx = 10 mph','Vx = 20 mph','Vx = 30 mph','Vx = 40 mph','Vx = 50 mph','Vx = 60 mph','Vx = 70 mph','Vx = 80 mph','Vx = 90 mph','Vx = 100 mph','Vx = 110 mph','Vx = 120 mph')