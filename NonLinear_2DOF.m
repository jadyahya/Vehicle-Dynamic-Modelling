%% Response to steering angle with load shift between inner and outer wheels
V_x = 10:10:120; % in mph
m = 3000; % in lbm
C_af = [279.54,279.51,279.41,279.14,278.55,277.50,275.75,273.08,269.19,263.77,256.45,246.84]'*57.3; %lbf/rad
C_ar = [242.0,242.0,242.0,241.9,241.6,241.1,240.4,239.2,237.4,235.0,231.8,227.5]'*57.3; %lbf/rad
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

syms y(t)
y(t) = piecewise((t>=0) & (t <= 0.0625),720*t,(t<=3.0625) & (t>=0.0625),45,(t>=3.0625) & (t<=3.1875),-720*(t-3.0625)+45,(t>=3.1875) & (t<=6.1875),-45,(t>=6.1875) & (t<=6.25),720*(t-6.1875)-45,(t<=9.25) & (t>=6.25),0);
xvalues = 0:0.01:9.25;
input_ackerman = double(y(xvalues))./15;
for i =1:length(V_x_SI)
    A = [ -2*(C_af_SI(i)+C_ar_SI(i))/(m_SI*V_x_SI(i)), -(2*(C_af_SI(i)*lf_SI-C_ar_SI(i)*lr_SI)/(m_SI*V_x_SI(i)^2)+1); -2*(C_af_SI(i)*lf_SI-C_ar_SI(i)*lr_SI)/Iz_SI, -2*(C_af_SI(i)*lf_SI^2+C_ar_SI(i)*lr_SI^2)/(Iz_SI*V_x_SI(i))];
    B = [2*C_af_SI(i)/m_SI/V_x_SI(i); 2*C_af_SI(i)*lf_SI/Iz_SI];
    sys = ss(A,B, eye(2), zeros(2,1));
    [o,t,x]= lsim(sys, input_ackerman,xvalues);
    plot(xvalues,o(:,2))
    hold on
    xlabel('time (s)')
    ylabel('Yaw rate (deg/s)')
    title('Yaw Rate vs time')
    grid on
    hold on
end
grid on
legend ('Vx = 10 mph','Vx = 20 mph','Vx = 30 mph','Vx = 40 mph','Vx = 50 mph','Vx = 60 mph','Vx = 70 mph','Vx = 80 mph','Vx = 90 mph','Vx = 100 mph','Vx = 110 mph','Vx = 120 mph')