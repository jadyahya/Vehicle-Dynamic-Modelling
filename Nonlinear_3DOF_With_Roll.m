%% Simulations

clear
clc

m = 3000; % in lbm
Iz = 40000; %lbm*ft^2 
Ixx = 15000; %lbm*ft^2 
lf = 3.5; %ft 
lr = 4.5; %ft
h1 = 0.5; %ft 
hcg = 1.2; %ft
t = 6; %ft
fo = 1000; %lb

V_x_SI = 60*0.44704; %m/s
m_SI = 3000*0.453592; %in kg
Iz_SI = Iz/23.73036; %in kg*m^2
Ixx_SI = Ixx/23.73036; %in kg*m^2
lf_SI =3.5*0.305; % in m
lr_SI =4.5*0.305; %in m
l_SI=lf_SI+lr_SI;
h1_SI = h1*0.305; %in m
hcg_SI = hcg*0.305; %in m
hr_SI=hcg_SI-h1_SI;
t_SI = 6*0.305; %ft
fo_SI = fo*4.448; %N
ms_SI = 2700*0.453592;

epsf =0.04;
epsr=0.04;

mr_SI = m_SI*lf_SI/l_SI;
mf_SI = m_SI*lr_SI/l_SI;
Wf_SI=mf_SI*9.81;
Wr_SI=mr_SI*9.81;


for i=10:10:120
    % Normal Damping & Stiffness
    roll_stiffnessf = 13317.5;
    roll_dampingf = 1335.8;

    roll_stiffnessr = 8701;
    roll_dampingr = 667.9;


    S= sim('Part4a_STABLE',9.25);
    V_x_SI= i*0.447;
    figure(1)
    plot(S.tout,S.beta)
    xlabel('Time (s)')
    ylabel('Beta (rad)')
    title('Side slip angle vs time for Normal Damping & Stiffness')
    grid on
    hold on

    figure(2)
    plot(S.tout,S.r)
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate vs time for Normal Damping & Stiffness')
    grid on
    hold on

    figure(3)
    plot(S.tout,S.phi)
    xlabel('Time (s)')
    ylabel('Roll angle (rad)')
    title('Roll angle vs time for Normal Damping & Stiffness')
    grid on
    hold on

    %Normal Damping & Low Stiffness
    roll_stiffnessf = 1000;
    roll_dampingf = 1335.8;

    roll_stiffnessr = 870;
    roll_dampingr = 667.9;


    S= sim('Part4a_STABLE',9.25);

    figure(4)
    plot(S.tout,S.beta)
    xlabel('Time (s)')
    ylabel('Beta (rad)')
    title('Side slip angle vs time with Normal Damping & Low Stiffness')
    grid on
    hold on

    figure(5)
    plot(S.tout,S.r)
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate vs time with Normal Damping & Low Stiffness')
    grid on
    hold on

    figure(6)
    plot(S.tout,S.phi)
    xlabel('Time (s)')
    ylabel('Roll angle (rad)')
    title('Roll angle vs time with Normal Damping & Low Stiffness')
    grid on
    hold on

    % Normal Damping & High Stiffness
    roll_stiffnessf = 133170.5;
    roll_dampingf = 1335.8;

    roll_stiffnessr = 87010;
    roll_dampingr = 667.9;


    S= sim('Part4a_STABLE',9.25)

    figure(7)
    plot(S.tout,S.beta)
    xlabel('Time (s)')
    ylabel('Beta (rad)')
    title('Side slip angle vs time with Normal Damping & High Stiffness')
    grid on
    hold on

    figure(8)
    plot(S.tout,S.r)
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate vs time with Normal Damping & High Stiffness')
    grid on
    hold on

    figure(9)
    plot(S.tout,S.phi)
    xlabel('Time (s)')
    ylabel('Roll angle (rad)')
    title('Roll angle vs time with Normal Damping & High Stiffness')
    grid on
    hold on

    % CHANGING DAMPING

    % Low Damping & Normal Stiffness
    roll_stiffnessf = 13317.5;
    roll_dampingf = 133.8;

    roll_stiffnessr = 8701;
    roll_dampingr = 66.9;


    S= sim('Part4a_STABLE',9.25);

    figure(10)
    plot(S.tout,S.beta)
    xlabel('Time (s)')
    ylabel('Beta (rad)')
    title('Side slip angle vs time with Low Damping & Normal Stiffness')
    grid on
    hold on

    figure(11)
    plot(S.tout,S.r)
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate vs time with Low Damping & Normal Stiffness')
    grid on
    hold on

    figure(12)
    plot(S.tout,S.phi)
    xlabel('Time (s)')
    ylabel('Roll angle (rad)')
    title('Roll angle vs time with Low Damping & Normal Stiffness')
    grid on
    hold on

    % Normal Stiffness & High Damping
    roll_stiffnessf = 13317.5;
    roll_dampingf = 13350.8;

    roll_stiffnessr = 8701;
    roll_dampingr = 6670.9;


    S= sim('Part4a_STABLE',10)

    figure(13)
    plot(S.tout,S.beta)
    xlabel('Time (s)')
    ylabel('Beta (rad)')
    title('Side slip angle vs time with Normal Stiffness & High Damping')
    grid on
    hold on

    figure(14)
    plot(S.tout,S.r)
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate vs time with Normal Stiffness & High Damping')
    grid on
    hold on

    figure(15)
    plot(S.tout,S.phi)
    xlabel('Time (s)')
    ylabel('Roll angle (rad)')
    title('Roll angle vs time with Normal Stiffness & High Damping')
    grid on
    hold on
    
    
    
end

for i = 1: 15
    figure(i)
    legend ('Vx = 10 mph','Vx = 20 mph','Vx = 30 mph','Vx = 40 mph','Vx = 50 mph','Vx = 60 mph','Vx = 70 mph','Vx = 80 mph','Vx = 90 mph','Vx = 100 mph','Vx = 110 mph','Vx = 120 mph')
end


%% NON LINEAR DAMPING AND STIFFNESS


for i=10:10:120
    % Normal Damping & Stiffness
    roll_stiffnessf = 10*i^2;
    roll_dampingf = 1*i^2;

    roll_stiffnessr = 60*i^2;
    roll_dampingr = 3*i^2;


    S= sim('Part4a',10);
    V_x_SI= i;
    figure(16)
    plot(S.tout,S.beta)
    xlabel('Time (s)')
    ylabel('Beta (rad)')
    title('Side slip angle vs time for Normal Damping & Stiffness')
    grid on
    hold on

    figure(17)
    plot(S.tout,S.r)
    xlabel('Time (s)')
    ylabel('Yaw rate (rad/s)')
    title('Yaw rate vs time for Normal Damping & Stiffness')
    grid on
    hold on

    figure(18)
    plot(S.tout,S.phi)
    xlabel('Time (s)')
    ylabel('Roll angle (rad)')
    title('Roll angle vs time for Normal Damping & Stiffness')
    grid on
    hold on  
    
end

for i = 16: 18
    figure(i)
    legend ('Vx = 10 mph','Vx = 20 mph','Vx = 30 mph','Vx = 40 mph','Vx = 50 mph','Vx = 60 mph','Vx = 70 mph','Vx = 80 mph','Vx = 90 mph','Vx = 100 mph','Vx = 110 mph','Vx = 120 mph')
end
