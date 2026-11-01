%%%% Level 2 Tripoli Rocket Sim
%%%%     Charles Massey
  


%%%% thrust curve from online credits:John Coker
clear; clc; close all;
%%%%Inputs:
mass = 6.064;   %kg with motor
d = 4*.0254; %m
area = pi*d^2 / 4;
burn_time = 2.21;  %s
mass_prop = .679;   %kg
C_D = .55;
g = 9.81;   %m/s^2
y0 = [0;0];   %%%initial height and velocity
tt = linspace(0, 3, 300);   % 0-3 seconds
motor_curve = [0,0;
             .04, 498.76;
             .13, 639.62;
             .22, 649.32;
             .31, 657.97;
             .4, 664.02;
             .49, 666.92;
             .58, 663.7;
             .68, 658.4;
             .77, 651.23;
             .85, 638.51;
             .94, 626.4;
             1.03, 612.56;
             1.13, 590.09;
             1.22, 562.39;
             1.31, 536.88;
             1.4, 511.61;
             1.49, 490.35;
             1.58, 468.98;
             1.67, 451.34;
             1.76, 430.18;
             1.85, 414.55;
             1.94, 398.12;
             2.03, 305.88;
             2.12, 55.54;
             2.21, 1.52];
%%% Motor model
TimeCurve = motor_curve(:,1);
ThrustCurve = motor_curve(:,2);
I_total = trapz(TimeCurve, ThrustCurve);
T_t = @(t) interp1 (TimeCurve,ThrustCurve,t,'linear',0);
I_cum = cumtrapz(TimeCurve, ThrustCurve);
I_t = @(t) interp1(TimeCurve, I_cum, t, 'linear', I_total);
m_t = @(t) mass - mass_prop * I_t(t) / I_total;

figure
subplot(3,1,1)
plot(tt, T_t(tt))        % thrust
ylabel('Thrust (N)')
subplot(3,1,2)
plot(tt,I_t(tt))        % impulse
ylabel('Impulse (N*s)')
subplot(3,1,3)
plot(tt, m_t(tt))        % mass
ylabel('Mass (kg)'); xlabel('Time (s)')
%%%%Results
eom = @(t, y) rocket_eom(t, y, T_t, m_t, C_D, area, g);
opts = odeset('MaxStep',.01);
[t,y] = ode45(eom,[0,30],y0,opts);   %%%%ODE 45!!!!!
h = y(:,1) *3.28084 ;
v = y(:,2) * 3.28084;
Apogee = max(h);
VMax = max(v);
fprintf('Apogee= %.2f feet\n', Apogee)
fprintf('Max Velocity = %.2f ft/s\n', VMax)
figure
subplot(2,1,1)
plot(t,h)
xlabel('TIME (s)')
ylabel('HEIGHT (ft)')
yline(0)
subplot(2,1,2)
plot(t,v)
xlabel('TIME (s)')
ylabel('VELOCITY (ft/s)')



%%%%Equations of motion
function dydt = rocket_eom(t, y, T_t, m_t, C_D, area, g)
h = y(1);
v = y(2);
rho = 1.225 * (1 - 2.25577e-5* h)^4.2559;
mass = m_t(t);
weight = mass*g;
thrust = T_t(t);
drag = 0.5 * rho * C_D * v*abs(v) * area;
a = (thrust-weight-drag)/mass;

if h<=0 && a <0    %%%on the pad
    a = 0;
end
dydt = [v; a];
end