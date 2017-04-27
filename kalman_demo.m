clc;
clear all;
% close all;


% % Real IMU Data
% data = load('slide_data.txt');
% sample = data(:,1);
% mag_x = data(:,2);
% mag_y = data(:,3);
% mag_z = data(:,4);
% acc_x = data(:,5);
% acc_y = data(:,6);
% acc_z = data(:,7);
% gyr_x = data(:,8);
% gyr_y = data(:,9);
% gyr_z = data(:,10);
% lbutton = data(:,11);
% rbutton = data(:,12);

data = load('2DChrisTest.txt');
t = data(:,1);
acc_x = data(:,2);
acc_y = data(:,3);
acc_z = data(:,4);
gyr_x = data(:,5);
gyr_y = data(:,6);
gyr_z = data(:,7);

% cal_max = 150;
cal_max = 25;
acc_xcal = mean(acc_x(1:cal_max));
acc_ycal = mean(acc_y(1:cal_max));
acc_zcal = mean(acc_z(1:cal_max));
gyr_xcal = mean(gyr_x(1:cal_max));
gyr_ycal = mean(gyr_y(1:cal_max));
gyr_zcal = mean(gyr_z(1:cal_max));

acc_x = acc_x-acc_xcal;
acc_y = acc_y-acc_ycal;
acc_z = acc_z-acc_zcal;
gyr_x = gyr_x-gyr_xcal;
gyr_y = gyr_y-gyr_ycal;
gyr_z = gyr_z-gyr_zcal;

g = 9.81; % m/s^2
% acc_scale_factor = 3/1000/g; % Online Dataset
g_range = 8;
acc_bit_resolution = 2^16;
acc_scale_factor = g*g_range/acc_bit_resolution; % Our IMU
acc_x = acc_scale_factor*acc_x;
acc_y = acc_scale_factor*acc_y;
acc_z = acc_scale_factor*acc_z;

gyr_range = 4000; %deg/s
gyr_bit_resolution = 2^16;
gyr_scale_factor = gyr_range/gyr_bit_resolution;
gyr_x = gyr_scale_factor*gyr_x;
gyr_y = gyr_scale_factor*gyr_y;
gyr_z = gyr_scale_factor*gyr_z;

acc_xvar = var(acc_x(1:cal_max));
acc_yvar = var(acc_y(1:cal_max));
acc_zvar = var(acc_y(1:cal_max));
gyr_xvar = var(gyr_x(1:cal_max));
gyr_yvar = var(gyr_y(1:cal_max));
gyr_zvar = var(gyr_z(1:cal_max));

n = size(data,1);
% t = linspace(0,3.5,n);
dt = t(2)-t(1);

A = [ 1 0 0 dt 0 0 dt^2/2 0 0 0 0 0 0 0 0 ;
      0 1 0 0 dt 0 0 dt^2/2 0 0 0 0 0 0 0 ;
      0 0 1 0 0 dt 0 0 dt^2/2 0 0 0 0 0 0 ;
      0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 ;
      0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 ;
      0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 ;
      0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ;
      0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 ;
      0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 ;
      0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 ;
      0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 ;
      0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt ;
      0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 ;
      0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 ;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ];
H = [ 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ;
      0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 ;
      0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 ;
      0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 ;
      0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 ;
      0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ];

ux = .25;
uy = .25;
uz = .25;
% ux = acc_xvar;
% uy = acc_yvar;
% uz = acc_zvar;
uphi = .25;
utheta = .25;
upsi = .25;
% uphi = gyr_xvar;
% utheta = gyr_zvar;
% upsi = gyr_zvar;
eps_acc = 0.05;
eps_gyr = 0.05;

R = diag([ux uy uz uphi utheta upsi]);
Q = diag([zeros(1,6), eps_acc, eps_acc, eps_acc, 0, 0, 0, eps_gyr, eps_gyr, eps_gyr]);
P = [ zeros(6,15); zeros(9,6), diag([ux, uy, uy, 0, 0, 0, uphi, utheta, upsi])];
x = zeros(15,1);

% Kalman
global_x = [0; 0; 0]; % inertial position
% dcm = [ 1 0 0; 0 1 0; 0 0 1 ];
for k = 1:n
    om_p = gyr_x(k);
    om_q = gyr_y(k);
    om_r = gyr_z(k);
    phi = x(10);
    theta = x(11);
    psi = x(12);
    phidot = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta)]*[om_p;om_q;om_r];
    thetadot = [0 cosd(phi) -sind(phi)]*[om_p;om_q;om_r];
    psidot = [0 sind(phi)*secd(theta) cosd(phi)*secd(theta)]*[om_p;om_q;om_r];
%     z = [acc_x(k) acc_y(k) acc_z(k) gyr_x(k) gyr_y(k) gyr_z(k)]';
    z = [acc_x(k) acc_y(k) acc_z(k) phidot thetadot psidot]';
    x = A*x;
    P = A*P*A' + Q;
    K = P*H'/(H*P*H'+R);
    x = x+K*(z-H*x);
    P = (eye(length(x))-K*H)*P;
    kalman_pos_x(k) = x(1);
    kalman_pos_y(k) = x(2);
    kalman_pos_z(k) = x(3);
    kalman_vel_x(k) = x(4);
    kalman_vel_y(k) = x(5);
    kalman_vel_z(k) = x(6);
    kalman_acc_x(k) = x(7);
    kalman_acc_y(k) = x(8);
    kalman_acc_z(k) = x(9);
    kalman_phi(k) = x(10);
    kalman_theta(k) = x(11);
    kalman_psi(k) = x(12);
    kalman_phidot(k) = x(13);
    kalman_thetadot(k) = x(14);
    kalman_psidot(k) = x(15);
%     This part doesn't work.
%     dphi = x(13)*dt;
%     dtheta = x(14)*dt;
%     dpsi = x(15)*dt;
%     dcm = dcm * [ 1, -dpsi, dtheta; dpsi, 1, -dphi; -dtheta, dphi, 1 ];
    phi = x(10);
    theta= x(11);
    psi = x(12);
    
    dcy = [cosd(psi) sind(psi) 0; -sind(psi) cosd(psi) 0; 0 0 1];
    dcp = [cosd(theta) 0 -sind(theta); 0 1 0; sind(theta) 0 cosd(theta)];
    dcr = [1 0 0; 0 cosd(phi) sind(phi); 0 -sind(phi) cosd(phi)];
    dcm = dcy*dcp*dcr;
    body_vel = [x(4);x(5);x(6)];
    global_vel = dcm*body_vel;
    global_x = global_x + global_vel*dt;
    kalman_global_x(k) = global_x(1);
    kalman_global_y(k) = global_x(2);
    kalman_global_z(k) = global_x(3);
end


% Direct
x = zeros(15,1);
global_x = [0; 0; 0]; % inertial position
for k = 1:n
    om_p = gyr_x(k);
    om_q = gyr_y(k);
    om_r = gyr_z(k);
    phi = x(10);
    theta = x(11);
    psi = x(12);
    phidot = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta)]*[om_p;om_q;om_r];
    thetadot = [0 cosd(phi) -sind(phi)]*[om_p;om_q;om_r];
    psidot = [0 sind(phi)*secd(theta) cosd(phi)*secd(theta)]*[om_p;om_q;om_r];
    
    x = A*x;
    x(7) = acc_x(k);
    x(8) = acc_y(k);
    x(9) = acc_z(k);
    x(13) = phidot;
    x(14) = thetadot;
    x(15) = psidot;
%     x(13) = gyr_x(k);
%     x(14) = gyr_y(k);
%     x(15) = gyr_z(k);
    raw_pos_x(k) = x(1);
    raw_pos_y(k) = x(2);
    raw_pos_z(k) = x(3);
    raw_vel_x(k) = x(4);
    raw_vel_y(k) = x(5);
    raw_vel_z(k) = x(6);
    raw_acc_x(k) = x(7);
    raw_acc_y(k) = x(8);
    raw_acc_z(k) = x(9);
    raw_phi(k) = x(10);
    raw_theta(k) = x(11);
    raw_psi(k) = x(12);
    raw_phidot(k) = x(13);
    raw_thetadot(k) = x(14);
    raw_psidot(k) = x(15);
    
    phi = x(10);
    theta= x(11);
    psi = x(12);
    
    dcy = [cosd(psi) sind(psi) 0; -sind(psi) cosd(psi) 0; 0 0 1];
    dcp = [cosd(theta) 0 -sind(theta); 0 1 0; sind(theta) 0 cosd(theta)];
    dcr = [1 0 0; 0 cosd(phi) sind(phi); 0 -sind(phi) cosd(phi)];
    dcm = dcy*dcp*dcr;
    body_vel = [x(4);x(5);x(6)];
    global_vel = dcm*body_vel;
    global_x = global_x + global_vel*dt;
    raw_global_x(k) = global_x(1);
    raw_global_y(k) = global_x(2);
    raw_global_z(k) = global_x(3);
end

% Low Pass
Navg = 5;
x = zeros(15,1);
global_x = [0; 0; 0]; % inertial position
for k = 1:n
    om_p = gyr_x(k);
    om_q = gyr_y(k);
    om_r = gyr_z(k);
    phi = x(10);
    theta = x(11);
    psi = x(12);
    phidot(k) = [1 sind(phi)*tand(theta) cosd(phi)*tand(theta)]*[om_p;om_q;om_r];
    thetadot(k) = [0 cosd(phi) -sind(phi)]*[om_p;om_q;om_r];
    psidot(k) = [0 sind(phi)*secd(theta) cosd(phi)*secd(theta)]*[om_p;om_q;om_r];
    
    x = A*x;
    i = min([k,Navg])-1;
    x(7) = mean(acc_x(k-i:k));
    x(8) = mean(acc_y(k-i:k));
    x(9) = mean(acc_z(k-i:k));
    x(13) = mean(phidot(k-i:k));
    x(14) = mean(thetadot(k-i:k));
    x(15) = mean(psidot(k-i:k));
%     x(13) = mean(gyr_x(k-i:k));
%     x(14) = mean(gyr_y(k-i:k));
%     x(15) = mean(gyr_z(k-i:k));
    lowpass_pos_x(k) = x(1);
    lowpass_pos_y(k) = x(2);
    lowpass_pos_z(k) = x(3);
    lowpass_vel_x(k) = x(4);
    lowpass_vel_y(k) = x(5);
    lowpass_vel_z(k) = x(6);
    lowpass_acc_x(k) = x(7);
    lowpass_acc_y(k) = x(8);
    lowpass_acc_z(k) = x(9);
    lowpass_phi(k) = x(10);
    lowpass_theta(k) = x(11);
    lowpass_psi(k) = x(12);
    lowpass_phidot(k) = x(13);
    lowpass_thetadot(k) = x(14);
    lowpass_psidot(k) = x(15);
    
    phi = x(10);
    theta= x(11);
    psi = x(12);
    
    dcy = [cosd(psi) sind(psi) 0; -sind(psi) cosd(psi) 0; 0 0 1];
    dcp = [cosd(theta) 0 -sind(theta); 0 1 0; sind(theta) 0 cosd(theta)];
    dcr = [1 0 0; 0 cosd(phi) sind(phi); 0 -sind(phi) cosd(phi)];
    dcm = dcy*dcp*dcr;
    body_vel = [x(4);x(5);x(6)];
    global_vel = dcm*body_vel;
    global_x = global_x + global_vel*dt;
    lowpass_global_x(k) = global_x(1);
    lowpass_global_y(k) = global_x(2);
    lowpass_global_z(k) = global_x(3);
end



fignum = 1;
% Plot Raw Data
figure(fignum); fignum = fignum+1;
plot(t,acc_x,t,acc_y,t,acc_z);
legend('x','y','z');
title('Acceleration Data');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

figure(fignum); fignum = fignum+1;
plot(t,gyr_x,t,gyr_y,t,gyr_z);
legend('x','y','z');
title('Gyro Data');
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');

% figure(fignum); fignum = fignum+1;
% plot(t,mag_x,t,mag_y,t,mag_z);
% legend('x','y','z');
% title('Magnetic Field Data');
% xlabel('Time (s)');
% ylabel('Magnetic Field (\muT)');



figure(fignum); fignum = fignum+1;
subplot(3,1,1);
plot(t,raw_acc_x,t,kalman_acc_x,t,lowpass_acc_x);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Acceleration Results');
xlabel('Time (s)');
ylabel('x-Acceleration (m/s^2)');
subplot(3,1,2);
plot(t,raw_acc_y,t,kalman_acc_y,t,lowpass_acc_y);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('y-Acceleration (m/s^2)');
subplot(3,1,3);
plot(t,raw_acc_z,t,kalman_acc_z,t,lowpass_acc_z);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('z-Acceleration (m/s^2)');

figure(fignum); fignum = fignum+1;
subplot(3,1,1);
plot(t,raw_vel_x,t,kalman_vel_x,t,lowpass_vel_x);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Velocity Results');
xlabel('Time (s)');
ylabel('x-Velocity (m/s)');
subplot(3,1,2);
plot(t,raw_vel_y,t,kalman_vel_y,t,lowpass_vel_y);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('y-Velocity (m/s)');
subplot(3,1,3);
plot(t,raw_vel_z,t,kalman_vel_z,t,lowpass_vel_z);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('z-Velocity (m/s)');

figure(fignum); fignum = fignum+1;
subplot(3,1,1);
plot(t,raw_pos_x,t,kalman_pos_x,t,lowpass_pos_x);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Position Results');
xlabel('Time (s)');
ylabel('x (m)');
subplot(3,1,2);
plot(t,raw_pos_y,t,kalman_pos_y,t,lowpass_pos_y);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('y (m)');
subplot(3,1,3);
plot(t,raw_pos_z,t,kalman_pos_z,t,lowpass_pos_z);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('z (m)');

figure(fignum); fignum = fignum+1;
subplot(3,1,1);
plot(t,raw_global_x,t,kalman_global_x,t,lowpass_global_x);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Global Position Results');
xlabel('Time (s)');
ylabel('x (m)');
subplot(3,1,2);
plot(t,raw_global_y,t,kalman_global_y,t,lowpass_global_y);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('y (m)');
subplot(3,1,3);
plot(t,raw_global_z,t,kalman_global_z,t,lowpass_global_z);
legend('Raw','Kalman Filter','Low Pass Filter');
xlabel('Time (s)');
ylabel('z (m)');



figure(fignum); fignum = fignum+1;
subplot(3,1,1);
plot(t,raw_phi,t,kalman_phi,t,lowpass_phi);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Roll Angle');
xlabel('Time (s)');
ylabel('Roll angle \phi (deg)');
subplot(3,1,2);
plot(t,raw_theta,t,kalman_theta,t,lowpass_theta);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Pitch Angle');
xlabel('Time (s)');
ylabel('Pitch angle \theta (deg)');
subplot(3,1,3);
plot(t,raw_psi,t,kalman_psi,t,lowpass_psi);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Yaw Angle');
xlabel('Time (s)');
ylabel('Yaw angle \psi (deg)');


figure(fignum); fignum = fignum+1;
subplot(3,1,1);
plot(t,raw_phidot,t,kalman_phidot,t,lowpass_phidot);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Roll Rate');
xlabel('Time (s)');
ylabel('Roll rate d\phi/dt (deg/s)');
subplot(3,1,2);
plot(t,raw_thetadot,t,kalman_thetadot,t,lowpass_thetadot);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Pitch Rate');
xlabel('Time (s)');
ylabel('d\theta/dt (deg/s)');
subplot(3,1,3);
plot(t,raw_psidot,t,kalman_psidot,t,lowpass_psidot);
legend('Raw','Kalman Filter','Low Pass Filter');
title('Yaw Rate');
xlabel('Time (s)');
ylabel('d\psi/dt (deg/s)');

figure(fignum); fignum = fignum+1;
% plot(kalman_pos_x,kalman_pos_y,'b',raw_pos_x,raw_pos_y,'r',lowpass_pos_x,lowpass_pos_y,'g');
plot(kalman_global_x,kalman_global_y,'b',raw_global_x,raw_global_y,'r',lowpass_global_x,lowpass_global_y,'g');
hold on;
% pk = plot(kalman_pos_x(1),kalman_pos_y(1),'o','markerfacecolor','blue');
% pd = plot(raw_pos_x(1),raw_pos_y(1),'o','markerfacecolor','red');
% pl = plot(lowpass_pos_x(1),lowpass_pos_y(1),'o','markerfacecolor','green');
pk = plot(kalman_global_x(1),kalman_global_y(1),'o','markerfacecolor','blue');
pd = plot(raw_global_x(1),raw_global_y(1),'o','markerfacecolor','red');
pl = plot(lowpass_global_x(1),lowpass_global_y(1),'o','markerfacecolor','green');
hold off;
axis equal
legend('Kalman','Raw','Low Pass');
for k = 2:n
    pk.XData = kalman_global_x(k);
    pk.YData = kalman_global_y(k);
    pd.XData = raw_global_x(k);
    pd.YData = raw_global_y(k);
    pl.XData = lowpass_global_x(k);
    pl.YData = lowpass_global_y(k);
%     pk.XData = kalman_pos_x(k);
%     pk.YData = kalman_pos_y(k);
%     pd.XData = raw_pos_x(k);
%     pd.YData = raw_pos_y(k);
%     pl.XData = lowpass_pos_x(k);
%     pl.YData = lowpass_pos_y(k);
    drawnow
end
