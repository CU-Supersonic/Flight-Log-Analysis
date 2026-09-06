%% logreader.m
% Reads a PX4/ArduPilot-style .ulg flight log and plots vehicle
% local-position (EKF-fused) position, velocity, and acceleration data,
% including a 3D flight path, N/E/D velocity components, and cumulative
% ground-track distance vs. time.

% written by John Mongellow, commented by Claude

clc; clear; close all

%% --- Load the log file ---
% ulogreader is a MATLAB function (Aerospace Toolbox / UAV Toolbox) that
% parses PX4 .ulg binary log files into a MATLAB object.
ulg = ulogreader('log_231_2026-9-4-19-20-12.ulg');


% Extract all topics (message streams) and their data from the log as a
% table, where each row corresponds to one topic/instance and its
% associated timetable of messages.
topics = readTopicMsgs(ulg);

%% --- Extract local position topic (instance 0) ---

[locData, t_loc] = extractTopic(topics, 'vehicle_local_position');

% Local-XYZ position (m), relative to the EKF origin.
ac_x_loc = locData.x;
ac_y_loc = locData.y;
ac_z_loc = locData.z;

% Euclidean (straight-line) distance from the origin at each sample.
ac_pos_loc = vecnorm([ac_x_loc, ac_y_loc, ac_z_loc], 2, 2);

% Local-XYZ velocity (m/s).
ac_vx_loc = locData.vx;
ac_vy_loc = locData.vy;
ac_vz_loc = locData.vz;

% Total speed magnitude.
ac_v_loc = vecnorm([ac_vx_loc, ac_vy_loc, ac_vz_loc], 2, 2);
ac_vxy_loc = vecnorm([ac_vx_loc, ac_vy_loc], 2, 2);

% Local-XYZ acceleration (m/s^2), if logged.
ac_ax_loc = locData.ax;
ac_ay_loc = locData.ay;
ac_az_loc = locData.az;

% Total acceleration magnitude -- used later for takeoff detection.
ac_a_loc = vecnorm([ac_ax_loc, ac_ay_loc, ac_az_loc], 2, 2);

%% --- Extract local anglular velocity topic (instance 0) ---

[angData, t_ang] = extractTopic(topics, 'vehicle_angular_velocity');

% Local-XYZ angular position (Rad), relative to the EKF origin.
ac_xyz_ang = angData.xyz;
ac_dxyz_ang = angData.xyz_derivative;

%% --- Extract airspeed topic (instance 0) ---

[aspdData, t_aspd] = extractTopic(topics, 'airspeed');

aspd_ind = aspdData.indicated_airspeed_m_s;
aspd_true = aspdData.true_airspeed_m_s;

%% --- Extract telemetry topic (instance 0) ---

[telemData, t_telem] = extractTopic(topics, 'telemetry_status');

%% --- Extract radio topic (instance 0) ---

[radioData, t_radio] = extractTopic(topics, 'radio_status');

rssi = double(radioData.rssi) / 1.9 - 127;
noise = double(radioData.noise) / 1.9 - 127;
snr = rssi - noise;

%% --- Extract air data topic (instance 0) ---

[airdataData, t_airdata] = extractTopic(topics, 'vehicle_air_data');

%% --- Extract servo position topic (instance 0) ---

[servoData, t_servo] = extractTopic(topics, 'actuator_outputs');

throttle_pos = (servoData.output(:,3)-1000)/900;

%% --- Extract battery topic (instance 0) ---

[battData, t_batt] = extractTopic(topics, 'battery_status');

batt_v = battData.voltage_v;

%% --- Extract attitude topic (instance 0) ---

[attData, t_att] = extractTopic(topics, 'vehicle_attitude');

q_att = attData.q;  % Nx4, PX4 quaternion order: [w, x, y, z]

%% --- Extract gps topic (instance 0) ---

[gpsData, t_gps] = extractTopic(topics, 'vehicle_gps_position');

noise_gps = gpsData.noise_per_ms / 1.9 - 127;
sats_used = gpsData.satellites_used;

%% --- Plot 1: 3D flight path ---
% Note the sign flips: NED has Down as positive-z and implicitly
% negative "up," so negating x/y/z here effectively plots the path in a
% more intuitive (North, East, Up) sense for visualization.
figure()
plot3(-ac_x_loc, -ac_y_loc, -ac_z_loc)
grid minor
axis equal
title("3D Flight Path (Local-Position EKF Origin)")
xlabel("North (m)")
ylabel("East (m)")
zlabel("Altitude / Up (m)")

%% --- Compute total ground speed magnitude ---

ac_vz = abs(ac_vz_loc);

%% --- Plot 2: Velocity components and total speed vs. time ---
figure()
sgtitle("Local-Position (EKF) Velocity Components and Total Speed vs. Flight Time")

subplot(4,1,1)
plot(t_loc, ac_vx_loc)          % X velocity component
ylim([-20 20])
yline(0)
grid minor
title("X Velocity")
xlabel("Flight Time (s)")
ylabel("V_x (m/s)")

subplot(4,1,2)
plot(t_loc, ac_vy_loc)          % Y velocity component
ylim([-20 20])
yline(0)
grid minor
title("Y Velocity")
xlabel("Flight Time (s)")
ylabel("V_y (m/s)")

subplot(4,1,3)
plot(t_loc, ac_vz_loc)          % Z velocity component
ylim([-20 20])
yline(0)
grid minor
title("Z Velocity")
xlabel("Flight Time (s)")
ylabel("V_z (m/s)")

subplot(4,1,4)
plot(t_loc, ac_v_loc)           % Total speed magnitude
ylim([0 20])
yline(0)
grid minor
title("Total Ground Speed Magnitude")
xlabel("Flight Time (s)")
ylabel("|V| (m/s)")

%% --- Plot 3: Velocity components and total speed vs. time ---
figure()
sgtitle("Local-Position (EKF) Acceleration Components and Total Acceleration vs. Flight Time")

subplot(4,1,1)
plot(t_loc, ac_ax_loc)          % X velocity component
ylim([-20 20])
yline(0)
grid minor
title("X Acceleration")
xlabel("Flight Time (s)")
ylabel("a_x (m/s)")

subplot(4,1,2)
plot(t_loc, ac_ay_loc)          % Y velocity component
ylim([-20 20])
yline(0)
grid minor
title("Y Acceleration")
xlabel("Flight Time (s)")
ylabel("a_y (m/s)")

subplot(4,1,3)
plot(t_loc, ac_az_loc)          % Z velocity component
ylim([-20 20])
yline(0)
grid minor
title("Z Acceleration")
xlabel("Flight Time (s)")
ylabel("a_z (m/s)")

subplot(4,1,4)
plot(t_loc, ac_a_loc)           % Total speed magnitude
ylim([0 20])
yline(0)
grid minor
title("Total Ground Acceleration Magnitude")
xlabel("Flight Time (s)")
ylabel("|a| (m/s)")

%% --- Plot 4: Velocity components and XY speed vs. time ---
tlims = [t_loc(1), t_loc(end)];

ms2mph = 2.236936;

figure()
plot(t_aspd, aspd_true)
hold on
plot(t_loc, ac_v_loc)
xlim(tlims)
ylim([-5 20])
yline(0)
grid minor
title("Combined EKF magnitude and Airspeed (m/s)")
xlabel("Flight Time (s)")
ylabel("V (m/s)")

%% --- Plot 5: Distance from origin vs. flight time ---

figure()
plot(t_loc, ac_pos_loc)
grid minor
title("Distance from GCS vs. Flight Time")
xlabel("Flight Time (s)")
ylabel("Distance from GCS (m)")

%% --- Plot 6: SNR vs. flight time ---

figure()
plot(t_radio, snr)
ylim([0 100])
grid minor
title("Telemetry Radio Link SNR vs. Flight Time")
xlabel("Flight Time (s)")
ylabel("SNR (dB)")

%% --- Plot 7: Noise Floors vs. flight time ---

figure()
plot(t_gps, noise_gps)
ylim([-100 0])
grid minor
title("GPS Noise Floor vs. Flight Time")
xlabel("Flight Time (s)")
ylabel("Noise Floor (dB)")

%% --- Plot 8: Battery voltage vs. flight time ---

figure()
plot(t_batt, batt_v)
ylim([10 13])
grid minor
title("Battery Voltage vs. Flight Time")
xlabel("Flight Time (s)")
ylabel("Voltage (V)")

%% --- Resample attitude onto position timebase ---
% Attitude is usually logged much faster than position; interpolate
% quaternion components independently then re-normalize (cheap and fine
% for animation purposes; use SLERP if you need high-fidelity rotation).

q_interp = interp1(seconds(t_att), q_att, seconds(t_loc), 'linear', 'extrap');
q_interp = q_interp ./ vecnorm(q_interp, 2, 2);   % re-normalize unit quaternions

% %% --- Build a simple aircraft shape (nose along +X, body frame) ---
% % Replace this with an STL import (stlread) if you have a CAD model.
% 
% bodyLen = 5; wingSpan = 4; tailSpan = 2;
% verts = [ ...
%     bodyLen/2   0        0;      % nose
%     -bodyLen/2   0        0;      % tail
%     0           wingSpan/2  0;   % right wingtip
%     0          -wingSpan/2  0;   % left wingtip
%     -bodyLen/2   tailSpan/2  0;   % right tail
%     -bodyLen/2  -tailSpan/2  0];  % left tail
% 
% faces = [1 3 2; 1 2 4; 2 5 6];  % simple triangulated shape
% 
% %% --- Set up figure, static flight path, and moving aircraft patch ---
% 
% figure()
% plot3(-ac_x_loc, -ac_y_loc, -ac_z_loc, 'b:')   % faded full path for reference
% hold on
% grid minor
% axis equal
% xlabel("North (m)"); ylabel("East (m)"); zlabel("Altitude / Up (m)")
% title("Aircraft 3D Track Animation")
% view(3)
% 
% acPatch = patch('Vertices', verts, 'Faces', faces, ...
%     'FaceColor', 'red', 'EdgeColor', 'k');
% tform = hgtransform;
% acPatch.Parent = tform;
% 
% %% --- Animate ---
% 
% nFrames = length(t_loc);
% skip = 1;   % downsample frames for playback speed / render time
% 
% for i = 1:skip:nFrames
% 
%     pos = [-ac_x_loc(i), -ac_y_loc(i), -ac_z_loc(i)];
%     q = q_interp(i, :);
%     R = quat2rotm(q);
% 
%     S = diag([1 -1 -1]);
%     R_plot = S * R * S;
% 
%     T = eye(4);
%     T(1:3,1:3) = R_plot;
%     T(1:3,4) = pos';
% 
%     tform.Matrix = T;
% 
%     drawnow   % use plain drawnow, not limitrate, while debugging
%     pause(0.01)   % or compute actual dt from t_loc(i+skip)-t_loc(i)
% end