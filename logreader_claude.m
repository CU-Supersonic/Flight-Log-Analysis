%% logreader.m
% Reads a PX4/ArduPilot-style .ulg flight log and plots vehicle GPS-derived
% position and velocity data, including a 3D flight path, N/E/D velocity
% components, and cumulative ground-track distance vs. time.

% written by John Mongellow, commented by Claude

% test change for pushing -Joe

clc; clear; close all

%% --- Load the log file ---
% ulogreader is a MATLAB function (Aerospace Toolbox / UAV Toolbox) that
% parses PX4 .ulg binary log files into a MATLAB object.
ulg = ulogreader('log_142_2026-8-19-17-19-08.ulg');

% Extract all topics (message streams) and their data from the log as a
% table, where each row corresponds to one topic/instance and its
% associated timetable of messages.
topics = readTopicMsgs(ulg);

%% --- Extract GPS position topic (instance 0) ---
% Logs can contain multiple instances of the same topic (e.g., dual GPS
% receivers). Here we specifically grab GPS instance 0.
idx0_gps = strcmp(topics.TopicNames, 'vehicle_gps_position') & topics.InstanceID == 0;
gpsData = topics.TopicMessages{idx0_gps};

% Convert timestamps to elapsed seconds since the first GPS sample.
t_gps = seconds(gpsData.timestamp - gpsData.timestamp(1));
%t_gps = (gpsData.timestamp);

% Pull out latitude, longitude, and MSL altitude (all doubles, in
% degrees/degrees/meters respectively).
ac_lat_gps = gpsData.latitude_deg;
ac_lon_gps = gpsData.longitude_deg;
ac_alt_gps = gpsData.altitude_msl_m;

% GPS-reported velocity components in the North-East-Down (NED) frame,
% cast to double for downstream math.
ac_v_n = double(gpsData.vel_n_m_s);   % North velocity (m/s)
ac_v_e = double(gpsData.vel_e_m_s);   % East velocity (m/s)
ac_v_d = double(gpsData.vel_d_m_s);   % Down velocity (m/s), positive = descending

%% --- Extract airspeed topic (instance 0) ---
idx0_airspd = strcmp(topics.TopicNames, 'airspeed') & topics.InstanceID == 0;
airspdData = topics.TopicMessages{idx0_airspd};

t_airspd = seconds(airspdData.timestamp - airspdData.timestamp(1));

airspd_ind = airspdData.indicated_airspeed_m_s;
airspd_true = airspdData.true_airspeed_m_s;

%% --- Convert LLA to local NED coordinates ---
% Stack lat/lon/alt into an Nx3 matrix for the conversion function.
pos_lla_gps = [ac_lat_gps, ac_lon_gps, ac_alt_gps];

% Use the very first GPS fix as the local tangent-plane origin.
pos_lla0_gps = pos_lla_gps(1,:);

% lla2ned converts geodetic coordinates to local North-East-Down
% Cartesian coordinates relative to the origin, using a flat-Earth
% approximation ("flat" method, valid for short-range/local flights).
pos_ned_gps = lla2ned(pos_lla_gps, pos_lla0_gps, "flat");

%% --- Plot 1: 3D flight path ---
% Note the sign flips: NED has Down as positive-z and implicitly
% negative "up," so negating N, E, D here effectively plots the path in
% a more intuitive (North, East, Up) sense for visualization.
figure()
plot3(-pos_ned_gps(:,1), -pos_ned_gps(:,2), -pos_ned_gps(:,3))
grid minor
axis equal
title("3D Flight Path (Local NED Origin at Launch)")
xlabel("North (m)")
ylabel("East (m)")
zlabel("Altitude / Up (m)")

%% --- Compute total ground speed magnitude ---
% Loop over each GPS sample and compute the 3D velocity vector norm
% (i.e., total speed) from the N/E/D components.
for i = 1:length(ac_v_n)
    ac_v_norm(i) = norm([ac_v_n(i), ac_v_e(i), ac_v_d(i)]);
    ac_v_xy(i) = norm([ac_v_n(i), ac_v_e(i)]);
end

ac_v_z = abs(ac_v_d);

%% --- Plot 2: Velocity components and total speed vs. time ---
figure()
sgtitle("GPS Velocity Components and Total Speed vs. Flight Time")

subplot(4,1,1)
plot(t_gps, ac_v_n)          % North velocity component
ylim([-15 15])
yline(0)
grid minor
title("North Velocity")
xlabel("Flight Time (s)")
ylabel("V_N (m/s)")

subplot(4,1,2)
plot(t_gps, ac_v_e)          % East velocity component
ylim([-15 15])
yline(0)
grid minor
title("East Velocity")
xlabel("Flight Time (s)")
ylabel("V_E (m/s)")

subplot(4,1,3)
plot(t_gps, ac_v_d)          % Down velocity component
ylim([-15 15])
yline(0)
grid minor
title("Down Velocity")
xlabel("Flight Time (s)")
ylabel("V_D (m/s)")

subplot(4,1,4)
plot(t_gps, ac_v_norm)       % Total speed magnitude
ylim([-15 15])
yline(0)
grid minor
title("Total Ground Speed Magnitude")
xlabel("Flight Time (s)")
ylabel("|V| (m/s)")

%% --- Plot 3: Velocity components and XY speed vs. time ---
tlims = [t_gps(1), t_gps(end)];

ms2mph = 2.236936;

figure()
subplot(3,1,1)
plot(t_gps, ac_v_xy)
xlim(tlims)
ylim([0 40])
grid minor
title("XY Velocity (mph)")
xlabel("Flight Time (s)")
ylabel("V_{XY} (m/s)")
yline(0)
yline(11.2 / ms2mph)
xline(t_gps(end) - 2)

subplot(3,1,2)
plot(t_gps, ac_v_z)
xlim(tlims)
ylim([0 15])
grid minor
title("Down Velocity (mph)")
xlabel("Flight Time (s)")
ylabel("V_D (m/s)")
yline(0)
yline(2.2 / ms2mph)
xline(t_gps(end) - 2)

subplot(3,1,3)
plot(t_airspd, airspd_ind)
hold on
plot(t_airspd, airspd_true)
xlim(tlims)
ylim([-40 40])
grid minor
title("Airspeed (mph)")
xlabel("Flight Time (s)")
ylabel("V_{XY} (m/s)")
yline(0)
yline(13.4 * ms2mph)
xline(t_gps(end) - 2)

%% --- Compute distance from origin over time ---
% For each NED position sample (using the same sign-flipped convention
% as the 3D plot), compute the straight-line distance from the launch
% point (the local origin defined above).
for i = 1:length(pos_ned_gps)
    pos_ned_gps_norm(i) = norm([-pos_ned_gps(i,1), -pos_ned_gps(i,2), -pos_ned_gps(i,3)]);
end

%% --- Plot 4: Distance from origin vs. flight time ---
figure()
plot(t_gps, pos_ned_gps_norm)
grid minor
title("Distance from GCS vs. Flight Time")
xlabel("Flight Time (s)")
ylabel("Distance from GCS (m)")

%% --- Detect longest period satisfying low-speed hold conditions ---
% Finds the longest contiguous stretch of time during which BOTH
% conditions are simultaneously true:
%   ac_v_xy(t) < v_xy_min   (horizontal speed below threshold)
%   ac_v_z(t)  < v_z_min    (vertical/descent speed below threshold)
% Thresholds are in m/s, matching the raw ac_v_xy / ac_v_z units
% (these are NOT mph-converted, unlike the values plotted above).

v_xy_min = 5 / ms2mph;   % [m/s] horizontal speed threshold -- SET AS NEEDED
v_z_min  = 1 / ms2mph;   % [m/s] vertical speed threshold  -- SET AS NEEDED

% Logical mask: true wherever both conditions hold at a given GPS sample.
cond_hold = (ac_v_xy(900:2680) < v_xy_min) & (ac_v_z(900:2680) < v_z_min);

% Identify contiguous runs of "true" in cond_hold using diff of the
% zero-padded logical vector. d==1 marks a run start, d==-1 marks the
% index just after a run ends.
d = diff([0, cond_hold, 0]);
run_starts = find(d == 1);      % indices into cond_hold where a run begins
run_ends   = find(d == -1) - 1; % indices into cond_hold where a run ends

if isempty(run_starts)
    fprintf('No period found where ac_v_xy < %.2f m/s and ac_v_z < %.2f m/s.\n', ...
        v_xy_min, v_z_min);
else
    % Duration of each run, computed from actual GPS timestamps (not
    % sample count) in case of irregular sampling.
    run_durations = t_gps(run_ends) - t_gps(run_starts);

    [max_duration, longest_idx] = max(run_durations);
    t_start = t_gps(run_starts(longest_idx));
    t_end   = t_gps(run_ends(longest_idx));

    fprintf('\n--- Longest low-speed hold period ---\n');
    fprintf('Conditions: ac_v_xy < %.2f m/s AND ac_v_z < %.2f m/s\n', v_xy_min, v_z_min);
    fprintf('Duration:   %.2f s\n', max_duration);
    fprintf('Start time: %.2f s (flight time)\n', t_start);
    fprintf('End time:   %.2f s (flight time)\n', t_end);
    fprintf('---------------------------------------\n\n');
end