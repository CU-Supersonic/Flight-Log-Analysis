%% Wing Spar Deflection & Stress Analysis
% Sandwich I-beam spar: CF woven-fabric face sheets (caps) + balsa core (web)
% Geometry and loading taken from hand-sketch (root/tip cross-sections + v(x) lift dist.)
%
% Cantilever beam: root fixed (x=0), tip free (x=L)
% Method: Euler-Bernoulli beam theory, transformed-section approach for
% the composite (CF + balsa) cross section.
%
% ASSUMPTION FLAGGED: the sketch draws the balsa web as a square patch
% narrower than the full chord, but does not dimension its width. This
% script assumes web width = web height at each span station (i.e. the
% core stays "square" as drawn, tapering with depth). Replace w_web(x)
% below with a real value if you have one -- this is the single
% guessed dimension in the whole model.

clear; clc; close all;

%% ---------------- GEOMETRY (from sketch) ----------------
L      = 1.0;          % span, m
c_r    = 0.625;         % root chord, m
c_t    = 0.375;         % tip chord, m
t_r    = 0.0537;        % tip tickness, m
t_t    = 0.0270;        % 

t_skin = 0.002;         % CF skin (cap) thickness, EACH face, m

h_core_r = t_r - 2*t_skin;      % balsa core height at root (= t_r - 2*t_skin), m
h_core_t = t_t - 2*t_skin;      % balsa core height at tip  (= t_t - 2*t_skin), m

%% ---------------- MATERIAL PROPERTIES ----------------
% Conservative working values (woven fabric CF/epoxy, hand-layup; balsa core)
E_cf    = 65e9;         % Pa, woven CF/epoxy modulus (0/90 balanced fabric)
E_balsa = 3.0e9;        % Pa, balsa modulus along grain (typical 2.5-4 GPa)

sigma_allow_cf    = 350e6;  % Pa, conservative working allowable, woven CF
sigma_allow_balsa = 1.0e6;  % Pa, ~150-200 psi shear-driven working value;
                             % used here only as an axial sanity check

n_ratio = E_balsa / E_cf;   % modular ratio for transformed section

%% ---------------- LOADING (from sketch) ----------------
% v(x) = 363.8 - 181.9*x   [N/m], root = 0, tip = 1 m
w_r = 3638;   % N/m at root
w_t = 1819;   % N/m at tip

%% ---------------- DISCRETIZE SPAN ----------------
N = 1001;
x = linspace(0, L, N)';

% Chord taper (linear, root -> tip)
c = c_r + (c_t - c_r) * (x / L);

% Core (balsa web) height taper (linear, root -> tip)
h_core = h_core_r + (h_core_t - h_core_r) * (x / L);

% ASSUMED web width = web height (square core, as drawn)
w_web = h_core;

% Distributed load (matches v(x) given on sketch)
w = w_r + (w_t - w_r) * (x / L);

%% ---------------- SECTION PROPERTIES vs. SPAN ----------------
y_cap = h_core/2 + t_skin/2;         % centroid distance of each cap from NA

I_cap_own = c .* t_skin.^3 / 12;               % each cap's own I about its centroid
I_cap_pa  = c .* t_skin .* y_cap.^2;           % parallel-axis term
I_caps    = 2 * (I_cap_own + I_cap_pa);        % both caps (top+bottom)

I_core = w_web .* h_core.^3 / 12;              % solid balsa core

I_transformed = I_caps + n_ratio .* I_core;    % transformed to CF-equivalent
EI = E_cf .* I_transformed;                    % effective bending stiffness

%% ---------------- INTERNAL LOADS (cantilever, root fixed / tip free) ----------------
V = zeros(N,1);   % shear force
M = zeros(N,1);   % bending moment

for i = 1:N
    if i == N
        % only one point left (the tip) -- integral over zero-length span = 0
        V(i) = 0;
        M(i) = 0;
    else
        V(i) = trapz(x(i:end), w(i:end));                       % shear = load outboard of x
        M(i) = trapz(x(i:end), w(i:end) .* (x(i:end) - x(i)));   % moment = 1st moment of outboard load
    end
end

%% ---------------- CURVATURE, SLOPE, DEFLECTION ----------------
kappa = M ./ EI;                 % curvature

theta = zeros(N,1);              % slope, theta(0) = 0 (fixed root)
defl  = zeros(N,1);              % deflection, v(0) = 0 (fixed root)

for i = 2:N
    theta(i) = trapz(x(1:i), kappa(1:i));
end
for i = 2:N
    defl(i) = trapz(x(1:i), theta(1:i));
end

%% ---------------- STRESSES ----------------
sigma_cf    = M .* y_cap ./ I_transformed;                 % peak CF cap stress
sigma_balsa = n_ratio .* M .* (h_core/2) ./ I_transformed; % peak balsa axial stress (transformed)

MS_cf    = sigma_allow_cf ./ max(abs(sigma_cf)) - 1;
MS_balsa = sigma_allow_balsa ./ max(abs(sigma_balsa)) - 1;

%% ---------------- RESULTS SUMMARY ----------------
fprintf('=== Wing Spar Analysis Summary ===\n');
fprintf('Tip deflection:            %.4f m  (%.2f%% of span)\n', defl(end), 100*defl(end)/L);
fprintf('Tip slope:                 %.4f deg\n', rad2deg(theta(end)));
fprintf('Root bending moment:       %.2f N-m\n', M(1));
fprintf('Root shear force:          %.2f N\n', V(1));
fprintf('Max CF cap stress:         %.2f MPa  (allow %.0f MPa, MS = %.2f)\n', ...
        max(abs(sigma_cf))/1e6, sigma_allow_cf/1e6, MS_cf);
fprintf('Max balsa axial stress:    %.3f MPa  (allow %.2f MPa, MS = %.2f)\n', ...
        max(abs(sigma_balsa))/1e6, sigma_allow_balsa/1e6, MS_balsa);
fprintf('\nNOTE: balsa MS above is an axial sanity check only -- balsa core\n');
fprintf('failure in a sandwich shear web is normally shear- or buckling-\n');
fprintf('driven, not axial. Check core shear and cap wrinkling separately.\n');

%% ---------------- PLOTS ----------------
figure('Name','Wing Spar Analysis','Position',[100 100 900 700]);

subplot(3,2,1);
plot(x, w, 'LineWidth', 1.5); grid on;
xlabel('Span, x (m)'); ylabel('Load, w(x) (N/m)');
title('Distributed Aero Load');

subplot(3,2,2);
plot(x, V, 'LineWidth', 1.5); grid on;
xlabel('Span, x (m)'); ylabel('Shear, V(x) (N)');
title('Shear Force Diagram');

subplot(3,2,3);
plot(x, M, 'LineWidth', 1.5); grid on;
xlabel('Span, x (m)'); ylabel('Moment, M(x) (N-m)');
title('Bending Moment Diagram');

subplot(3,2,4);
plot(x, defl*1000, 'LineWidth', 1.5); grid on;
xlabel('Span, x (m)'); ylabel('Deflection, v(x) (mm)');
title(sprintf('Beam Deflection (tip = %.2f mm)', defl(end)*1000));

subplot(3,2,5);
plot(x, sigma_cf/1e6, 'LineWidth', 1.5); grid on;
xlabel('Span, x (m)'); ylabel('CF Cap Stress (MPa)');
title('CF Cap Bending Stress');
yline(sigma_allow_cf/1e6, 'r--', 'Allowable');

subplot(3,2,6);
plot(x, EI, 'LineWidth', 1.5); grid on;
xlabel('Span, x (m)'); ylabel('EI (N-m^2)');
title('Effective Bending Stiffness');

sgtitle('Tapered CF/Balsa Sandwich Wing Spar - Cantilever Analysis');