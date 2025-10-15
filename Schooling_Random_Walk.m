function y_out = Schooling_Random_Walk()
% schooling_FieldOfVision
% ---------------------------------------------------------------
% Agent-based "boids" schooling in 2D with:
%  - Cohesion (attraction), Separation (repulsion), Alignment
%  - Optional Field-of-View (range + angle) for alignment
%  - Optional Predator (boids repel; predator seeks nearby boids)
%  - Toroidal (wrap-around) boundaries
%  - Live plotting
%  - Random Walk - Each boid receives zero-mean Gaussian noise added to its velocity

%% VISUAL DEFAULTS
close all;
set(0,'DefaultLineLineWidth',1.5,'DefaultLineMarkerSize',6,...
      'DefaultTextFontSize',12,'DefaultAxesFontSize',12);

rng(1); % set seed for reproducibility (change/remove if you like)

%% USER PARAMETERS ---------------------------------------------------------
% Simulation
N          = 80;     % number of boids (prey)
frames     = 400;    % number of frames
limit      = 100;    % axis extent half-width (plot is [-limit, limit])
L          = 2*limit;% domain size for torus wrapping
delta      = 1.0;    % time step

% Initial conditions (Gaussian)
P          = 10;     % position spread
V          = 10;     % velocity spread

% Boid gains
c_cohesion     = 1.0e-5;  % attraction gain
c_separation   = 1.0e-2;  % repulsion gain (inverse-square)
c_align        = 0.6;     % alignment gain
c_noise        = 0.08;    % <<< RANDOM-WALK NOISE GAIN
vlimit         = 4.0;     % max speed per boid

% Alignment mode / field of view
use_fov        = true;    % true => local (radius+angle) alignment
searchRadius   = 18;      % neighbor distance for FOV
searchAngleDeg = 150;     % FOV half-angle in degrees (0..180)

use_global_drift = false; % if true, add a small constant drift to the right
c_drift          = 0.0;   % magnitude of that drift (try 0.1)

% Predator settings
use_predator       = true;
pred_gain_repulse  = 12.0;    % boid repulsion from predator (inverse-square)
pred_seek_gain     = 0.8;     % predator steering toward nearby prey (FOV-averaged)
pred_searchRadius  = 45;      % predator "search radius" for steering
pred_searchAngle   = 35;      % predator steering FOV (deg)
pred_vlimit        = 5.0;     % predator speed cap
pred_noise_gain    = 0.0;     % optional predator jitter (e.g., 0.03)
pred_init_offset   = [40; 40];% initial predator offset

% Plotting
show_quiver    = true;  % draw velocity arrows (slower)

%% INITIALIZE --------------------------------------------------------------
if use_predator
    numAgents = N + 1;        % last agent is predator
else
    numAgents = N;
end

% Positions and velocities: columns are agents
p = P * randn(2, numAgents);
v = V * randn(2, numAgents);

% Normalize initial velocities to unit speed per column
v = normalize_columns(v);

% Optionally offset predator start
if use_predator
    p(:, end) = p(:, end) + pred_init_offset;
end

fig = figure('Color','w','Position',[100 100 800 800]);
set(fig,'InvertHardcopy','off');
set(fig,'Renderer','opengl');   % safer for getframe

ax = axes('Parent',fig,'Color','w');   % persistent white axes
axis(ax,'equal');
% turn OFF all gridlines and axis numbers/labels
grid(ax,'off'); box(ax,'on');
ax.XTick = []; ax.YTick = [];
ax.XLabel.String = ''; ax.YLabel.String = '';
ax.XColor = [0 0 0];  % axis line color
ax.YColor = [0 0 0];
ax.LineWidth = 1.0;   % thickness of the axis lines
ax.Title.Color = [0 0 0];

% --- VIDEO SETUP  ---
record_video = true; % toggle recording on/off

if record_video
    outdir = 'media'; if ~exist(outdir,'dir'), mkdir(outdir); end

    % don't reuse 'p' or 'v' here — avoid collisions with positions/velocities
    profNames = strings(0,1);
    try
        profs = VideoWriter.getProfiles;
        profNames = lower(string({profs.Name}));
    catch
        % some environments don't support getProfiles; we'll just fall back
        profNames = strings(0,1);
    end

    if any(profNames == "mpeg-4")
        outfile = fullfile(outdir,'schooling_demo.mp4');
        vw = VideoWriter(outfile,'MPEG-4');
    else
        outfile = fullfile(outdir,'schooling_demo.avi');
        vw = VideoWriter(outfile,'Motion JPEG AVI');
    end

    vw.FrameRate = 30;
    open(vw);
end
% --- END VIDEO SETUP ---



%% MAIN LOOP ---------------------------------------------------------------
for k = 1:frames
    % Buffers
    v_coh   = zeros(2, N);  % cohesion
    v_sep   = zeros(2, N);  % separation
    v_align = zeros(2, N);  % alignment

    % ---------------- RANDOM WALK TERM -----------------------------------
    % Zero-mean Gaussian noise added to boid velocities each step. This
    % creates a stochastic component of motion; with only this term active,
    % the system behaves like a 2D random walk (with speed limiting).
    v_noise = c_noise * randn(2, N);
    % ---------------------------------------------------------------------

    % Optional constant drift (e.g., to the right)
    v_drift = [1; 0] * c_drift;
    if norm(v_drift) > vlimit
        v_drift = (v_drift / norm(v_drift)) * vlimit;
    end

    % --- Compute interactions among boids (O(N^2))
    for n = 1:N
        sumHead = [0; 0]; countHead = 0; % for FOV alignment

        for m = 1:N
            if m == n, continue; end

            % Displacement with torus wrapping
            r = wrap_delta(p(:,m) - p(:,n), L);
            rmag = max(norm(r), 1e-9); % epsilon for stability

            % Cohesion (attraction)
            v_coh(:,n) = v_coh(:,n) + c_cohesion * r;

            % Separation (inverse-square repulsion)
            v_sep(:,n) = v_sep(:,n) - c_separation * (r / (rmag^2));

            % Local alignment (FOV) accumulation
            if use_fov
                if rmag <= searchRadius
                    theta = angle_between(v(:,n), r); % angle between heading and neighbor direction
                    if theta <= searchAngleDeg
                        sumHead   = sumHead + v(:,m);
                        countHead = countHead + 1;
                    end
                end
            end
        end

        % Alignment term
        if use_fov
            if countHead > 0
                avgHead = sumHead / countHead;
                if any(avgHead)
                    v_align(:,n) = c_align * (avgHead / max(norm(avgHead),1e-9));
                end
            else
                v_align(:,n) = [0; 0];
            end
        else
            % Global average alignment (over boids only)
            avgV = mean(v(:,1:N), 2);
            if any(avgV)
                v_align(:,n) = c_align * (avgV / max(norm(avgV),1e-9));
            end
        end
    end

    % Predator interactions ------------------------------------------------
    pred_steer = [0;0];
    if use_predator
        predIdx = numAgents;
        % Boid repulsion from predator
        for n = 1:N
            r_bp = wrap_delta(p(:,predIdx) - p(:,n), L); % vector from boid to predator
            rmag = max(norm(r_bp), 1e-9);
            v(:,n) = v(:,n) - pred_gain_repulse * (r_bp / (rmag^2));
        end

        % Predator FOV-based seeking toward nearby prey
        sumVec = [0;0]; countP = 0;
        for n = 1:N
            r_pb = wrap_delta(p(:,n) - p(:,predIdx), L); % vector from predator to boid
            rmag = max(norm(r_pb), 1e-9);
            if rmag <= pred_searchRadius
                theta = angle_between(v(:,predIdx), r_pb);
                if theta <= pred_searchAngle
                    sumVec = sumVec + r_pb;
                    countP = countP + 1;
                end
            end
        end
        if countP > 0
            pred_steer = pred_seek_gain * (sumVec / max(norm(sumVec), 1e-9));
        end
    end

    % --- Update velocities (boids)
    for n = 1:N
        v(:,n) = v(:,n) + v_coh(:,n) + v_sep(:,n) + v_align(:,n) + v_noise(:,n) + v_drift;
    end

    % --- Update predator velocity
    if use_predator
        v(:,predIdx) = v(:,predIdx) + pred_steer + pred_noise_gain * randn(2,1);
    end

    % --- Speed cap (all agents)
    for i = 1:numAgents
        s = norm(v(:,i));
        vmax = vlimit;
        if i == numAgents && use_predator
            vmax = pred_vlimit;
        end
        if s > vmax
            v(:,i) = v(:,i) / s * vmax;
        end
    end

    % --- Position update + wrap
    p = p + v * delta;
    p = wrap_position(p, L);

    % --- Plot
    cla(ax);                       % keep styling; just clear contents
    hold(ax,'on');
    
    % boids
    plot(ax, p(1,1:N), p(2,1:N), 'k.', 'MarkerSize', 10);
    
    % predator
    if use_predator
        plot(ax, p(1,predIdx), p(2,predIdx), 'r.', 'MarkerSize', 18);
    end
    
    % blue arrows for boids
    idx = 1:N;
    quiver(ax, p(1,idx), p(2,idx), v(1,idx), v(2,idx), 0.5, 'Color', [0 0.45 0.90]);
    
    % red arrow for predator
    if use_predator
        quiver(ax, p(1,predIdx), p(2,predIdx), v(1,predIdx), v(2,predIdx), 0.5, 'Color', [0.85 0 0]);
    end
    
    % limits + square, no grid/box/ticks
    set(ax,'XLim',[-limit limit],'YLim',[-limit limit]);
    axis(ax,'square');
    grid(ax,'off'); box(ax,'on');
    ax.XTick = []; ax.YTick = [];
    ax.XLabel.String = ''; ax.YLabel.String = '';
    ax.XColor = [0 0 0];  % axis line color
    ax.YColor = [0 0 0];
    ax.LineWidth = 1.0;   % thickness of the axis lines
    
    title(ax,'Schooling with Random Walk and Predator');
    drawnow;


    if record_video
        frame = getframe(fig);
        writeVideo(vw, frame);
    end

    hold off;

    pause(0.01); % slow down a touch for viewing
end

if exist('record_video','var') && record_video
    close(vw);
    fprintf("Saved: %s\n", outfile);
end

y_out = 0;

end
% ======================= HELPER FUNCTIONS ===============================

function X = normalize_columns(X)
% Normalize each column vector to unit length (leave zeros alone)
	n = vecnorm(X,2,1);
	n(n==0) = 1;
	X = X ./ n;
end

function r = wrap_delta(r, L)
% Wrap a displacement vector r into [-L/2, L/2] (torus metric)
	for d = 1:2
	    if r(d) >  L/2, r(d) = r(d) - L; end
	    if r(d) < -L/2, r(d) = r(d) + L; end
	end
end

function P = wrap_position(P, L)
% Wrap absolute positions columnwise into [-L/2, L/2]
	for d = 1:2
	    over  = P(d,:) >  L/2;
	    under = P(d,:) < -L/2;
	    P(d,over)  = P(d,over)  - L;
	    P(d,under) = P(d,under) + L;
	end
end

function a = angle_between(u, v)
% Angle (degrees) between 2D vectors u and v
	den = max(norm(u)*norm(v), 1e-12);
	c = max(min(dot(u,v)/den, 1), -1); % clamp for numerical safety
	a = acosd(c);
end

function out = ternary(cond, a, b)
% Simple ternary helper for titles
	if cond, out = a; else, out = b; end
end
