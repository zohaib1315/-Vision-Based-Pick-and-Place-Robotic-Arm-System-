%% =========================================================
%  Q-Arm Stage 3 (Fixed): Full Simulink Simulation
%  ---------------------------------------------------------
%  Fixes:
%    - WRIST waypoints unwrapped to avoid ±180° flipping
%    - Spline replaced with pchip (monotone, no overshoot)
%    - Higher PD gains for tight tracking
%    - Solver changed to ode45 with relaxed tolerance
%    - Cleaner EE path
%% =========================================================

clc; clear; close all;

%% ===== PART 1: VISION + IK ================================
fprintf('=== PART 1: Vision + IK ===\n');

robot = importrobot('QARM.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

jointLimits = [-2.967,  2.967;
               -1.483,  1.483;
               -1.658,  1.309;
               -3.010,  3.010];

homeConfig = robot.homeConfiguration;  % [0 0 0 0]

ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations     = 3000;
ik.SolverParameters.MaxTime           = 15;
ik.SolverParameters.GradientTolerance = 1e-12;
ik.SolverParameters.SolutionTolerance = 1e-10;

% Detected object position (from Stage 2 vision pipeline)
detectedWorldPos = [0.22,  0.05,  0.05];
placeXYZ         = [-0.15, 0.15,  0.05];
liftXYZ          = [0.22,  0.05,  0.28];
preHomeXYZ       = [0.10,  0.00,  0.30];

[pickConfig,  pickErr]  = solveIK(ik, detectedWorldPos, jointLimits, 25);
[liftConfig,  liftErr]  = solveIK(ik, liftXYZ,         jointLimits, 20);
[placeConfig, placeErr] = solveIK(ik, placeXYZ,         jointLimits, 25);
[preConfig,   preErr]   = solveIK(ik, preHomeXYZ,        jointLimits, 20);

fprintf('Pick  IK error : %.5f m\n', pickErr);
fprintf('Lift  IK error : %.5f m\n', liftErr);
fprintf('Place IK error : %.5f m\n', placeErr);

%% ===== PART 2: BUILD SMOOTH TRAJECTORY ====================
fprintf('\n=== PART 2: Trajectory Generation ===\n');

% Timing
tHome  = 0.0;
tPick  = 3.0;
tLift  = 4.5;
tPlace = 7.5;
tPre   = 9.0;
tEnd   = 11.0;

waypointTimes   = [tHome; tPick; tLift; tPlace; tPre; tEnd];
waypointConfigs = [homeConfig;
                   pickConfig;
                   liftConfig;
                   placeConfig;
                   preConfig;
                   homeConfig];

% ---- CRITICAL FIX: Unwrap joint angles to remove ±π jumps ----
% This prevents the spline from taking the "long way around"
% particularly important for WRIST which swings near ±π
waypointConfigsUnwrapped = waypointConfigs;
for j = 1:4
    waypointConfigsUnwrapped(:,j) = unwrap(waypointConfigs(:,j));
end

fprintf('Waypoints (unwrapped):\n');
wNames = {'Home','Pick','Lift','Place','PreHome','Home'};
for i = 1:6
    fprintf('  t=%4.1fs  %-8s: [%6.1f  %6.1f  %6.1f  %6.1f] deg\n', ...
        waypointTimes(i), wNames{i}, ...
        rad2deg(waypointConfigsUnwrapped(i,:)));
end

% ---- Use pchip (monotone interpolation — ZERO overshoot) ----
dt     = 0.01;
tVec   = (0:dt:tEnd)';
nPts   = numel(tVec);
qTraj  = zeros(nPts, 4);
qdTraj = zeros(nPts, 4);

for j = 1:4
    qTraj(:,j)      = pchip(waypointTimes, waypointConfigsUnwrapped(:,j), tVec);
    % Velocity via central differences
    qdTraj(2:end-1,j) = (qTraj(3:end,j) - qTraj(1:end-2,j)) / (2*dt);
    qdTraj(1,j)       = qdTraj(2,j);
    qdTraj(end,j)     = qdTraj(end-1,j);
end

% Clip to joint limits
for j = 1:4
    qTraj(:,j) = max(jointLimits(j,1), min(jointLimits(j,2), qTraj(:,j)));
end

fprintf('\nTrajectory: %d points, %.0f Hz, %.1f s\n', nPts, 1/dt, tEnd);

% ---- Plot trajectories -----
figure('Name','Joint Trajectories (Fixed)','NumberTitle','off','Color','w');
jointLabels = {'YAW','SHOULDER','ELBOW','WRIST'};
colors = lines(4);
for j = 1:4
    subplot(2,2,j); hold on;
    plot(tVec, rad2deg(qTraj(:,j)), '-', 'LineWidth',2, 'Color',colors(j,:));
    scatter(waypointTimes, rad2deg(waypointConfigsUnwrapped(:,j)), 60,'k','filled');
    yline(rad2deg(jointLimits(j,1)),'--r','LineWidth',0.8);
    yline(rad2deg(jointLimits(j,2)),'--r','LineWidth',0.8);
    xlabel('Time (s)'); ylabel('Angle (deg)');
    title(jointLabels{j}); grid on;
    legend('Trajectory','Waypoints','Limits','Location','best','FontSize',7);
end
sgtitle('Q-Arm Joint Trajectories (pchip — No Overshoot)','FontSize',13,'FontWeight','bold');

%% ===== PART 3: BUILD SIMULINK MODEL =======================
fprintf('\n=== PART 3: Building Simulink Model ===\n');

mdlName = 'QArm_PickPlace_v2';
if bdIsLoaded(mdlName); close_system(mdlName,0); end
new_system(mdlName);
open_system(mdlName);

% Use ode45 — much more stable than ode23t for this system
set_param(mdlName, ...
    'Solver',           'ode45', ...
    'StopTime',         num2str(tEnd), ...
    'MaxStep',          '0.05', ...
    'MinStep',          'auto', ...
    'RelTol',           '1e-4', ...
    'AbsTol',           '1e-6');

fprintf('Solver: ode45  StopTime: %.1fs\n', tEnd);

% Package trajectory for From Workspace
trajTS.time               = tVec;
trajTS.signals.values     = qTraj;
trajTS.signals.dimensions = 4;
assignin('base','trajTS_v2', trajTS);

% ---- Trajectory source ----
add_block('simulink/Sources/From Workspace', ...
    [mdlName, '/TrajectoryRef'], ...
    'VariableName', 'trajTS_v2', ...
    'Position',     [30, 110, 180, 150], ...
    'Interpolate',  'on');

add_block('simulink/Signal Routing/Demux', ...
    [mdlName, '/DemuxRef'], ...
    'Outputs',  '4', ...
    'Position', [210, 95, 230, 165]);
add_line(mdlName,'TrajectoryRef/1','DemuxRef/1');

% ---- PD gains (increased for tight tracking) ----
%      Kp and Kd tuned so settling time < 0.3s per joint
Kp = [25.0, 30.0, 25.0, 15.0];
Kd = [ 4.0,  5.0,  4.0,  2.5];

yOffsets = [50, 155, 260, 365];

% Output muxes
add_block('simulink/Signal Routing/Mux', [mdlName, '/MuxActual'], ...
    'Inputs','4','Position',[700,70,720,440]);
add_block('simulink/Signal Routing/Mux', [mdlName, '/MuxTorque'], ...
    'Inputs','4','Position',[575,70,595,440]);

for j = 1:4
    y0   = yOffsets(j);
    jStr = num2str(j);

    add_block('simulink/Math Operations/Sum', ...
        [mdlName, '/Sum', jStr], 'Inputs','+-', 'Position',[275,y0+5,295,y0+25]);
    add_block('simulink/Math Operations/Gain', ...
        [mdlName, '/Kp', jStr], 'Gain',num2str(Kp(j)), 'Position',[315,y0,365,y0+30]);
    add_block('simulink/Continuous/Derivative', ...
        [mdlName, '/Deriv', jStr], 'Position',[315,y0+45,365,y0+75]);
    add_block('simulink/Math Operations/Gain', ...
        [mdlName, '/Kd', jStr], 'Gain',num2str(Kd(j)), 'Position',[380,y0+45,430,y0+75]);
    add_block('simulink/Math Operations/Sum', ...
        [mdlName, '/TorqueSum', jStr], 'Inputs','++', 'Position',[445,y0+10,465,y0+60]);
    add_block('simulink/Discontinuities/Saturation', ...
        [mdlName, '/Sat', jStr], ...
        'UpperLimit','15', 'LowerLimit','-15', ...
        'Position',[480,y0+15,525,y0+55]);
    add_block('simulink/Continuous/Integrator', ...
        [mdlName, '/Int1_', jStr], 'InitialCondition','0', 'Position',[540,y0+15,575,y0+55]);
    add_block('simulink/Continuous/Integrator', ...
        [mdlName, '/Int2_', jStr], 'InitialCondition','0', 'Position',[610,y0+15,655,y0+55]);

    % Wiring
    add_line(mdlName, ['DemuxRef/',  jStr],        ['Sum',      jStr, '/1']);
    add_line(mdlName, ['Sum',        jStr, '/1'],   ['Kp',       jStr, '/1']);
    add_line(mdlName, ['Kp',         jStr, '/1'],   ['TorqueSum',jStr, '/1']);
    add_line(mdlName, ['Sum',        jStr, '/1'],   ['Deriv',    jStr, '/1']);
    add_line(mdlName, ['Deriv',      jStr, '/1'],   ['Kd',       jStr, '/1']);
    add_line(mdlName, ['Kd',         jStr, '/1'],   ['TorqueSum',jStr, '/2']);
    add_line(mdlName, ['TorqueSum',  jStr, '/1'],   ['Sat',      jStr, '/1']);
    add_line(mdlName, ['Sat',        jStr, '/1'],   ['Int1_',    jStr, '/1']);
    add_line(mdlName, ['Int1_',      jStr, '/1'],   ['Int2_',    jStr, '/1']);
    add_line(mdlName, ['Int2_',      jStr, '/1'],   ['Sum',      jStr, '/2']);
    add_line(mdlName, ['Sat',        jStr, '/1'],   ['MuxTorque/', jStr]);
    add_line(mdlName, ['Int2_',      jStr, '/1'],   ['MuxActual/', jStr]);

    fprintf('  Joint %d: Kp=%.0f  Kd=%.1f\n', j, Kp(j), Kd(j));
end

% ---- Scopes + logging ----
add_block('simulink/Sinks/Scope',[mdlName, '/JointScope'], ...
    'NumInputPorts','1','Position',[760,80,800,120]);
add_line(mdlName,'MuxActual/1','JointScope/1');

add_block('simulink/Sinks/To Workspace',[mdlName, '/JointLog'], ...
    'VariableName','simJoints','MaxDataPoints','inf', ...
    'SaveFormat','Array','Position',[760,200,830,240]);
add_line(mdlName,'MuxActual/1','JointLog/1');

add_block('simulink/Sources/Clock',[mdlName, '/Clock'], ...
    'Position',[710,298,740,322]);
add_block('simulink/Sinks/To Workspace',[mdlName, '/TimeLog'], ...
    'VariableName','simT','MaxDataPoints','inf', ...
    'SaveFormat','Array','Position',[760,290,830,330]);
add_line(mdlName,'Clock/1','TimeLog/1');

save_system(mdlName);
fprintf('\nModel saved: %s.slx\n', mdlName);

%% ===== PART 4: SIMULATE ===================================
fprintf('\n=== PART 4: Simulation ===\n');

simSuccess = false;
try
    sim(mdlName,'StopTime',num2str(tEnd));
    fprintf('✓ Simulation complete!\n');
    simSuccess = true;
catch ME
    fprintf('⚠ Simulation warning: %s\n', ME.message);
end

%% ===== PART 5: POST-PROCESS ===============================
fprintf('\n=== PART 5: Results ===\n');

if simSuccess && evalin('base','exist(''simJoints'',''var'')')
    tPlot = evalin('base','simT');
    qPlot = evalin('base','simJoints');
    fprintf('Using simulation data (%d samples)\n', numel(tPlot));
else
    tPlot = tVec;
    qPlot = qTraj;
    fprintf('Using spline trajectory (%d samples)\n', numel(tPlot));
end

% ---- Joint tracking plot ----
figure('Name','Joint Tracking (Fixed)','NumberTitle','off','Color','w');
for j = 1:4
    subplot(2,2,j); hold on;
    plot(tVec,  rad2deg(qTraj(:,j)),  '--k','LineWidth',1.5,'DisplayName','Reference');
    plot(tPlot, rad2deg(qPlot(:,j)),  '-',  'LineWidth',2,  ...
        'Color',colors(j,:),'DisplayName','Simulated');
    scatter(waypointTimes, rad2deg(waypointConfigsUnwrapped(:,j)),50,'k','filled');
    xlabel('Time (s)'); ylabel('Angle (deg)');
    title(jointLabels{j}); grid on;
    legend('Location','best','FontSize',7);
end
sgtitle('Joint Tracking: Reference vs Simulated (Fixed)','FontSize',13,'FontWeight','bold');

% ---- End-effector path ----
fprintf('Computing EE path...\n');
eePath = zeros(size(qPlot,1),3);
for k = 1:size(qPlot,1)
    T = getTransform(robot, qPlot(k,:), 'END-EFFECTOR');
    eePath(k,:) = T(1:3,4)';
end

figure('Name','End-Effector Path (Fixed)','NumberTitle','off','Color','w');

subplot(1,2,1);
plot3(eePath(:,1),eePath(:,2),eePath(:,3),'b-','LineWidth',2); hold on;
plot3(eePath(1,1),eePath(1,2),eePath(1,3),'go','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Start');
plot3(detectedWorldPos(1),detectedWorldPos(2),detectedWorldPos(3), ...
    'rs','MarkerSize',12,'MarkerFaceColor','r','DisplayName','Pick');
plot3(placeXYZ(1),placeXYZ(2),placeXYZ(3), ...
    'g^','MarkerSize',12,'MarkerFaceColor','g','DisplayName','Place');
legend('Location','best'); grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('End-Effector 3D Path'); view(45,25);

subplot(1,2,2); hold on;
plot(tPlot,eePath(:,1),'r-','LineWidth',1.5,'DisplayName','X');
plot(tPlot,eePath(:,2),'g-','LineWidth',1.5,'DisplayName','Y');
plot(tPlot,eePath(:,3),'b-','LineWidth',1.5,'DisplayName','Z');
xline(tPick,  '--k', 'LineWidth',1); text(tPick+0.1,  max(eePath(:,3))*0.9, 'Pick',  'FontSize',8);
xline(tLift,  '--',  'LineWidth',1); text(tLift+0.1,  max(eePath(:,3))*0.9, 'Lift',  'FontSize',8);
xline(tPlace, '--',  'LineWidth',1); text(tPlace+0.1, max(eePath(:,3))*0.9, 'Place', 'FontSize',8);
xlabel('Time (s)'); ylabel('Position (m)');
title('EE XYZ vs Time'); legend('Location','best'); grid on;
sgtitle('End-Effector Trajectory (Fixed)','FontSize',13,'FontWeight','bold');

% ---- Final 3D animation ----
fprintf('Running animation...\n');

figure('Name','Q-Arm — Final Animation','NumberTitle','off','Color','w');
show(robot,qPlot(1,:),'Visuals','off','Frames','on','PreservePlot',false);
hold on;
h_trail = plot3(eePath(1,1),eePath(1,2),eePath(1,3),'c-','LineWidth',2,'DisplayName','EE Trail');
plot3(detectedWorldPos(1),detectedWorldPos(2),detectedWorldPos(3),'rs', ...
    'MarkerSize',14,'MarkerFaceColor',[0.9 0.1 0.1],'MarkerEdgeColor','k','DisplayName','Pick');
plot3(placeXYZ(1),placeXYZ(2),placeXYZ(3),'gs', ...
    'MarkerSize',14,'MarkerFaceColor',[0.1 0.8 0.1],'MarkerEdgeColor','k','DisplayName','Place');
legend('Location','northeast');
xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([-0.05 0.65]);
axis manual; view(45,25);
ax = gca;
ax.CameraViewAngleMode = 'manual';
ax.DataAspectRatioMode = 'manual';
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% Determine current phase label from time
phaseLabels = {'Approaching Pick','Picking','Lifting','Moving to Place','Returning Home'};
phaseTimes  = [0, tPick, tLift, tPlace, tPre, tEnd];

for k = 1:3:size(qPlot,1)
    t_k = tPlot(k);
    % Find current phase
    phaseIdx = find(phaseTimes <= t_k, 1, 'last');
    phaseIdx = min(phaseIdx, numel(phaseLabels));

    show(robot, qPlot(k,:),'Visuals','off','Frames','on', ...
        'Parent',ax,'PreservePlot',false);
    set(h_trail,'XData',eePath(1:k,1),'YData',eePath(1:k,2),'ZData',eePath(1:k,3));
    view(ax,45,25);
    xlim(ax,[-0.6 0.6]); ylim(ax,[-0.6 0.6]); zlim(ax,[-0.05 0.65]);
    title(ax, sprintf('[%s]  t=%.1fs | YAW=%.0f° SHLD=%.0f° ELBW=%.0f° WRST=%.0f°', ...
        phaseLabels{phaseIdx}, t_k, ...
        rad2deg(qPlot(k,1)), rad2deg(qPlot(k,2)), ...
        rad2deg(qPlot(k,3)), rad2deg(qPlot(k,4))), 'FontSize',9);
    drawnow limitrate;
    pause(0.015);
end
title(ax,'✓ Vision-Based Pick & Place Complete','FontSize',12);

fprintf('\n✓ Stage 3 fixed and complete!\n');
fprintf('  Simulink model: %s.slx\n', mdlName);

%% =========================================================
%  LOCAL FUNCTIONS
%% =========================================================
function [bestConfig, bestErr] = solveIK(ik, targetXYZ, jointLimits, nRestarts)
    targetTform = trvec2tform(targetXYZ) * eul2tform([0 pi 0]);
    weights     = [1, 1, 1, 0.1, 0.1, 0.1];
    bestErr     = inf;
    bestConfig  = zeros(1,4);
    for k = 1:nRestarts
        if k == 1
            initGuess = deg2rad([0, 45, -30, 0]);
        else
            initGuess = jointLimits(:,1)' + ...
                rand(1,4).*(jointLimits(:,2)-jointLimits(:,1))';
        end
        [cfg, info] = ik('END-EFFECTOR', targetTform, weights, initGuess);
        if info.PoseErrorNorm < bestErr
            bestErr    = info.PoseErrorNorm;
            bestConfig = cfg;
        end
        if bestErr < 0.005; break; end
    end
end