clear; clc; close all;

%% ============================================================
%  MATLAB - RYSUNKI TEORETYCZNE SLAM Z ROSBAGA
%  Plik generuje czytelne rysunki do pracy magisterskiej:
%  skan LiDAR, chmura punktów, budowa mapy, trajektoria,
%  pose graph, schematyczne submapy, loop closure,
%  porównanie bez/z loop closure.
%% ============================================================

%% === WYGLĄD WYKRESÓW ===

set(groot, "defaultFigureColor", "w");
set(groot, "defaultAxesColor", "w");
set(groot, "defaultAxesFontSize", 13);
set(groot, "defaultAxesLineWidth", 1.1);
set(groot, "defaultLineLineWidth", 1.4);
set(groot, "defaultAxesGridAlpha", 0.25);
set(groot, "defaultTextInterpreter", "none");
set(groot, "defaultLegendInterpreter", "none");
set(groot, "defaultAxesTickLabelInterpreter", "none");

%% === USTAWIENIA GŁÓWNE ===

bagPath = "/home/jaokoz/ros2_ws/rosbag2_cartografer_sim_proba_1000";
scanTopic = "/scan";

outDir = "/home/jaokoz/ros_scripts/wyniki/rosbag2_cartografer_sim_proba_1000";

if ~exist(outDir, "dir")
    mkdir(outDir);
end

%% === PARAMETRY LiDAR / SLAM ===

maxRange = 8.0;              % [m]
minRange = 0.05;             % [m]

mapResolution = 20;          % [komórki/m]
scanStep = 5;                % używamy co 5. skanu

loopClosureThreshold = 210;
loopClosureSearchRadius = 8; % [m]

%% === WCZYTANIE ROSBAGA ===

disp("==========================================");
disp("WCZYTYWANIE ROSBAGA");
disp("==========================================");

bag = ros2bagreader(bagPath);

disp("Dostępne topiki:");
disp(bag.AvailableTopics);

topicsFile = fullfile(outDir, "available_topics.txt");
writetable(bag.AvailableTopics, topicsFile);

%% === WCZYTANIE SKANÓW LiDAR ===

disp("==========================================");
disp("WCZYTYWANIE SKANÓW LiDAR");
disp("==========================================");

scanBag = select(bag, "Topic", scanTopic);
scanMsgs = readMessages(scanBag);

nScans = numel(scanMsgs);
fprintf("Liczba wiadomości /scan: %d\n", nScans);

if nScans == 0
    error("Brak wiadomości na topiku /scan.");
end

getField = @(msg, name1, name2) localGetField(msg, name1, name2);

%% ============================================================
%  RYSUNEK 1: POJEDYNCZY SKAN LiDAR
%% ============================================================

disp("Rysunek 1: pojedynczy skan LiDAR");

msg = scanMsgs{1};

[ranges, angles, valid] = localReadScan(msg, getField, minRange, maxRange);

scan1 = lidarScan(ranges(valid), angles(valid));

fig1 = figure("Name", "Pojedynczy skan LiDAR", "Color", "w");
plot(scan1);
axis equal;
grid on;
xlabel("x_L [m]");
ylabel("y_L [m]");
title("Pojedynczy skan LiDAR w lokalnym układzie czujnika");
localSaveFigure(fig1, outDir, "01_pojedynczy_skan_lidar");

%% ============================================================
%  RYSUNEK 2: SKAN JAKO CHMURA PUNKTÓW XY
%% ============================================================

disp("Rysunek 2: skan jako punkty XY");

x = ranges(valid) .* cos(angles(valid));
y = ranges(valid) .* sin(angles(valid));

fig2 = figure("Name", "Skan jako punkty XY", "Color", "w");
scatter(x, y, 10, "filled");
axis equal;
grid on;
xlabel("x_L [m]");
ylabel("y_L [m]");
title("Reprezentacja skanu LiDAR jako chmury punktów 2D");
localSaveFigure(fig2, outDir, "02_skan_jako_punkty_xy");

Tscan = table( ...
    x(:), ...
    y(:), ...
    ranges(valid), ...
    angles(valid), ...
    'VariableNames', ["x_m", "y_m", "range_m", "angle_rad"] ...
);

writetable(Tscan, fullfile(outDir, "02_punkty_pierwszego_skanu.csv"));

%% ============================================================
%  RYSUNEK 3: KILKA KOLEJNYCH SKANÓW
%% ============================================================

disp("Rysunek 3: kilka kolejnych skanów");

fig3 = figure("Name", "Kilka kolejnych skanów", "Color", "w");
hold on;
grid on;
axis equal;

exampleIdx = unique(round(linspace(1, min(nScans, 200), 5)));

for k = 1:numel(exampleIdx)
    msg = scanMsgs{exampleIdx(k)};
    [ranges, angles, valid] = localReadScan(msg, getField, minRange, maxRange);

    x = ranges(valid) .* cos(angles(valid));
    y = ranges(valid) .* sin(angles(valid));

    scatter(x, y, 8, "filled");
end

xlabel("x_L [m]");
ylabel("y_L [m]");
title("Kilka kolejnych skanów LiDAR w lokalnym układzie czujnika");
legend("skan 1", "skan 2", "skan 3", "skan 4", "skan 5", "Location", "best");
localSaveFigure(fig3, outDir, "03_kilka_kolejnych_skanow");

%% ============================================================
%  BUDOWA OBIEKTU lidarSLAM
%% ============================================================

disp("==========================================");
disp("BUDOWA MATLAB lidarSLAM");
disp("==========================================");

slamObj = lidarSLAM(mapResolution, maxRange);

slamObj.LoopClosureThreshold = loopClosureThreshold;
slamObj.LoopClosureSearchRadius = loopClosureSearchRadius;

acceptedScans = 0;
loopClosureScanIndices = [];

snapshotCounter = 1;
snapshotScans = round(linspace(1, nScans, 4));

for i = 1:scanStep:nScans

    msg = scanMsgs{i};
    [ranges, angles, valid] = localReadScan(msg, getField, minRange, maxRange);

    if sum(valid) < 20
        continue;
    end

    scan = lidarScan(ranges(valid), angles(valid));

    [isAccepted, ~, optimizationInfo] = addScan(slamObj, scan);

    if isAccepted
        acceptedScans = acceptedScans + 1;
    end

    if optimizationInfo.IsPerformed
        fprintf("Loop closure / optymalizacja przy skanie: %d\n", i);
        loopClosureScanIndices = [loopClosureScanIndices; i]; %#ok<AGROW>
    end

    if snapshotCounter <= numel(snapshotScans)
        if i >= snapshotScans(snapshotCounter)

            figSnap = figure("Name", "Etap budowy mapy", "Color", "w");
            show(slamObj);
            axis equal;
            grid on;
            xlabel("x [m]");
            ylabel("y [m]");
            title("Narastanie mapy SLAM - etap " + string(snapshotCounter));

            filename = sprintf("04_budowa_mapy_etap_%d", snapshotCounter);
            localSaveFigure(figSnap, outDir, filename);

            snapshotCounter = snapshotCounter + 1;
        end
    end
end

fprintf("Zaakceptowane skany: %d\n", acceptedScans);
fprintf("Liczba optymalizacji / loop closures: %d\n", numel(loopClosureScanIndices));

%% ============================================================
%  RYSUNEK 4: KOŃCOWY WYNIK SLAM
%% ============================================================

fig4 = figure("Name", "Końcowy wynik SLAM", "Color", "w");
show(slamObj);
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Końcowy wynik algorytmu lidarSLAM");
localSaveFigure(fig4, outDir, "05_koncowy_wynik_slam");

%% ============================================================
%  RYSUNEK 5: TRAJEKTORIA
%% ============================================================

pg = slamObj.PoseGraph;
poses = nodes(pg);

fig5 = figure("Name", "Trajektoria robota", "Color", "w");
plot(poses(:,1), poses(:,2), "-o", "MarkerSize", 3);
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Trajektoria estymowana na podstawie dopasowania skanów");
localSaveFigure(fig5, outDir, "06_trajektoria_estymowana");

Tposes = array2table(poses, "VariableNames", ["x_m", "y_m", "theta_rad"]);
writetable(Tposes, fullfile(outDir, "06_trajektoria_estymowana.csv"));

%% ============================================================
%  RYSUNEK 6: POSE GRAPH
%% ============================================================

fig6 = figure("Name", "Pose graph", "Color", "w");
show(pg);
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Graf pozy: węzły trajektorii i ograniczenia między pozami");
localSaveFigure(fig6, outDir, "07_pose_graph");


%% ============================================================
%  RYSUNEK 6A: IDEA SCAN-TO-SUBMAP
%% ============================================================

disp("Rysunek 6A: idea scan-to-submap");

figSub = figure("Name", "Idea scan-to-submap", "Color", "w");
tiledlayout(1, 3, "Padding", "compact", "TileSpacing", "compact");

% ============================================================
% Dane poglądowe - lokalna submapa i bieżący skan
% ============================================================

% Ściany / przeszkody jako prosta geometria środowiska
wall1 = [-3 -2; -3 2; -2.6 2; -2.6 -2];
wall2 = [-2.8 1.6; 1.5 1.6; 1.5 1.25; -2.8 1.25];
wall3 = [1.2 -1.8; 1.6 -1.8; 1.6 1.6; 1.2 1.6];
wall4 = [-2.8 -1.7; -0.6 -1.7; -0.6 -1.35; -2.8 -1.35];
obst  = [-0.2 -0.4; 0.5 -0.4; 0.5 0.4; -0.2 0.4];

walls = {wall1, wall2, wall3, wall4, obst};

% Punkty aktywnej submapy - zaszumione próbki z kilku wcześniejszych skanów
rng(4);
submapPts = [];

for w = 1:numel(walls)
    poly = walls{w};
    polyClosed = [poly; poly(1,:)];

    for e = 1:size(polyClosed,1)-1
        p1 = polyClosed(e,:);
        p2 = polyClosed(e+1,:);
        n = 35;

        a = linspace(0, 1, n)';
        pts = p1 + a .* (p2 - p1);
        pts = pts + 0.025 * randn(size(pts));

        submapPts = [submapPts; pts]; %#ok<AGROW>
    end
end

% Bieżący skan odpowiada fragmentowi submapy, ale jest przesunięty i obrócony
currentIdx = submapPts(:,1) > -2.7 & submapPts(:,1) < 1.4 & ...
             submapPts(:,2) > -1.6 & submapPts(:,2) < 1.5;

currentScanTrue = submapPts(currentIdx,:);
currentScanTrue = currentScanTrue(1:4:end,:);

% Błąd pozy początkowej
dtheta = deg2rad(10);
Rerr = [cos(dtheta) -sin(dtheta); sin(dtheta) cos(dtheta)];
terr = [0.45 -0.35];

currentScanInitial = (Rerr * currentScanTrue')' + terr;

% Po dopasowaniu skan wraca w pobliże aktywnej submapy
currentScanMatched = currentScanTrue + 0.015 * randn(size(currentScanTrue));

% Pozycje robota
robotInitial = [0.0 -1.0];
robotWrong   = robotInitial + terr;
robotMatched = robotInitial;

% ============================================================
% Panel 1 - aktywna submapa
% ============================================================

nexttile;
hold on; grid on; axis equal;

for w = 1:numel(walls)
    poly = walls{w};
    fill(poly(:,1), poly(:,2), [0.82 0.82 0.82], ...
        "EdgeColor", [0.25 0.25 0.25], ...
        "LineWidth", 1.2);
end

scatter(submapPts(:,1), submapPts(:,2), 8, ...
    [0.25 0.25 0.25], "filled");

plot(robotInitial(1), robotInitial(2), "ko", ...
    "MarkerFaceColor", "w", ...
    "MarkerSize", 8, ...
    "LineWidth", 1.5);

xlabel("x [m]");
ylabel("y [m]");
title("1. Aktywna submapa");
xlim([-3.4 2.0]);
ylim([-2.2 2.3]);

text(-3.2, 2.05, "submapa z kilku wcześniejszych skanów", ...
    "FontSize", 10);

% ============================================================
% Panel 2 - bieżący skan przed dopasowaniem
% ============================================================

nexttile;
hold on; grid on; axis equal;

for w = 1:numel(walls)
    poly = walls{w};
    fill(poly(:,1), poly(:,2), [0.90 0.90 0.90], ...
        "EdgeColor", [0.65 0.65 0.65], ...
        "LineWidth", 1.0);
end

scatter(submapPts(:,1), submapPts(:,2), 6, ...
    [0.70 0.70 0.70], "filled");

scatter(currentScanInitial(:,1), currentScanInitial(:,2), 28, ...
    "r", "filled");

plot(robotWrong(1), robotWrong(2), "ro", ...
    "MarkerFaceColor", "w", ...
    "MarkerSize", 9, ...
    "LineWidth", 1.7);

quiver(robotWrong(1), robotWrong(2), 0.35*cos(dtheta), 0.35*sin(dtheta), ...
    0, "r", "LineWidth", 1.6, "MaxHeadSize", 1.4);

xlabel("x [m]");
ylabel("y [m]");
title("2. Skan przed dopasowaniem");
xlim([-3.4 2.0]);
ylim([-2.2 2.3]);

text(-3.2, 2.05, "pozycja początkowa jest niedokładna", ...
    "FontSize", 10, ...
    "Color", "r");

% ============================================================
% Panel 3 - skan po dopasowaniu do submapy
% ============================================================

nexttile;
hold on; grid on; axis equal;

for w = 1:numel(walls)
    poly = walls{w};
    fill(poly(:,1), poly(:,2), [0.86 0.86 0.86], ...
        "EdgeColor", [0.35 0.35 0.35], ...
        "LineWidth", 1.2);
end

scatter(submapPts(:,1), submapPts(:,2), 8, ...
    [0.35 0.35 0.35], "filled");

scatter(currentScanMatched(:,1), currentScanMatched(:,2), 28, ...
    [0.0 0.55 0.0], "filled");

plot(robotMatched(1), robotMatched(2), "o", ...
    "Color", [0.0 0.45 0.0], ...
    "MarkerFaceColor", "w", ...
    "MarkerSize", 9, ...
    "LineWidth", 1.7);

quiver(robotMatched(1), robotMatched(2), 0.35, 0.0, ...
    0, ...
    "Color", [0.0 0.45 0.0], ...
    "LineWidth", 1.6, ...
    "MaxHeadSize", 1.4);

xlabel("x [m]");
ylabel("y [m]");
title("3. Dopasowanie scan-to-submap");
xlim([-3.4 2.0]);
ylim([-2.2 2.3]);

text(-3.2, 2.05, "skan dopasowany do aktywnej submapy", ...
    "FontSize", 10, ...
    "Color", [0.0 0.45 0.0]);

% ============================================================
% Wspólna legenda
% ============================================================

lgd = legend( ...
    "przeszkody / ściany", ...
    "punkty aktywnej submapy", ...
    "poza robota", ...
    "Location", "southoutside");

lgd.Layout.Tile = "south";

sgtitle("Idea dopasowania scan-to-submap", "FontWeight", "bold");

localSaveFigure(figSub, outDir, "07a_idea_scan_to_submap");

%% ============================================================
%  RYSUNEK 6A: SCHEMATYCZNE SUBMAPY
%% ============================================================

disp("Rysunek 6A: schematyczne submapy");

submapRows = [];

if exist("poses", "var") && size(poses,1) >= 10

    figSub = figure("Name", "Schematyczne submapy", "Color", "w");
    hold on;
    grid on;
    axis equal;

    % Cała trajektoria jako tło
    plot(poses(:,1), poses(:,2), "-", ...
        "Color", [0.15 0.15 0.15], ...
        "LineWidth", 1.4);

    xlabel("x [m]");
    ylabel("y [m]");
    title("Schematyczna ilustracja tworzenia submap");

    % ============================================================
    % Parametry symbolicznych submap
    % ============================================================
    % submapLength  - ile węzłów trajektorii tworzy jedną submapę
    % submapOverlap - ile węzłów nakłada się między kolejnymi submapami
    % submapSize    - symboliczny rozmiar rysowanej submapy [m]
    %
    % To nie są prawdziwe submapy Cartographera. To jest rysunek
    % poglądowy pokazujący zasadę: kolejne fragmenty trajektorii
    % tworzą lokalne, częściowo nakładające się submapy.
    % ============================================================

    submapLength = 35;
    submapOverlap = 12;
    submapSize = 1.2;

    colors = lines(12);
    submapId = 1;
    startIdx = 1;

    while startIdx < size(poses,1)

        endIdx = min(startIdx + submapLength - 1, size(poses,1));
        idx = startIdx:endIdx;

        if numel(idx) < 5
            break;
        end

        xs = poses(idx,1);
        ys = poses(idx,2);

        % Środek submapy jako średnia pozycji w danym fragmencie trajektorii
        cx = mean(xs);
        cy = mean(ys);

        % Orientacja submapy jako orientacja środkowej pozy
        midIdx = idx(round(numel(idx)/2));
        theta = poses(midIdx,3);

        c = colors(mod(submapId-1, size(colors,1)) + 1, :);

        % Fragment trajektorii należący do danej submapy
        plot(xs, ys, "-", ...
            "Color", c, ...
            "LineWidth", 2.4);

        % Symboliczny obszar submapy jako obrócony kwadrat
        halfSize = submapSize / 2;

        cornersLocal = [
            -halfSize, -halfSize;
             halfSize, -halfSize;
             halfSize,  halfSize;
            -halfSize,  halfSize;
            -halfSize, -halfSize
        ];

        R = [
            cos(theta), -sin(theta);
            sin(theta),  cos(theta)
        ];

        cornersGlobal = (R * cornersLocal')';
        cornersGlobal(:,1) = cornersGlobal(:,1) + cx;
        cornersGlobal(:,2) = cornersGlobal(:,2) + cy;

        plot(cornersGlobal(:,1), cornersGlobal(:,2), "--", ...
            "Color", c, ...
            "LineWidth", 1.7);

        % Środek submapy
        scatter(cx, cy, 75, c, "filled");

        % Orientacja submapy
        arrowLength = 0.45;
        quiver(cx, cy, ...
            arrowLength*cos(theta), ...
            arrowLength*sin(theta), ...
            0, ...
            "Color", c, ...
            "LineWidth", 1.6, ...
            "MaxHeadSize", 1.5);

        text(cx, cy, "S" + string(submapId), ...
            "FontSize", 11, ...
            "HorizontalAlignment", "center", ...
            "VerticalAlignment", "bottom", ...
            "Color", c);

        submapRows = [submapRows; ...
            submapId, startIdx, endIdx, cx, cy, theta]; %#ok<AGROW>

        submapId = submapId + 1;

        % Kolejna submapa zaczyna się wcześniej niż kończy poprzednia,
        % dzięki czemu widoczne jest nakładanie submap.
        startIdx = startIdx + submapLength - submapOverlap;
    end

    legend( ...
        "cała trajektoria", ...
        "fragment trajektorii submapy", ...
        "symboliczny obszar submapy", ...
        "środek submapy", ...
        "orientacja submapy", ...
        "Location", "bestoutside" ...
    );

    localSaveFigure(figSub, outDir, "07a_schematyczne_submapy");

    if ~isempty(submapRows)
        Tsubmaps = array2table(submapRows, ...
            "VariableNames", [ ...
                "submap_id", ...
                "start_node", ...
                "end_node", ...
                "center_x_m", ...
                "center_y_m", ...
                "theta_rad" ...
            ]);

        writetable(Tsubmaps, fullfile(outDir, "07a_schematyczne_submapy.csv"));
    end

else
    disp("Za mało pozycji w pose graph, aby narysować schematyczne submapy.");
end

%% ============================================================
%  ANALIZA KRAWĘDZI LOOP CLOSURE
%% ============================================================

edges = edgeNodePairs(pg);
loopEdges = [];

if isempty(edges)
    warning("Brak krawędzi w pose graph.");
else
    nonSequential = abs(edges(:,2) - edges(:,1)) > 1;
    loopEdges = edges(nonSequential, :);

    E = array2table(edges, "VariableNames", ["node_from", "node_to"]);
    writetable(E, fullfile(outDir, "08_pose_graph_edges.csv"));

    L = array2table(loopEdges, "VariableNames", ["node_from", "node_to"]);
    writetable(L, fullfile(outDir, "09_loop_closure_edges.csv"));

    fprintf("Liczba wszystkich krawędzi: %d\n", size(edges,1));
    fprintf("Liczba potencjalnych loop closure edges: %d\n", size(loopEdges,1));
end

%% ============================================================
%  RYSUNEK 7: LOOP CLOSURE NA TRAJEKTORII
%% ============================================================

fig7 = figure("Name", "Loop closure na trajektorii", "Color", "w");
plot(poses(:,1), poses(:,2), "-o", "MarkerSize", 3);
hold on;
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Trajektoria z zaznaczonymi krawędziami domknięcia pętli");

if ~isempty(loopEdges)
    for k = 1:size(loopEdges,1)
        id1 = loopEdges(k,1);
        id2 = loopEdges(k,2);

        p1 = poses(id1, 1:2);
        p2 = poses(id2, 1:2);

        plot([p1(1), p2(1)], [p1(2), p2(2)], "--", "LineWidth", 1.7);
    end

    legend("trajektoria", "krawędź loop closure", "Location", "best");
else
    text(mean(poses(:,1)), mean(poses(:,2)), ...
        "Brak wykrytych krawędzi loop closure", ...
        "HorizontalAlignment", "center", ...
        "FontSize", 13);
end

localSaveFigure(fig7, outDir, "10_trajektoria_z_loop_closure");

%% ============================================================
%  RYSUNEK 8: BEZ LOOP CLOSURE VS Z LOOP CLOSURE
%% ============================================================

slamNoLC = copy(slamObj);
removeLoopClosures(slamNoLC);

pgNoLC = slamNoLC.PoseGraph;
posesNoLC = nodes(pgNoLC);

fig8 = figure("Name", "Porównanie bez i z loop closure", "Color", "w");
plot(posesNoLC(:,1), posesNoLC(:,2), "-o", "MarkerSize", 3);
hold on;
plot(poses(:,1), poses(:,2), "-x", "MarkerSize", 4);
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Porównanie trajektorii: bez i z domknięciem pętli");
legend("bez loop closure", "po loop closure", "Location", "best");
localSaveFigure(fig8, outDir, "11_porownanie_bez_i_z_loop_closure");

%% ============================================================
%  RYSUNEK 9: MAPA BEZ LOOP CLOSURE
%% ============================================================

fig9 = figure("Name", "SLAM bez loop closure", "Color", "w");
show(slamNoLC);
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Wynik SLAM po usunięciu domknięć pętli");
localSaveFigure(fig9, outDir, "12_slam_bez_loop_closure");

%% ============================================================
%  RYSUNEK 10: MAPA PO LOOP CLOSURE
%% ============================================================

fig10 = figure("Name", "SLAM po loop closure", "Color", "w");
show(slamObj);
axis equal;
grid on;
xlabel("x [m]");
ylabel("y [m]");
title("Wynik SLAM po domknięciach pętli");
localSaveFigure(fig10, outDir, "13_slam_po_loop_closure");

%% ============================================================
%  RYSUNEK 11: ORIENTACJA ROBOTA theta
%% ============================================================

fig11 = figure("Name", "Orientacja robota", "Color", "w");
plot(poses(:,3));
grid on;
xlabel("Numer węzła grafu pozy [-]");
ylabel("Orientacja theta [rad]");
title("Zmiana orientacji robota w grafie pozy");
localSaveFigure(fig11, outDir, "14_orientacja_theta");

%% ============================================================
%  RYSUNEK 12: ODLEGŁOŚĆ MIĘDZY KOLEJNYMI POZAMI
%% ============================================================

if size(poses,1) > 1

    dxy = diff(poses(:,1:2));
    stepDistance = sqrt(sum(dxy.^2, 2));

    fig12 = figure("Name", "Przemieszczenia między pozami", "Color", "w");
    plot(stepDistance);
    grid on;
    xlabel("Numer przejścia między węzłami [-]");
    ylabel("Przemieszczenie między pozami [m]");
    title("Odległość między kolejnymi pozycjami w grafie");
    localSaveFigure(fig12, outDir, "15_przemieszczenia_miedzy_pozami");

    Tdist = table((1:numel(stepDistance)).', stepDistance(:), ...
        'VariableNames', ["edge_index", "distance_m"]);

    writetable(Tdist, fullfile(outDir, "15_przemieszczenia_miedzy_pozami.csv"));
end

%% ============================================================
%  PODSUMOWANIE
%% ============================================================

summaryFile = fullfile(outDir, "summary.txt");

fid = fopen(summaryFile, "w");

fprintf(fid, "MATLAB - analiza teoretyczna SLAM z rosbaga\n");
fprintf(fid, "==========================================\n\n");
fprintf(fid, "Rosbag: %s\n", bagPath);
fprintf(fid, "Topik LiDAR: %s\n", scanTopic);
fprintf(fid, "Liczba skanów: %d\n", nScans);
fprintf(fid, "Krok skanów użyty do SLAM: %d\n", scanStep);
fprintf(fid, "Zaakceptowane skany: %d\n", acceptedScans);
fprintf(fid, "Map resolution: %.2f cells/m\n", mapResolution);
fprintf(fid, "Max range: %.2f m\n", maxRange);
fprintf(fid, "LoopClosureThreshold: %.2f\n", loopClosureThreshold);
fprintf(fid, "LoopClosureSearchRadius: %.2f m\n", loopClosureSearchRadius);
fprintf(fid, "Liczba węzłów pose graph: %d\n", size(poses,1));
fprintf(fid, "Liczba krawędzi pose graph: %d\n", size(edges,1));
fprintf(fid, "Liczba potencjalnych loop closure edges: %d\n", size(loopEdges,1));

if exist("submapRows", "var")
    fprintf(fid, "Liczba schematycznych submap: %d\n", size(submapRows,1));
end

fclose(fid);

disp("==========================================");
disp("GOTOWE");
disp("Wyniki zapisano w:");
disp(outDir);
disp("==========================================");

%% ============================================================
%  FUNKCJE LOKALNE
%% ============================================================

function [ranges, angles, valid] = localReadScan(msg, getField, minRange, maxRange)

    ranges = double(getField(msg, "ranges", "Ranges"));
    angleMin = double(getField(msg, "angle_min", "AngleMin"));
    angleInc = double(getField(msg, "angle_increment", "AngleIncrement"));

    ranges = ranges(:);
    angles = angleMin + (0:numel(ranges)-1)' * angleInc;
    angles = angles(:);

    valid = isfinite(ranges) & ranges > minRange & ranges < maxRange;
    valid = valid(:);
end

function value = localGetField(msg, lowerName, upperName)

    if isstruct(msg)
        if isfield(msg, lowerName)
            value = msg.(lowerName);
        elseif isfield(msg, upperName)
            value = msg.(upperName);
        else
            error("Nie znaleziono pola: %s ani %s", lowerName, upperName);
        end
    else
        if isprop(msg, lowerName)
            value = msg.(lowerName);
        elseif isprop(msg, upperName)
            value = msg.(upperName);
        else
            try
                value = msg.(lowerName);
            catch
                try
                    value = msg.(upperName);
                catch
                    error("Nie znaleziono pola: %s ani %s", lowerName, upperName);
                end
            end
        end
    end
end

function localSaveFigure(fig, outDir, fileName)

    pngPath = fullfile(outDir, fileName + ".png");
    figPath = fullfile(outDir, fileName + ".fig");

    set(fig, "Color", "w");

    ax = findall(fig, "Type", "axes");
    for i = 1:numel(ax)
        set(ax(i), "Color", "w");
        set(ax(i), "FontSize", 13);
        set(ax(i), "LineWidth", 1.1);
        grid(ax(i), "on");
    end

    exportgraphics(fig, pngPath, "Resolution", 300);
    savefig(fig, figPath);
end