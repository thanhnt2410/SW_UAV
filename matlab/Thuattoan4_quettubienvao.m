function Result = Thuattoan4_quettubienvao(showPlots, showTables)
%% =================== Phân vùng tối ưu (lawn-mower trong ENU) ===================
if nargin < 1 || isempty(showPlots)
    showPlots = true;
end
if nargin < 2 || isempty(showTables)
    showTables = true;
end

%% --- STEP 0: Chọn file KML ---
[file, path] = uigetfile('*.kml','Chọn file KML từ Google Earth');
if isequal(file,0), error('Bạn chưa chọn file KML!'); end
kmlFile = fullfile(path,file);

doc = xmlread(kmlFile);
placemarks = doc.getElementsByTagName('Placemark');

lon_gcs = []; lat_gcs = [];
lon_poly = []; lat_poly = [];

for idx = 0:placemarks.getLength-1
    placemark = placemarks.item(idx);
    if isempty(lon_gcs)
        pointNodes = placemark.getElementsByTagName('Point');
        if pointNodes.getLength > 0
            coordsText = strtrim(get_child_text(pointNodes.item(0), 'coordinates'));
            if ~isempty(coordsText)
                [lon_gcs, lat_gcs] = parse_point_coord(coordsText);
            end
        end
    end
    if isempty(lon_poly)
        polyNodes = placemark.getElementsByTagName('Polygon');
        if polyNodes.getLength > 0
            coordsText = strtrim(get_child_text(polyNodes.item(0), 'coordinates'));
            if ~isempty(coordsText)
                [lon_poly, lat_poly] = parse_polygon_coords(coordsText);
            end
        end
    end
    if ~isempty(lon_gcs) && ~isempty(lon_poly)
        break;
    end
end

if isempty(lon_gcs) || isempty(lat_gcs)
    error('Không tìm thấy bất kỳ Placemark kiểu Point (GCS) trong file KML.');
end
if isempty(lon_poly)
    error('Không tìm thấy bất kỳ Placemark kiểu Polygon (ScanArea) trong file KML.');
end

%% --- STEP 1: Chuyển sang ENU với gốc tại GCS ---
wgs84 = wgs84Ellipsoid('meters');
[x_poly, y_poly, ~] = geodetic2enu(lat_poly, lon_poly, zeros(size(lat_poly)), ...
                                   lat_gcs, lon_gcs, 0, wgs84);

P = clean_polyshape_safe([x_poly, y_poly]);
if isempty(P.Vertices)
    error('Polygon rỗng sau khi làm sạch trong hệ ENU.');
end

%% --- STEP 2: Hiển thị polygon và GCS trong ENU ---
if showTables
    fprintf('Số đỉnh polygon hợp lệ trong ENU: %d\n', size(P.Vertices,1));
end
if showPlots
    fig1 = figure('Color','w'); hold on; grid on; axis equal; box on; %#ok<NASGU>
    plot(P,'FaceColor',[0.85 1.0 0.85],'FaceAlpha',0.35,'EdgeColor',[0.3 0.8 0.3],'LineWidth',1.5);
    plot(0, 0,'r^','MarkerSize',10,'LineWidth',2);
    text(0, 0,' UAV Base (ENU)','Color','r','FontWeight','bold');
    xlabel('East (m)'); ylabel('North (m)');
    title('Hình 1 – Phân vùng tối ưu trong hệ ENU');
end

%% --- STEP 3: Ví dụ chuyển waypoint ENU về lat/lon để xuất cho UAV ---
waypointsENU = P.Vertices; % sử dụng biên polygon làm ví dụ
[wayLat, wayLon] = enu_to_geodetic_batch(waypointsENU, lat_gcs, lon_gcs, wgs84);

if ~isempty(wayLat)
    if showTables
        fprintf('Waypoint ENU → geodetic (lat/lon) – hiển thị tối đa 5 điểm:\n');
        showCount = min(5, numel(wayLat));
        disp(table(waypointsENU(1:showCount,1), waypointsENU(1:showCount,2), ...
            wayLat(1:showCount), wayLon(1:showCount), ...
            'VariableNames', {'xEast_m','yNorth_m','Latitude_deg','Longitude_deg'}));
    end
else
    warning('Không có waypoint ENU nào để chuyển ngược.');
end

%% --- STEP 4: Quét từ biên vào (Perimeter-in sweep) ---
numUAV = 6;
camRadius = 10;            % bán kính footprint (m)
maxOverlap = 5;            % overlap tối đa giữa hai vòng
sweepSpacing = 2*camRadius - maxOverlap; % spacing hiệu dụng (m)
thetaStep = deg2rad(5);
thetaList = 0:thetaStep:(pi - thetaStep);
basePoint = [0 0];
costWeights = [0.9, 0.03, 0.6];
costTemplate = struct('turn',0,'length',0,'overlap',0,'total',0,'sweptArea',0,'turnCount',0);

bestConfig = struct('totalCost', inf, 'thetaBase', NaN, 'paths', [], 'costs', [], 'subPolys', []);

for thetaBase = thetaList
    rotP = rotate(P, -rad2deg(thetaBase), [0 0]);
    if isempty(rotP.Vertices)
        continue;
    end
    subPolysRot = split_polygon_into_strips(rotP, numUAV, sweepSpacing);
    if any(cellfun(@(poly) isempty(poly.Vertices), subPolysRot))
        continue;
    end

    thetaPaths = cell(numUAV,1);
    thetaCosts = repmat(costTemplate, numUAV,1);
    thetaTotal = 0;
    validConfig = true;

    for k = 1:numUAV
        perimPathRot = generate_perimeter_in_path(subPolysRot{k}, sweepSpacing);
        if size(perimPathRot,1) < 2
            validConfig = false;
            break;
        end
        perimPathENU = rotate_points(perimPathRot, thetaBase);
        pathWithTransit = add_transit_segments(perimPathENU, basePoint);
        subPolyENU = rotate(subPolysRot{k}, rad2deg(thetaBase), [0 0]);
        costStruct = evaluate_sweep_cost(pathWithTransit, perimPathENU, subPolyENU, camRadius, costWeights);
        thetaPaths{k} = pathWithTransit;
        thetaCosts(k) = costStruct;
        thetaTotal = thetaTotal + costStruct.total;
    end

    if ~validConfig
        continue;
    end

    fprintf('Theta_base = %5.1f deg → Tổng chi phí perimeter-in = %.3f\n', ...
            rad2deg(thetaBase), thetaTotal);

    if thetaTotal < bestConfig.totalCost
        bestConfig.totalCost = thetaTotal;
        bestConfig.thetaBase = thetaBase;
        bestConfig.paths = thetaPaths;
        bestConfig.costs = thetaCosts;
        bestConfig.subPolys = cellfun(@(poly) rotate(poly, rad2deg(thetaBase), [0 0]), ...
                                      subPolysRot, 'UniformOutput', false);
    end
end

if ~isfinite(bestConfig.totalCost)
    error('Không tìm được cấu hình perimeter-in hợp lệ.');
end

fprintf('\nPerimeter-in sweep tối ưu: θ_base = %.2f deg, tổng chi phí = %.3f\n', ...
        rad2deg(bestConfig.thetaBase), bestConfig.totalCost);

if showTables
    fprintf('Đang tổng hợp kết quả theo bảng số liệu...\n');
end

if showPlots
    fig2 = figure('Color','w'); hold on; grid on; axis equal; box on; %#ok<NASGU>
    plot(P,'FaceColor',[1.0 0.95 0.9],'FaceAlpha',0.25,'EdgeColor',[0.6 0.3 0.1],'LineWidth',1.2);
    colors = lines(numUAV);
    legendEntries = {'Vùng quét gốc'};
    for k = 1:numUAV
        subPoly = bestConfig.subPolys{k};
        if ~isempty(subPoly.Vertices)
            plot(subPoly,'FaceColor',colors(k,:),'FaceAlpha',0.08,'EdgeColor',colors(k,:));
        end
        path = bestConfig.paths{k};
        if ~isempty(path)
            plot(path(:,1), path(:,2), '-', 'Color', colors(k,:), 'LineWidth', 1.6);
            plot(path(1,1), path(1,2), 'o', 'Color', colors(k,:), 'MarkerFaceColor', colors(k,:), 'MarkerSize', 4);
        end
        legendEntries{end+1} = sprintf('UAV%d', k); %#ok<AGROW>
    end
    plot(0,0,'kp','MarkerSize',9,'MarkerFaceColor','y');
    xlabel('East (m)'); ylabel('North (m)');
    title(sprintf('Hình 2 – Perimeter-in sweep (θ_b = %.1f°)', rad2deg(bestConfig.thetaBase)));
    legend(legendEntries, 'Location','bestoutside');
end

turnCost = [bestConfig.costs.turn]';
lenCost = [bestConfig.costs.length]';
overlapCost = [bestConfig.costs.overlap]';
totalCost = [bestConfig.costs.total]';
areaSwept = [bestConfig.costs.sweptArea]';
turnCount = [bestConfig.costs.turnCount]';
if showTables
    fprintf('Bảng chi tiết cho từng UAV được hiển thị bên dưới.\n');
end

%% --- STEP 5: Kết quả mô phỏng & xuất cấu trúc chuẩn ---
pathLengthFull = zeros(numUAV,1);
for k = 1:numUAV
    pathLengthFull(k) = path_length_from_points(bestConfig.paths{k});
end
regionAreas = cellfun(@area, bestConfig.subPolys);
totalRegionArea = sum(regionAreas);
scanLen = areaSwept ./ (2*camRadius);
coverageEfficiency = totalRegionArea / max(sum(areaSwept), eps);
meanTurnCost = mean(turnCost);
stdPathLength = std(pathLengthFull);

perimeterResult = struct( ...
    'algorithm_name', 'Perimeter-in sweep', ...
    'theta_opt', rad2deg(bestConfig.thetaBase), ...
    'paths', {bestConfig.paths}, ...
    'path_length', pathLengthFull, ...
    'area_covered', areaSwept, ...
    'J_turn', turnCost, ...
    'J_length', lenCost, ...
    'J_overlap', overlapCost, ...
    'J_total', totalCost, ...
    'J_sum', sum(totalCost), ...
    'coverage_efficiency', coverageEfficiency, ...
    'mean_turn_cost', meanTurnCost, ...
    'std_path_length', stdPathLength, ...
    'region_area_sum', totalRegionArea, ...
    'scan_length', scanLen);

paths = bestConfig.paths;
partitions = bestConfig.subPolys;
path_length = pathLengthFull;
num_turns = turnCount;
covered_area = areaSwept;
overlap_ratio = sum(covered_area) / max(totalRegionArea, eps);
cost_per_UAV = totalCost;

uavMetricsTable = table((1:numUAV)', path_length, num_turns, covered_area, ...
    turnCost, lenCost, overlapCost, totalCost, ...
    'VariableNames', {'UAV','Path_length_m','Turn_count','Area_swept_m2', ...
    'J_turn','J_length','J_overlap','J_total'});
overallTable = table(rad2deg(bestConfig.thetaBase), bestConfig.totalCost, sum(path_length), totalRegionArea, ...
    sum(covered_area), overlap_ratio, sum(num_turns), ...
    'VariableNames', {'Theta_opt_deg','Total_cost','Total_path_length_m', ...
    'Region_area_m2','Swept_area_m2','Overlap_ratio','Total_turns'});
uavMetricsArray = table2array(uavMetricsTable);
overallArray = table2array(overallTable);
uavFormats = {'%02.0f','%.2f','%.0f','%.2f','%.3f','%.3f','%.3f','%.3f'};
overallFormats = {'%.2f','%.3f','%.2f','%.2f','%.2f','%.4f','%.0f'};
uavDisplay = cell(size(uavMetricsArray));
for col = 1:size(uavMetricsArray,2)
    uavDisplay(:,col) = arrayfun(@(val) sprintf(uavFormats{col}, val), ...
        uavMetricsArray(:,col), 'UniformOutput', false);
end
overallDisplay = cell(size(overallArray));
for col = 1:size(overallArray,2)
    overallDisplay(:,col) = arrayfun(@(val) sprintf(overallFormats{col}, val), ...
        overallArray(:,col), 'UniformOutput', false);
end
if showTables
    disp(uavMetricsTable);
    disp(overallTable);
    for k = 1:numUAV
        fprintf('UAV%02d | L = %.2f m | Turns = %.0f | Area = %.2f m^2 | J_turn = %.3f | J_length = %.3f | J_overlap = %.3f | J_total = %.3f\n', ...
            k, path_length(k), num_turns(k), covered_area(k), turnCost(k), lenCost(k), overlapCost(k), totalCost(k));
    end
    fprintf('Tổng quan | Theta* = %.2f deg | Tổng cost = %.3f | Tổng chiều dài = %.2f m | Diện tích vùng = %.2f m^2 | Diện tích quét = %.2f m^2 | Overlap ratio = %.4f | Tổng số lần quay = %.0f\n', ...
        rad2deg(bestConfig.thetaBase), bestConfig.totalCost, sum(path_length), totalRegionArea, sum(covered_area), overlap_ratio, sum(num_turns));
end

if showPlots
    columnFormatCharUAV = repmat({'char'},1,size(uavMetricsArray,2));
    columnFormatCharOverall = repmat({'char'},1,size(overallArray,2));
    statsFig = figure('Color','w','Name','Hình 3 – Thống kê số liệu Perimeter-in'); %#ok<NASGU>
    panel1 = uipanel(statsFig,'Title','Bảng chi tiết từng UAV','FontWeight','bold', ...
        'Units','normalized','Position',[0.05 0.48 0.9 0.47]);
    uitable(panel1,'Data',uavDisplay, ...
        'ColumnName',uavMetricsTable.Properties.VariableNames, ...
        'ColumnFormat',columnFormatCharUAV, ...
        'Units','normalized','Position',[0 0 1 1], ...
        'RowName',[]);
    panel2 = uipanel(statsFig,'Title','Tổng quan nhiệm vụ','FontWeight','bold', ...
        'Units','normalized','Position',[0.05 0.05 0.9 0.35]);
    uitable(panel2,'Data',overallDisplay, ...
        'ColumnName',overallTable.Properties.VariableNames, ...
        'ColumnFormat',columnFormatCharOverall, ...
        'Units','normalized','Position',[0 0 1 1], ...
        'RowName',[]);
end

Result = struct( ...
    'algorithm_name', 'Perimeter-in sweep', ...
    'numUAV', numUAV, ...
    'theta_opt', rad2deg(bestConfig.thetaBase), ...
    'paths', {paths}, ...
    'partition', {partitions}, ...
    'path_length', path_length, ...
    'num_turns', num_turns, ...
    'covered_area', covered_area, ...
    'overlap_ratio', overlap_ratio, ...
    'J_turn', turnCost, ...
    'J_length', lenCost, ...
    'J_overlap', overlapCost, ...
    'J_total', cost_per_UAV, ...
    'cost_per_UAV', cost_per_UAV, ...
    'total_cost', sum(cost_per_UAV), ...
    'coverage_efficiency', coverageEfficiency, ...
    'mean_turn_cost', meanTurnCost, ...
    'std_path_length', stdPathLength, ...
    'region_area_sum', totalRegionArea, ...
    'scan_length', scanLen);

fprintf(['\n[Perimeter-in] Tổng cost = %.3f, Tổng chiều dài = %.1f m, ', ...
         'η = %.3f, mean J_turn = %.3f, σ_L = %.2f m\n'], ...
        Result.total_cost, sum(path_length), overlap_ratio, ...
        meanTurnCost, stdPathLength);

end

%% =================== HÀM PHỤ ===================
function textValue = get_child_text(node, tagName)
    nodes = node.getElementsByTagName(tagName);
    if nodes.getLength == 0
        textValue = '';
    else
        textValue = char(nodes.item(0).getTextContent);
    end
end

function [lon, lat] = parse_point_coord(coordText)
    tokens = strsplit(strtrim(coordText), ',');
    lon = str2double(tokens{1});
    lat = str2double(tokens{2});
end

function [lon, lat] = parse_polygon_coords(coordText)
    rawTokens = regexp(coordText, '[\s\r\n]+', 'split');
    rawTokens = rawTokens(~cellfun('isempty', rawTokens));
    lon = zeros(numel(rawTokens),1);
    lat = zeros(numel(rawTokens),1);
    for i = 1:numel(rawTokens)
        tokens = strsplit(strtrim(rawTokens{i}), ',');
        lon(i) = str2double(tokens{1});
        lat(i) = str2double(tokens{2});
    end
    if numel(lon) >= 2 && lon(1) == lon(end) && lat(1) == lat(end)
        lon(end) = [];
        lat(end) = [];
    end
end

function [lat, lon] = enu_to_geodetic_batch(enuPts, lat0, lon0, ellipsoid)
    if isempty(enuPts)
        lat = [];
        lon = [];
        return;
    end
    [lat, lon, ~] = enu2geodetic( ...
        enuPts(:,1), enuPts(:,2), zeros(size(enuPts,1),1), ...
        lat0, lon0, 0, ellipsoid);
end

function subPolys = split_polygon_into_strips(polyIn, numStrips, spacing)
    subPolys = cell(numStrips,1);
    verts = polyIn.Vertices;
    if isempty(verts)
        return;
    end
    verts = verts(~any(isnan(verts),2),:);
    yMin = min(verts(:,2));
    yMax = max(verts(:,2));
    xMin = min(verts(:,1)) - spacing;
    xMax = max(verts(:,1)) + spacing;
    edges = linspace(yMin, yMax, numStrips+1);
    for i = 1:numStrips
        lower = edges(i);
        upper = edges(i+1);
        strip = polyshape([xMin xMax xMax xMin], [lower lower upper upper]);
        subPolys{i} = intersect(polyIn, strip);
    end
end

function path = generate_perimeter_in_path(polyRot, spacing)
    path = [];
    if isempty(polyRot.Vertices)
        return;
    end
    current = polyshape(polyRot.Vertices);
    maxLoops = 200;
    minArea = max(1.0, 0.0005*area(polyRot));
    for k = 1:maxLoops
        if isempty(current.Vertices) || area(current) < minArea
            break;
        end
        boundary = ordered_boundary(current);
        if isempty(boundary)
            break;
        end
        if isempty(path)
            path = boundary;
        else
            path = [path; boundary]; %#ok<AGROW>
        end
        buffered = polybuffer(current, -spacing);
        nextPoly = select_largest_component(buffered);
        if isempty(nextPoly.Vertices)
            break;
        end
        current = nextPoly;
    end
    path = unique_consecutive_points(path);
end

function boundary = ordered_boundary(polyObj)
    verts = polyObj.Vertices;
    verts = verts(~any(isnan(verts),2),:);
    if isempty(verts)
        boundary = [];
        return;
    end
    if norm(verts(1,:) - verts(end,:)) < 1e-9
        boundary = verts;
    else
        boundary = [verts; verts(1,:)];
    end
end

function polyOut = select_largest_component(polyIn)
    if isempty(polyIn)
        polyOut = polyshape();
        return;
    end
    try
        comps = regions(polyIn);
    catch
        polyOut = polyIn;
        return;
    end
    if isempty(comps)
        polyOut = polyshape();
        return;
    end
    if numel(comps) == 1
        polyOut = comps;
        return;
    end
    areas = arrayfun(@area, comps);
    [~, idx] = max(areas);
    polyOut = comps(idx);
end

function pathOut = add_transit_segments(pathIn, basePt)
    if isempty(pathIn)
        pathOut = pathIn;
        return;
    end
    tol = 1e-6;
    pathOut = pathIn;
    if norm(pathOut(1,:) - basePt) > tol
        pathOut = [basePt; pathOut]; %#ok<AGROW>
    end
    if norm(pathOut(end,:) - basePt) > tol
        pathOut = [pathOut; basePt]; %#ok<AGROW>
    end
end

function ptsOut = rotate_points(ptsIn, theta)
    if isempty(ptsIn)
        ptsOut = ptsIn;
        return;
    end
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    ptsOut = (R * ptsIn.').';
end

function costStruct = evaluate_sweep_cost(fullPath, scanPath, polyRegion, camRadius, weights)
    if nargin < 2 || isempty(scanPath)
        scanPath = fullPath;
    end
    costStruct = struct('turn',inf,'length',inf,'overlap',inf,'total',inf,'sweptArea',inf,'turnCount',inf);
    if size(fullPath,1) < 2 || area(polyRegion) <= 0
        return;
    end
    diffsFull = diff(fullPath,1,1);
    segLenFull = sqrt(sum(diffsFull.^2,2));
    Jlen = sum(segLenFull);
    if numel(segLenFull) >= 2
        headings = atan2(diffsFull(:,2), diffsFull(:,1));
        dHead = abs(wrapToPi_local(diff(headings)));
        Jturn = sum(dHead);
        turnCount = sum(dHead > deg2rad(1));
    else
        Jturn = 0;
        turnCount = 0;
    end
    scanLen = 0;
    if size(scanPath,1) >= 2
        scanLen = sum(sqrt(sum(diff(scanPath,1,1).^2,2)));
    end
    sweptArea = scanLen * 2 * camRadius;
    Jover = sweptArea / max(area(polyRegion), eps);
    total = weights(1)*Jturn + weights(2)*Jlen + weights(3)*Jover;
    costStruct.turn = Jturn;
    costStruct.length = Jlen;
    costStruct.overlap = Jover;
    costStruct.total = total;
    costStruct.sweptArea = sweptArea;
    costStruct.turnCount = turnCount;
end

function ptsOut = unique_consecutive_points(pts)
    if size(pts,1) <= 1
        ptsOut = pts;
        return;
    end
    diffs = [true; any(abs(diff(pts,1,1)) > 1e-9, 2)];
    ptsOut = pts(diffs,:);
end

function angles = wrapToPi_local(angleVec)
    angles = mod(angleVec + pi, 2*pi) - pi;
end

function L = path_length_from_points(path)
    if size(path,1) < 2
        L = 0;
    else
        L = sum(sqrt(sum(diff(path,1,1).^2,2)));
    end
end
