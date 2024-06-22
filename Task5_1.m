close all
clear
load data/RATIO.mat RATIO
load data/cleanData.mat cleanData
rng(1);
%% 检查数据
for i = [17 18 19 20]
    data = cleanData{i};
    
    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象
    
    % 可视化结果
    figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");
    
    % 原始点云数据
    pcshow(ptCloud, 'MarkerSize', 40);
    title('原始点云');
    xlabel('X'); ylabel('Y'); zlabel('Z');
end
close all
%% 分离出长方形块
data = cleanData{19};

% 使用 MATLAB 内置的 pcfitplane 函数
ptCloud = pointCloud(data); % 将数据转换为点云对象

% 使用 RANSAC 算法检测平面
maxDistance = 0.005; % 平面距离的阈值
[model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, [0 0 1], 'Confidence', 99.999, 'MaxNumTrials', 10000);

% 分离平面点和平面外点
planePoints = data(inlierIdx, :); % 平面上的点
remainingPoints = data(remainIdx, :); % 平面外的点

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
subplot(1, 3, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

% 检测到的平面点
subplot(1, 3, 2);
pcshow(planePoints, 'b', 'MarkerSize', 40);
title('检测到的平面点');
xlabel('X'); ylabel('Y'); zlabel('Z');

% 构建长方体与三棱柱的分离面
p1 = [0.009871631860733,0.018593976274133,0.010984361171722];
p2 = [-0.013323277235031,0.003625014331192,0.018150568008423];
p3 = [0.005797982215881,0.015077436342835,0.014956891536713];

[a, b, c, d] = constructPlaneEquation(p1, p2, p3);

% 分离长方体与三棱柱
inlierIdx = zeros(size(planePoints, 1), 1); outlierIdxIdx = zeros(size(planePoints, 1), 1);
for i = 1 : size(planePoints, 1)
    p1 = planePoints(i, 1);
    p2 = planePoints(i, 2);
    p3 = planePoints(i, 3);
    if a * p1 + b * p2 + c * p3 + d < 0
        inlierIdx(i) = 1;
    else
        outlierIdxIdx(i) = 1;
    end
end

inlierIdx = find(inlierIdx);
outlierIdxIdx = find(outlierIdxIdx);

cuboid = planePoints(inlierIdx, :);
Tri_Prism = planePoints(outlierIdxIdx, :);

% 分离点云数据
subplot(1, 3, 3);
pcshow(cuboid, 'g', 'MarkerSize', 40);
hold on
plotPlane(a, b, c, d)
pcshow(Tri_Prism, 'r', 'MarkerSize', 40);
title('分离点云数据');
xlabel('X'); ylabel('Y'); zlabel('Z');

% 使用PCA方法降维 + 最小外接矩形求取长和宽
[length, width, height] = findLengthAndWidth(cuboid);

%% 输出数据
fprintf('The length of cuboid is: %f\n', RATIO * length);
fprintf('The width of cuboid is: %f\n', RATIO * width);
fprintf('The height of cuboid is: %f\n', RATIO * height);
%% 辅助函数
function [a, b, c, d] = constructPlaneEquation(P1, P2, P3)
    % 输入：三个点，P1, P2, P3，格式为 [x, y, z]
    % 输出：平面方程的系数 a, b, c, d

    % 计算两个向量
    v1 = P2 - P1;
    v2 = P3 - P1;
    
    % 计算法向量（叉积）
    normal = cross(v1, v2);
    
    % 提取法向量的分量
    a = normal(1);
    b = normal(2);
    c = normal(3);
    
    % 计算常数项 d
    d = -(a * P1(1) + b * P1(2) + c * P1(3));
end

function plotPlane(a, b, c, d)
    % a, b, c, d 为平面方程 ax + by + cz + d = 0 的系数
    
    % 创建 x, y 的网格范围
    [X, Z] = meshgrid(-0.03:0.0001:0.03, 0.012:0.0001:0.022);
    
    % 计算 z 的值

    Y = -(a * X + c * Z + d) / b;
    
    % 绘制平面
    surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'y');

end

% 使用PCA方法降维 + 最小外接矩形求取长和宽
function [length, width, height] = findLengthAndWidth(points)
    % 中心化数据
    mean_points = mean(points);
    centered_points = points - mean_points;
    
    % 计算协方差矩阵
    cov_matrix = cov(centered_points);
    
    % 求解特征值和特征向量
    [V, D] = eig(cov_matrix);
    
    % 按特征值大小排序
    [~, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    
    % 投影数据到前两个主成分
    projected_points = double(centered_points * V(:, 1:2));

    % 计算点云的凸包
    k = convhull(projected_points(:,1), projected_points(:,2));
    
    % 提取凸包点
    hull_points = projected_points(k, :);
    
    % 计算最小外接矩形
    [minAreaRect, ~] = minBoundingRect(hull_points);

    % 计算长宽高
    length = norm(minAreaRect(1,:) - minAreaRect(2,:));
    width = norm(minAreaRect(2,:) - minAreaRect(3,:));
    height = max(points(:, 3));

    % 绘制点云和最小外接矩形
    figure;
    scatter(projected_points(:,1), projected_points(:,2), 'b', 'filled');
    hold on;
    plot([minAreaRect(:,1); minAreaRect(1,1)], [minAreaRect(:,2); minAreaRect(1,2)], 'r-', 'LineWidth', 2);
    title('最小外接矩形');
    axis equal
    xlabel('X');
    ylabel('Y');
    hold off;

end

% 最小外接矩形算法
function [mbr, area] = minBoundingRect(pts)
    % Calculate the minimum bounding rectangle using rotating calipers method
    K = convhull(pts); % Convex hull
    pts = pts(K, :); % Convex hull vertices
    n = size(pts, 1); % Number of vertices

    if n < 3
        error('At least 3 points are needed to form a rectangle.');
    end

    angles = atan2(pts([2:end, 1], 2) - pts(:, 2), pts([2:end, 1], 1) - pts(:, 1));
    angles = unique(mod(angles, pi/2)); % Reduce angles to the first quadrant

    minArea = inf;
    mbr = [];

    for angle = angles'
        R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        rot_pts = (R * pts')';
        min_pts = min(rot_pts);
        max_pts = max(rot_pts);
        area = prod(max_pts - min_pts);

        if area < minArea
            minArea = area;
            bounds = [min_pts; max_pts];
            mbr = [bounds(1,1), bounds(1,2);
                   bounds(1,1), bounds(2,2);
                   bounds(2,1), bounds(2,2);
                   bounds(2,1), bounds(1,2)];
            mbr = (R' * mbr')';
        end
    end
end