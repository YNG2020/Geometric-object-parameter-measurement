close all
clear
load 'data/cleanData.mat' cleanData
rng(1);
%% 检查数据
data = [cleanData{1}; cleanData{2}; cleanData{4}];

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
subplot(1, 2, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云', 'FontSize', 15);
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);

% 检测到的平面点
subplot(1, 2, 2);
pcshow(planePoints, 'r', 'MarkerSize', 40);
title('检测到的平面点', 'FontSize', 15);
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);

%% 分离出平面，然后使用PCA方法降维 + 最小外接矩形求取长和宽
lengthAll = 0; widthAll = 0; heightAll = 0;
for i = [1 2 4]

    data = cleanData{i};

    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象
    
    % 使用 RANSAC 算法检测平面
    maxDistance = 0.005; % 平面距离的阈值
    [model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99.999, 'MaxNumTrials', 10000);
    
    % 分离平面点和平面外点
    planePoints = data(inlierIdx, :); % 平面上的点
    remainingPoints = data(remainIdx, :); % 平面外的点

    points = planePoints;
    
    % 使用PCA方法降维 + 最小外接矩形求取长和宽
    [length, width, height] = findLengthAndWidth(points);
    
    if length < width
        tmp = length;
        length = width;
        width = tmp;
    end

    % 计算长宽高
    lengthAll = length + lengthAll;
    widthAll = width + widthAll;
    heightAll = height + heightAll;
end
length = lengthAll / 3;
width = widthAll / 3;
height = heightAll / 3;
%% 输出结果
RATIO = (60 / length + 30 / width + 30 / height) / 3;
fprintf('The length is: %f\n', RATIO * length);
fprintf('The width is: %f\n', RATIO * width);
fprintf('The height is: %f\n', RATIO * height);
save 'data/RATIO.mat' RATIO;

%% 辅助函数
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
    title('PCA + 最小外接矩形算法', 'FontSize', 15);
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
