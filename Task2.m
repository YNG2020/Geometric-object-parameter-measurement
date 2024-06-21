load 'cleanData.mat' cleanData
rng(1);
data = [cleanData{7}; cleanData{8}];

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
pcshow(planePoints, 'r', 'MarkerSize', 40);
title('检测到的平面点');
xlabel('X'); ylabel('Y'); zlabel('Z');

% 去除了平面的点云数据
subplot(1, 3, 3);
pcshow(remainingPoints, 'b', 'MarkerSize', 40);
title('去平面的点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

diameter = 0; height = 0;
for i = [7 8]

    data = cleanData{i};

    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象
    
    % 使用 RANSAC 算法检测平面
    maxDistance = 0.005; % 平面距离的阈值
    [model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, [0 0 1], 'Confidence', 99.999, 'MaxNumTrials', 10000);
    
    % 分离平面点和平面外点
    planePoints = data(inlierIdx, :); % 平面上的点
    remainingPoints = data(remainIdx, :); % 平面外的点

    points = planePoints;
    
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
    projected_points = centered_points * V(:, 1:2);

    axis equal

    [center, radius] = findMinCircle(projected_points);
    
    % 绘制结果
    figure;
    scatter(projected_points(:,1), projected_points(:,2), 'filled');
    hold on;
    theta = linspace(0, 2*pi, 100);
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    plot(x, y, 'r-', 'LineWidth', 2);
    plot(center(1), center(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    title('最小覆盖圆');
    axis equal;
    hold off;

    height = height + max(points(:, 3));
    diameter = diameter + 2 * radius;
end

diameter = diameter / 2;
height = height / 2;
load RATIO.mat RATIO
fprintf('The diameter is: %f\n', RATIO * diameter);
fprintf('The height is: %f\n', RATIO * height);

function [center, radius] = findMinCircle(points)
    % 主函数，用于找到点集的最小覆盖圆
    % points: 点集（nx2矩阵）
    % center: 最小覆盖圆的中心
    % radius: 最小覆盖圆的半径
    
    % 调用递归辅助函数
    [center, radius] = minCircleHelper(points, []);
end

function [center, radius] = minCircleHelper(P, R)
    % 递归辅助函数，P是剩余点集，R是边界上的点
    if isempty(P) || numel(R) == 3
        % 基本情况：如果没有点或边界上有三个点
        [center, radius] = getCircle(R);
        return;
    end
    
    % 随机选择一个点进行递归
    P = P(randperm(size(P, 1)), :); % 打乱点集顺序
    p = P(end, :); % 选择最后一个点
    P(end, :) = []; % 从点集P中移除这个点
    
    [center, radius] = minCircleHelper(P, R);
    
    % 检查这个点是否在当前最小圆内
    if norm(p - center) > radius
        % 如果不在，则将这个点添加到边界点集R
        [center, radius] = minCircleHelper(P, [R; p]);
    end
end

function [center, radius] = getCircle(R)
    % 计算点集R的最小覆盖圆
    switch size(R, 1)
        case 0
            center = [0, 0];
            radius = 0;
        case 1
            center = R(1, :);
            radius = 0;
        case 2
            % 两点的情况
            center = (R(1, :) + R(2, :)) / 2;
            radius = norm(R(1, :) - R(2, :)) / 2;
        case 3
            % 三点的情况，计算经过三点的圆
            A = R(1, :);
            B = R(2, :);
            C = R(3, :);
            D = 2 * (A(1) * (B(2) - C(2)) + B(1) * (C(2) - A(2)) + C(1) * (A(2) - B(2)));
            if D == 0
                center = [NaN, NaN];
                radius = NaN;
            else
                center = [(norm(A)^2 * (B(2) - C(2)) + norm(B)^2 * (C(2) - A(2)) + norm(C)^2 * (A(2) - B(2))) / D, ...
                          (norm(A)^2 * (C(1) - B(1)) + norm(B)^2 * (A(1) - C(1)) + norm(C)^2 * (B(1) - A(1))) / D];
                radius = norm(A - center);
            end
        otherwise
            error('Unexpected number of boundary points.');
    end
end


