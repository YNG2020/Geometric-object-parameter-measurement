load 'cleanData.mat' cleanData
load 'RATIO.mat' RATIO;
rng(1);
close all
%% 检查数据
for i = [21 22 23 24]
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

%% 分离出平面，然后用PCA方法求出长方形平面的长轴和短轴，并将其投影到水平面上，然后把长方形的四个端点找出来即可求得长和宽
length = 0; width = 0; height = 0;
for i = [22 23]

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
    
    % PCA
    [V, centered_points] = myPCA(points);
    % 投影数据到前两个主成分
    projected_points = centered_points * V(:, 1:2);

    figure;
    hold on
    scatter(projected_points(:,1), projected_points(:,2), 'filled');
    axis equal

    % 计算边界点的极值来确定长和宽
    min_x = min(projected_points(:, 1));
    max_x = max(projected_points(:, 1));
    min_y = min(projected_points(:, 2));
    max_y = max(projected_points(:, 2));

    plot([min_x, min_x, max_x, max_x, min_x], [max_y, min_y, min_y, max_y, max_y], 'Color', 'r', 'LineWidth', 2)
    title('投影至点云数据的两个最大特征值对应的特征空间');
    
    % 计算长宽高
    length = length + (max_x - min_x);
    width = width + (max_y - min_y);
end

%% 输出结果
length = length / 2;
width = width / 2;
fprintf('The length is: %f\n', RATIO * length);
fprintf('The width is: %f\n', RATIO * width);

%% 辅助函数
function [V, centered_points] = myPCA(points)
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
end