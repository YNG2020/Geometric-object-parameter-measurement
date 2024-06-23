close all
clear
load data/RATIO.mat RATIO
load data/cleanData.mat cleanData
rng(1);
%% 检查数据
data = [cleanData{13}; cleanData{14}; cleanData{15}; cleanData{16}];

% 使用 MATLAB 内置的 pcfitplane 函数
ptCloud = pointCloud(data); % 将数据转换为点云对象

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
subplot(1, 3, 1)
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云', 'FontSize', 15);
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);

%% 方法一：将点云最高点视为球体最高点
diameter1 = 0;
for i = [13 14 15 16]
    data = cleanData{i};
    diameter1 = diameter1 + max(data(:, 3));
end
radius1 = diameter1 / 8;
%% 方法二：用RANSAC拟合

radius2All = 0;
for i = [13 14 15 16]

    % RANSAC 参数
    maxIterations = 100000;
    maxDistance = 0.005;
    bestInliers = 0;
    data = [cleanData{i}];
    
    % 使用 MATLAB 内置的 pcfitsphere 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象
    model = pcfitsphere(ptCloud, maxDistance, 'Confidence', 99.999, 'MaxNumTrials', 10000);
    center = model.Center; radius2All = radius2All + model.Radius;
    model.Radius
end

radius2 = radius2All / 4;
% 绘制最佳拟合球体
% 创建球体表面数据
[theta, phi] = meshgrid(linspace(0, pi, 30), linspace(0, 2 * pi, 30));
x_sphere = model.Radius * sin(theta) .* cos(phi) + center(1);
y_sphere = model.Radius * sin(theta) .* sin(phi) + center(2);
z_sphere = model.Radius * cos(theta) + center(3);


% 绘制点云数据和拟合球体
subplot(1, 3, 2)
scatter3(data(:, 1), data(:, 2), data(:, 3), 'filled');
hold on;
surf(x_sphere, y_sphere, z_sphere, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);
set(gcf, 'Color', 'k'); % 设置图形窗口背景颜色
ax = gca; % 获取当前坐标轴
ax.Color = 'k'; % 设置坐标轴背景颜色为黑色
ax.XColor = 'w';ax.YColor = 'w';ax.ZColor = 'w';
colormap('cool'); % 设置颜色映射
axis equal;
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);
title('点云和拟合球体 (MSAC)', 'FontSize', 15, 'Color', 'w');
hold off;

%% 方法三：用最小二乘拟合

radius3All = 0;
for i = [13 14 15 16]
    data = [cleanData{i}];
    N = size(data, 1);
    % 构建V矩阵
    V = [2 * data, ones(N, 1)];
    % 构建b向量
    b = sum(data.^2, 2);
    % 使用最小二乘法求解
    w = (V' * V) \ (V' * b);
    % 提取球心坐标和半径
    center = w(1:3)';
    radius3All = radius3All + sqrt(w(4) + sum(center.^2));
    sqrt(w(4) + sum(center.^2))
end

radius3 = radius3All / 4;
% 绘制点云数据和拟合球体
subplot(1, 3, 3)
scatter3(data(:, 1), data(:, 2), data(:, 3), 'filled');
hold on;
% 创建球体表面数据
[theta, phi] = meshgrid(linspace(0, pi, 30), linspace(0, 2 * pi, 30));
x_sphere = sqrt(w(4) + sum(center.^2)) * sin(theta) .* cos(phi) + center(1);
y_sphere = sqrt(w(4) + sum(center.^2)) * sin(theta) .* sin(phi) + center(2);
z_sphere = sqrt(w(4) + sum(center.^2)) * cos(theta) + center(3);
set(gcf, 'Color', 'k'); % 设置图形窗口背景颜色
ax = gca; % 获取当前坐标轴
ax.Color = 'k'; % 设置坐标轴背景颜色为黑色
ax.XColor = 'w';ax.YColor = 'w';ax.ZColor = 'w';

surf(x_sphere, y_sphere, z_sphere, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);
axis equal;
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);
title('点云和拟合球体 (LS)', 'FontSize', 15, 'Color', 'w');
hold off;
%% 输出结果
fprintf("The radius calculated by the maximun height is: %f\n mm", radius1 * RATIO);
fprintf("The radius calculated by MSAC is: %f\n mm", radius2 * RATIO);
fprintf("The radius calculated by LS is: %f\n mm", radius3 * RATIO);
