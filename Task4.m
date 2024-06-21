load 'cleanData.mat' cleanData
load RATIO.mat RATIO
rng(1);

%% 检查数据
data = [cleanData{13}; cleanData{14}; cleanData{15}; cleanData{16}];

% 使用 MATLAB 内置的 pcfitplane 函数
ptCloud = pointCloud(data); % 将数据转换为点云对象

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% 方法一：将点云最高点视为球体最高点
diameter1 = 0;
for i = [13 14 15 16]
    data = cleanData{i};
    diameter1 = diameter1 + max(data(:, 3));
end
radius1 = diameter1 / 8;
%% 方法二：用RANSAC拟合
% 迭代 RANSAC
% RANSAC 参数
maxIterations = 100000;
maxDistance = 0.005;
bestInliers = 0;
data = [cleanData{13}; cleanData{14}; cleanData{15}; cleanData{16}];

% 使用 MATLAB 内置的 pcfitsphere 函数
ptCloud = pointCloud(data); % 将数据转换为点云对象
model = pcfitsphere(ptCloud, maxDistance, 'Confidence', 99.999, 'MaxNumTrials', 10000);

% 绘制最佳拟合球体
center = model.Center; radius2 = model.Radius;
% 创建球体表面数据
[theta, phi] = meshgrid(linspace(0, pi, 30), linspace(0, 2 * pi, 30));
x_sphere = radius2 * sin(theta) .* cos(phi) + center(1);
y_sphere = radius2 * sin(theta) .* sin(phi) + center(2);
z_sphere = radius2 * cos(theta) + center(3);

% 绘制点云数据和拟合球体
figure;
scatter3(data(:, 1), data(:, 2), data(:, 3), 'filled');
hold on;
surf(x_sphere, y_sphere, z_sphere, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
colormap('cool'); % 设置颜色映射
colorbar;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('点云数据和拟合球体');
hold off;

%% 方法三：用最小二乘拟合
data = [cleanData{13}; cleanData{14}; cleanData{15}; cleanData{16}];
N = size(data, 1);
% 构建V矩阵
V = [2 * data, ones(N, 1)];
% 构建b向量
b = sum(data.^2, 2);
% 使用最小二乘法求解
w = (V' * V) \ (V' * b);
% 提取球心坐标和半径
center = w(1:3)';
radius3 = sqrt(w(4) + sum(center.^2));

% 绘制点云数据和拟合球体
figure;
scatter3(data(:, 1), data(:, 2), data(:, 3), 'filled');
hold on;
% 创建球体表面数据
[theta, phi] = meshgrid(linspace(0, pi, 30), linspace(0, 2 * pi, 30));
x_sphere = radius3 * sin(theta) .* cos(phi) + center(1);
y_sphere = radius3 * sin(theta) .* sin(phi) + center(2);
z_sphere = radius3 * cos(theta) + center(3);
surf(x_sphere, y_sphere, z_sphere, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
colormap('cool'); % 设置颜色映射
colorbar;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('点云数据和拟合球体');
hold off;
%% 输出结果
fprintf("The radius calculated bt method1 (easy method) is: %f\n", radius1 * RATIO);
fprintf("The radius calculated bt method2 (RANSAC) is: %f\n", radius2 * RATIO);
fprintf("The radius calculated bt method3 (LS) is: %f\n", radius3 * RATIO);
