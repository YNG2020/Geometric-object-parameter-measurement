load 'cleanData.mat' cleanData
load 'RATIO.mat' RATIO;
rng(1);
%% 分离出长方形块
data = cleanData{20};

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



%% 输出数据
% fprintf('The length of A is: %f\n', RATIO * len);
% fprintf('The length of B is: %f\n', RATIO * len);
% fprintf('The length of C is: %f\n', RATIO * len);
% fprintf('The length of D is: %f\n', RATIO * len);