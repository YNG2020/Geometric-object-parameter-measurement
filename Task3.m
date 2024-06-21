load 'cleanData.mat' cleanData
rng(1);

for i = [9 10 11 12]
    data = [cleanData{i}];

    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象

    % 使用 RANSAC 算法检测平面
    maxDistance = 0.005; % 平面距离的阈值
    [model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99.999, 'MaxNumTrials', 10000);

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

end

A = zeros(4, 3);
b = zeros(4, 1);
for i = [9 10 11 12]

    data = cleanData{i};

    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象

    % 使用 RANSAC 算法检测平面
    maxDistance = 0.005; % 平面距离的阈值
    [model, ~, ~] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99.999, 'MaxNumTrials', 10000);
    A(i - 7, :) = model.Normal;
    b(i - 7) = -model.Parameters(4);
end

% 使用最小二乘法求解
x = A\b;

% 显示结果
disp('The intersection point of the four planes is:');
disp(['x = ', num2str(x(1))]);
disp(['y = ', num2str(x(2))]);
disp(['z = ', num2str(x(3))]);



