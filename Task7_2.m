load 'cleanData.mat' cleanData
load 'RATIO.mat' RATIO;
rng(1);
close all
%% 检查数据
for i = [25 26 27 28]
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

%% 构造分离平面，分开圆柱顶面与其它点云
data = cleanData{27};

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
subplot(1, 3, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

xlim = [0.05 0.15];
ylim = [0.07 0.11];
zlim = [0.09 0.13];

p1 = [0.107610315084457,0.103516727685928,0.103835910558701];
p2 = [0.079292714595795,0.081717140972614,0.116279602050781];
p3 = [0.126926958560944,0.078680209815502,0.089770913124084];
[plane, ~] = divideData(data, p1, p2, p3, xlim, ylim, zlim);

%% 使用RANSAC方法去噪
maxDistance = 0.002; % 平面距离的阈值

ptCloud = pointCloud(plane);
[model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99.99, 'MaxNumTrials', 10000);

% 分离平面点和平面外点
other = plane(remainIdx, :);
cleanPlane = plane(inlierIdx, :); % 平面上的点

subplot(1, 3, 3);
hold on
title('去噪结果');
pcshow(cleanPlane, 'g', 'MarkerSize', 40);
pcshow(other, 'r', 'MarkerSize', 40);

%% 使用PCA方法降维 + 使用最小覆盖圆求取直径
[PCAPlane] = myPCA(cleanPlane);

% 计算边界点的极值来确定圆柱的高
min_x = min(PCAPlane(:, 1));
max_x = max(PCAPlane(:, 1));

height = max_x - min_x;

% 绘制结果
figure;
scatter(PCAPlane(:,1), PCAPlane(:,2), 'filled');
title('投影至点云数据的两个最大特征值对应的特征空间');
axis equal;
hold off;

%% 输出结果
fprintf('The height is: %f\n', RATIO * height);

%% 辅助函数：用于构造分割平面方程数据分离
function [inlier, outliner] = divideData(data, p1, p2, p3, xlim, ylim, zlim)
    [a, b, c, d] = constructPlaneEquation(p1, p2, p3);
    % 将数据分为两块
    inlierIdx = zeros(size(data, 1), 1); outlierIdxIdx = zeros(size(data, 1), 1);
    for i = 1 : size(data, 1)
        p1 = data(i, 1);
        p2 = data(i, 2);
        p3 = data(i, 3);
        if a * p1 + b * p2 + c * p3 + d >= 0
            inlierIdx(i) = 1;
        else
            outlierIdxIdx(i) = 1;
        end
    end

    inlier = data(inlierIdx == 1, :);
    outliner = data(outlierIdxIdx == 1, :);
    
    % 分离点云数据
    subplot(1, 3, 2);
    pcshow(outliner, 'r', 'MarkerSize', 40);
    hold on
    plotPlane(a, b, c, d, xlim, ylim, zlim, 'y')
    pcshow(inlier, 'g', 'MarkerSize', 40);
    title('分离点云数据');
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

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

function plotPlane(a, b, c, d, xlim, ylim, zlim, color)
    % a, b, c, d 为平面方程 ax + by + cz + d = 0 的系数
   
    x = zeros(4, 1);
    x(1) = -(b * ylim(1) + c * zlim(1) + d) / a;x(2) = -(b * ylim(1) + c * zlim(2) + d) / a;
    x(3) = -(b * ylim(2) + c * zlim(1) + d) / a;x(4) = -(b * ylim(2) + c * zlim(2) + d) / a;
    [maxDiffX] = findMaxDiff(x);
    y = zeros(4, 1);
    y(1) = -(a * xlim(1) + c * zlim(1) + d) / b;y(2) = -(a * xlim(1) + c * zlim(2) + d) / b;
    y(3) = -(a * xlim(2) + c * zlim(1) + d) / b;y(4) = -(a * xlim(2) + c * zlim(2) + d) / b;
    [maxDiffY] = findMaxDiff(y);
    z = zeros(4, 1);
    z(1) = -(a * xlim(1) + b * ylim(1) + d) / c;z(2) = -(a * xlim(1) + b * ylim(2) + d) / c;
    z(3) = -(a * xlim(2) + b * ylim(1) + d) / c;z(4) = -(a * xlim(2) + b * ylim(2) + d) / c;
    [maxDiffZ] = findMaxDiff(z);

    if min([maxDiffX maxDiffY maxDiffZ]) == maxDiffX
        [Y, Z] = meshgrid(linspace(ylim(1), ylim(2)), linspace(zlim(1), zlim(2)));
        X = -(b * Y + c * Z + d) / a;
    end
    if min([maxDiffX maxDiffY maxDiffZ]) == maxDiffY
        [X, Z] = meshgrid(linspace(xlim(1), xlim(2)), linspace(zlim(1), zlim(2)));
        Y = -(a * X + c * Z + d) / b;
    end
    if min([maxDiffX maxDiffY maxDiffZ]) == maxDiffZ
        [X, Y] = meshgrid(linspace(xlim(1), xlim(2)), linspace(ylim(1), ylim(2)));
        Z = -(a * X + b * Y + d) / c;
    end
    % 绘制平面
    surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', color);

end

function [maxDiff] = findMaxDiff(x)
    % 计算所有可能的差值组合
    comb_indices = nchoosek(1:length(x), 2);
    differences = abs(x(comb_indices(:,1)) - x(comb_indices(:,2)));
    % 找出最大的差值
    maxDiff = max(differences);
end

%% 辅助函数：PCA
function [projected_points] = myPCA(points)
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
    projected_points = centered_points * V(:, 1:2);
end
