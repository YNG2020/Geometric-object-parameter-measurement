% 设置随机种子以保证结果可复现
rng(0);

% 生成一些正态分布的内群点数据
x_inliers = 3 * randn(1, 60);
y_inliers = 2 * x_inliers + randn(1, 60);

% 生成一些离群点数据
x_outliers = 10 * randn(1, 20);
y_outliers = 10 * randn(1, 20);

% 合并数据
x_data = [x_inliers, x_outliers];
y_data = [y_inliers, y_outliers];

% 图像左侧 - 原始数据
figure;
subplot(1, 2, 1);
scatter(x_data, y_data, 'filled');
title('在含噪数据中寻找最佳拟合线', 'fontSize', 13);
xlabel('X', 'fontSize', 13);
ylabel('Y', 'fontSize', 13);
axis off
axis equal;

% 使用 RANSAC 算法来拟合线性模型
data = [x_data', y_data'];
[~, inlierIdx] = ransacfitline(data', 1, 0.5, 1000);

% 获取内群点和离群点
x_inliers = data(inlierIdx, 1);
y_inliers = data(inlierIdx, 2);
x_outliers = data(~inlierIdx, 1);
y_outliers = data(~inlierIdx, 2);

% 使用内群点拟合 RANSAC 线性模型
p_ransac = polyfit(x_inliers, y_inliers, 1);
x_fit = linspace(min(x_data), max(x_data), 100);
y_fit_ransac = polyval(p_ransac, x_fit);

% 使用最小二乘法拟合线性模型
p_ls = polyfit(x_data, y_data, 1);
y_fit_ls = polyval(p_ls, x_fit);

% 图像右侧 - RANSAC 和最小二乘法拟合结果
subplot(1, 2, 2);
hold on;
scatter(x_outliers, y_outliers, 'filled', 'r');
scatter(x_inliers, y_inliers, 'filled', 'b');
plot(x_fit, y_fit_ransac, 'b', 'LineWidth', 2);
plot(x_fit, y_fit_ls, 'g--', 'LineWidth', 2);
title('RANSAC和LS拟合结果', 'fontSize', 13);
xlabel('X', 'fontSize', 13);
ylabel('Y', 'fontSize', 13);
legend({'离群点', '内群点', 'RANSAC拟合线', 'LS拟合线'}, 'Location', 'northwest', 'box', 'off', 'fontSize', 12);
axis equal;
axis off
hold off;

% RANSAC 拟合线性模型函数
function [M, inlierIdx] = ransacfitline(x, t, d, k)
    % RANSAC algorithm for line fitting.
    % x - points
    % t - distance threshold
    % d - number of points required to assert model validity
    % k - max iterations

    n = size(x, 2);
    bestM = [];
    bestInlierIdx = [];
    bestInliers = 0;
    iter = 0;

    while iter < k
        % Randomly select 2 points to fit line
        indices = randperm(n, 2);
        sample = x(:, indices);

        % Fit line
        v = sample(:, 2) - sample(:, 1);
        v = v / norm(v);
        normal = [-v(2), v(1)];

        dists = abs(normal * (x - sample(:, 1)));
        inlierIdx = dists < t;

        % Check if we have the best model so far
        numInliers = sum(inlierIdx);
        if numInliers > bestInliers && numInliers >= d
            bestM = [sample(:, 1), sample(:, 1) + v];
            bestInliers = numInliers;
            bestInlierIdx = inlierIdx;
        end

        iter = iter + 1;
    end

    M = bestM;
    inlierIdx = bestInlierIdx;
end



% % rng(2)
% close all
% num_points = 300; % Number of random points
% xc = 3; % Center x-coordinate
% yc = 5; % Center y-coordinate
% width = 2; % Rectangle width
% height = 2; % Rectangle height
% theta = pi / 6; % Rotation angle (30 degrees)
% 
% X = generate_random_points_in_rotated_rectangle(num_points, xc, yc, width, height, theta);
% 
% % 数据中心化
% X_centered = X - mean(X);
% 
% % 计算协方差矩阵
% C = cov(X_centered);
% 
% % 计算特征值和特征向量
% [V, D] = eig(C);
% 
% % 按特征值降序排序
% [~, order] = sort(diag(D), 'descend');
% V = V(:, order);
% 
% % 投影到主成分上
% X_pca = X_centered * V;
% 
% % 计算投影范围
% range_pc1 = [min(X_pca(:, 1)), max(X_pca(:, 1))];
% range_pc2 = [min(X_pca(:, 2)), max(X_pca(:, 2))];
% 
% % 主成分方向
% pc1_dir = V(:, 1);
% pc2_dir = V(:, 2);
% 
% % 调整主成分线段的长度
% pc1_line = [[xc yc]; [xc yc] + pc1_dir' * range_pc1(2)];
% pc2_line = [[xc yc]; [xc yc] + pc2_dir' * range_pc2(2)];
% 
% % 创建图形窗口
% figure;
% 
% % 绘制变换前的数据和主成分方向
% subplot(1, 2, 1);
% hold on;
% meanX = mean(X_transformed);
% pc1 = V(:, 1)' * sqrt(D(1, 1));
% pc2 = V(:, 2)' * sqrt(D(2, 2));
% scatter(X(:, 1), X(:, 2), 'filled', 'b', 'DisplayName', '数据点');
% h1 = plot(pc1_line(:, 1), pc1_line(:, 2), 'r-', 'LineWidth', 2);
% h2 = plot(pc2_line(:, 1), pc2_line(:, 2), 'g-', 'LineWidth', 2);
% legend([h1 h2], {'主成分 1', '主成分 2'}, 'box', 'off', 'fontSize', 12); % 显示图例
% title('原始数据', 'FontSize', 13);
% xlabel('X', 'FontSize', 13);
% ylabel('Y', 'FontSize', 13);
% axis equal;
% grid on;
% 
% % 绘制变换后的数据
% subplot(1, 2, 2);
% scatter(X_pca(:, 1), X_pca(:, 2), 'filled', 'b');
% title('PCA变换后的数据', 'FontSize', 13);
% xlabel('主成分 1', 'FontSize', 13);
% ylabel('主成分 2', 'FontSize', 13);
% axis equal;
% grid on;
% 
% exportgraphics(gcf, "figure/f18.png", 'Resolution', 600, 'BackgroundColor', 'w');
% 
% function points = generate_random_points_in_rotated_rectangle(num_points, xc, yc, width, height, theta)
%     % Generate points in a rotated rectangle
% 
%     % Define half-width and half-height for convenience
%     half_width = width / 2;
%     half_height = height / 2;
% 
%     % Generate random points within the [-0.5, 0.5] range
%     random_points = (rand(num_points, 2) - 0.5) .* [width, height];
% 
%     % Create rotation matrix
%     rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
% 
%     % Rotate points around the origin (0, 0)
%     rotated_points = (rotation_matrix * random_points')';
% 
%     % Translate points to the center (xc, yc)
%     translated_points = rotated_points + [xc, yc];
% 
%     % Output the generated points
%     points = translated_points;
% 
%     % Define rectangle corners before rotation
%     % corners = [-half_width, -half_height; 
%     %            half_width, -half_height; 
%     %            half_width,  half_height; 
%     %            -half_width, half_height];
% 
%     % Rotate corners around the origin
%     % rotated_corners = (rotation_matrix * corners')';
% 
%     % Translate corners to the center (xc, yc)
%     % translated_corners = rotated_corners + [xc, yc];
% 
%     % Plotting
%     % figure;
%     % hold on;
%     % Plot the random points
%     % plot(translated_points(:, 1), translated_points(:, 2), 'bo');
% 
%     % Plot the rectangle
%     % fill(translated_corners([1:4, 1], 1), translated_corners([1:4, 1], 2), 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'k');
% 
%     % axis equal;
%     % title('Random Points in Rotated Rectangle');
%     % xlabel('X');
%     % ylabel('Y');
%     % hold off;
% end


% % 清空环境
% clc; clear; close all;
% 
% % 生成随机点
% N = 100;
% points = rand(N, 2);
% 
% % 计算凸包
% k = convhull(points(:, 1), points(:, 2));
% 
% % 提取凸包顶点
% hull_points = points(k, :);
% 
% % 初始化最小面积和矩形
% min_area = inf;
% min_rect = [];
% 
% % 计算凸包边数
% num_edges = length(k);
% 
% % 旋转卡壳
% for i = 1:num_edges - 1
%     % 当前边
%     edge = hull_points(i + 1, :) - hull_points(i, :);
% 
%     % 单位向量
%     unit_edge = edge / norm(edge);
% 
%     % 垂直向量
%     perp_edge = [-unit_edge(2), unit_edge(1)];
% 
%     % 旋转矩阵
%     R = [unit_edge; perp_edge];
% 
%     % 投影到旋转坐标系
%     proj = hull_points * R';
% 
%     % 计算最小外接矩形
%     min_x = min(proj(:, 1));
%     max_x = max(proj(:, 1));
%     min_y = min(proj(:, 2));
%     max_y = max(proj(:, 2));
% 
%     % 矩形顶点在旋转坐标系中的坐标
%     rect = [min_x, min_y;
%             max_x, min_y;
%             max_x, max_y;
%             min_x, max_y];
% 
%     % 转换到原始坐标系
%     rect = rect * R;
% 
%     % 计算矩形面积
%     area = (max_x - min_x) * (max_y - min_y);
% 
%     % 更新最小面积矩形
%     if area < min_area
%         min_area = area;
%         min_rect = rect;
%     end
% end
% 
% % 绘制点
% fig = figure;
% scatter(points(:, 1), points(:, 2), 'filled', 'b');
% hold on;
% 
% % 绘制凸包
% plot(hull_points(:, 1), hull_points(:, 2), 'r-', 'LineWidth', 2);
% 
% % 绘制最小外接矩形
% plot([min_rect(:, 1); min_rect(1, 1)], [min_rect(:, 2); min_rect(1, 2)], 'g-', 'LineWidth', 2);
% 
% % 设置图形属性
% axis equal;
% axis off
% xlabel('X');
% ylabel('Y');
% legend('点集', '凸包', '最小外接矩形', 'box', 'off', 'fontSize', 12);
% hold off;
% exportgraphics(gcf, "figure/f19.png", 'Resolution', 600);