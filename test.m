rng(1)
close all
num_points = 1000; % Number of random points
xc = 0; % Center x-coordinate
yc = 0; % Center y-coordinate
width = 2; % Rectangle width
height = 2; % Rectangle height
theta = pi / 6; % Rotation angle (30 degrees)

points = generate_random_points_in_rotated_rectangle(num_points, xc, yc, width, height, theta);


% 计算数据的均值
data_mean = mean(points);

% 将数据中心化
centered_data = points - data_mean;

% 计算协方差矩阵
cov_matrix = cov(centered_data);

% 进行特征值分解
[eigenvectors, eigenvalues] = eig(cov_matrix);

% 提取特征值和特征向量
[~, idx] = sort(diag(eigenvalues), 'descend'); % 按降序排序特征值
principal_components = eigenvectors(:, idx);

% 将数据投影到主成分方向
transformed_data = centered_data * principal_components;

% 绘制原始数据点云
fig = figure("Name", "figure");

subplot(1, 2, 1);
scatter(points(:,1), points(:,2), 10, 'filled', 'color', 'b');
hold on;
title('Original 2D Points');
xlabel('X');
ylabel('Y');
grid on;
axis equal;

% 绘制主成分方向
for i = 1:2
    % 绘制每个主成分方向
    line([data_mean(1), data_mean(1) + principal_components(1,i)], ...
         [data_mean(2), data_mean(2) + principal_components(2,i)], ...
         'Color', 'r', 'LineWidth', 2, 'LineStyle', '--');
end

% 绘制中心点
plot(data_mean(1), data_mean(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
hold off;
legend({'Data Points', 'Principal Components'},'box', 'off');

% 绘制转换后的数据点云
subplot(1, 2, 2);
scatter(transformed_data(:,1), transformed_data(:,2), 10, 'filled', 'color', 'b');
title('PCA Transformed 2D Points');
xlabel('Principal Component 1');
ylabel('Principal Component 2');
axis equal;
grid on;

% 设置整个图形窗口的属性
set(gcf, 'Position', [100, 100, 800, 400]);

exportgraphics(fig, "figure/f18.png", 'Resolution', 600);

function points = generate_random_points_in_rotated_rectangle(num_points, xc, yc, width, height, theta)
    % Generate points in a rotated rectangle
    
    % Define half-width and half-height for convenience
    half_width = width / 2;
    half_height = height / 2;
    
    % Generate random points within the [-0.5, 0.5] range
    random_points = (rand(num_points, 2) - 0.5) .* [width, height];
    
    % Create rotation matrix
    rotation_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    
    % Rotate points around the origin (0, 0)
    rotated_points = (rotation_matrix * random_points')';
    
    % Translate points to the center (xc, yc)
    translated_points = rotated_points + [xc, yc];
    
    % Output the generated points
    points = translated_points;
    
end
