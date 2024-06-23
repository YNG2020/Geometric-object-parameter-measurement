% 清空环境
clc; clear; close all;

% 生成随机点
N = 100;
points = rand(N, 2);

% 计算凸包
k = convhull(points(:, 1), points(:, 2));

% 提取凸包顶点
hull_points = points(k, :);

% 初始化最小面积和矩形
min_area = inf;
min_rect = [];

% 计算凸包边数
num_edges = length(k);

% 旋转卡壳
for i = 1:num_edges - 1
    % 当前边
    edge = hull_points(i + 1, :) - hull_points(i, :);
    
    % 单位向量
    unit_edge = edge / norm(edge);
    
    % 垂直向量
    perp_edge = [-unit_edge(2), unit_edge(1)];
    
    % 旋转矩阵
    R = [unit_edge; perp_edge];
    
    % 投影到旋转坐标系
    proj = hull_points * R';
    
    % 计算最小外接矩形
    min_x = min(proj(:, 1));
    max_x = max(proj(:, 1));
    min_y = min(proj(:, 2));
    max_y = max(proj(:, 2));
    
    % 矩形顶点在旋转坐标系中的坐标
    rect = [min_x, min_y;
            max_x, min_y;
            max_x, max_y;
            min_x, max_y];
    
    % 转换到原始坐标系
    rect = rect * R;
    
    % 计算矩形面积
    area = (max_x - min_x) * (max_y - min_y);
    
    % 更新最小面积矩形
    if area < min_area
        min_area = area;
        min_rect = rect;
    end
end

% 绘制点
fig = figure;
scatter(points(:, 1), points(:, 2), 'filled', 'b');
hold on;

% 绘制凸包
plot(hull_points(:, 1), hull_points(:, 2), 'r-', 'LineWidth', 2);

% 绘制最小外接矩形
plot([min_rect(:, 1); min_rect(1, 1)], [min_rect(:, 2); min_rect(1, 2)], 'g-', 'LineWidth', 2);

% 设置图形属性
axis equal;
axis off
xlabel('X');
ylabel('Y');
legend('点集', '凸包', '最小外接矩形', 'box', 'off', 'fontSize', 12);
hold off;
exportgraphics(gcf, "figure/f19.png", 'Resolution', 600);