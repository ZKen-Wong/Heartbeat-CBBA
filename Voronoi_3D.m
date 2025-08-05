% 生成随机的三维数据点
points = rand(30, 3); % 30个随机点

% 创建三维 Delaunay 三角剖分
DT = delaunayTriangulation(points);

% 计算 Voronoi 图的顶点和单元索引
[V, R] = voronoiDiagram(DT);

% 绘制 Voronoi 单元格
figure('Color', 'w');
hold on; grid on;
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Voronoi Diagram');

for i = 1:length(R)
    % 提取每个 Voronoi 单元的顶点索引
    vertIndices = R{i};
    
    % 排除无穷远的单元格
    if all(vertIndices ~= 1)
        % 计算凸包 (单元格的外轮廓面)
        K = convhull(V(vertIndices, :));
        
        % 使用 trisurf 绘制单元格
        trisurf(K, V(vertIndices,1), V(vertIndices,2), V(vertIndices,3), ...
            'FaceColor', rand(1,3), 'FaceAlpha', 0.5, 'EdgeColor', 'k');
    end
end

% 在图中绘制原始数据点
scatter3(points(:,1), points(:,2), points(:,3), ...
    40, 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');

view(3);
