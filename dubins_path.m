function [path3D, totalLength] = dubins_path(startPose3D, goalPose3D, turningRadius, stepSize, maxClimbRate)
% DUBINS3D_PATH 生成3D Dubins路径（XY平面为Dubins路径，Z轴受限于爬升率）
%
% 输入：
%   startPose3D = [x0, y0, z0, yaw0]   % 起点坐标和偏航角（rad）
%   goalPose3D  = [xg, yg, zg, yawg]   % 终点坐标和偏航角（rad）
%   turningRadius                    % 最小转弯半径
%   stepSize                         % 插值步长（默认 0.5）
%   maxClimbRate                     % 最大单位长度爬升/下降速率（默认 0.3）
%
% 输出：
%   path3D = Nx3 矩阵，表示 [x y z] 的路径点
%   totalLength = 路径长度（欧氏长度）

    if nargin < 4
        stepSize = 0.5;
    end
    if nargin < 5
        maxClimbRate = 0.2; % 最大爬升/下降速率（单位：m/每m水平）
    end

    % test 参数
    % startPose3D = [0, 0, 0, 0*pi/180];
    % goalPose3D = [10, 10, -30, 45*pi/180];
    % turningRadius = 15;
    % maxClimbRate = 0.2;

    % 使用 stateSpaceDubins 创建 2D 路径空间
    bounds = [min([startPose3D(1), goalPose3D(1)])-100, max([startPose3D(1), goalPose3D(1)])+100; 
              min([startPose3D(2), goalPose3D(2)])-100, max([startPose3D(2), goalPose3D(2)])+100; 
              -pi, pi];
    dubinsSpace = stateSpaceDubins(bounds);
    dubinsSpace.MinTurningRadius = turningRadius;

    % 创建 navPath 对象并添加起点终点
    pathobj = navPath(dubinsSpace);
    waypoints = [startPose3D(1:2), startPose3D(4); 
                 goalPose3D(1:2),  goalPose3D(4)];
    append(pathobj, waypoints);

    % 插值以获取更平滑路径
    interpolate(pathobj, ceil(pathLength(pathobj)/stepSize));
    poses2D = pathobj.States; % [x y yaw]
    N = size(poses2D,1);

    % 水平路径长度
    d_xy = sum(vecnorm(diff(poses2D(:,1:2)), 2, 2));
    dz = goalPose3D(3) - startPose3D(3);
    slope = abs(dz / d_xy);

    % 判断是否超过爬升率
    if slope <= maxClimbRate
        % 正常线性插值
        z = linspace(startPose3D(3), goalPose3D(3), N)';
        % 3D 路径
        path3D = [poses2D(:,1:2), z];
    else
        % 计算需要转多少圈，多延长0.1圈以防万一
        n_turns = abs(dz / maxClimbRate-d_xy) / (2*pi*turningRadius)-0.5;
        N = ceil(abs(dz / maxClimbRate-d_xy) / stepSize);
        theta = linspace(startPose3D(4), 2*pi*n_turns + startPose3D(4), N)';

        % 计算下降段路径
        R = turningRadius;
        x_spiral = R * sin(theta)-R*sin(theta(1)); % -R 保持0点
        y_spiral = R * cos(theta)-R*cos(theta(1));

        % 将螺旋叠加到路径起点
        offsetX = poses2D(1,1);
        offsetY = poses2D(1,2);
        % 合并下降path
        spiralXY = [x_spiral + offsetX, y_spiral + offsetY];
        
        %从下降后的点重新计算路径
        new_startPose3D = [spiralXY(end,1:2), -theta(end)];
        pathobj = navPath(dubinsSpace);
        waypoints = [new_startPose3D; 
                 goalPose3D(1:2),  goalPose3D(4)];
        append(pathobj, waypoints);

        % 插值以获取更平滑路径
        interpolate(pathobj, ceil(pathLength(pathobj)/stepSize));
        poses2D = pathobj.States; % [x y yaw]
        poses2D = [spiralXY;poses2D(:,1:2)];
        N = size(poses2D,1);

        % z 插值
        z = linspace(startPose3D(3), goalPose3D(3), N)';

        % 合并路径
        path3D = [poses2D(:,1:2), z];

        % 水平路径长度
        d_xy = sum(vecnorm(diff(path3D(:,1:2)), 2, 2));
        dz = goalPose3D(3) - startPose3D(3);
        slope = abs(dz / d_xy);
        
    end

    

    % 计算路径长度（欧氏距离）
    diffs = diff(path3D);
    dists = vecnorm(diffs, 2, 2);
    totalLength = sum(dists);
end