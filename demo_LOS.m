function demo_LOS()
    close();
    clear();
    %% Initial
    dt = 0.1; T = 600; t = 0:dt:T;  %仿真时间轴
    s = linspace(0, 400, length(t));
    
    % 状态初始化 state initial
    x = zeros(12,1);
    start_pos = [0, 0, 0, 0*pi/180]; % x y z yaw
    x(1) = 1.35; % 初始速度 u ≠ 0
    x(7) = start_pos(1); % 重叠x初始坐标
    x(8) = start_pos(2); % 重叠y初始坐标
    x(12) = start_pos(4); % 初始指向

    % 路径 path
    % path = [s; 6*cos(0.1*s); -0.2*s];  % 3D path: x, y = sin(x), z下降
    % Diamet = 20;
    % path = [Diamet*sin(0.1*s)+Diamet; Diamet*cos(0.1*s)+Diamet;  -0.1*s];   % 3D path: 转圈下降
    %生成任意两点之间的 dubins path
    goal = [10, 100, -80, 0*pi/180];
    R_min = 25;
    maxClimbRate = 0.2;
    step = 0.5;
    [path, L] = dubins_path(start_pos, goal, R_min, step, maxClimbRate);
    
    % check path(test)
    % plot3(path(:,1), path(:,2), path(:,3), 'b-', 'LineWidth', 2);
    % grid on; axis equal; xlabel('X'); ylabel('Y'); zlabel('Z');
    % title(['3D Dubins Path, Length = ', num2str(L, '%.2f'), ' m']);
    
    % Los 参数
    L = 10;       % 前视距离
    K_yaw = -3;
    states = zeros(length(t), 12);

    % 深度PID控制
    z_Kp = 0.1;
    z_Ki = 0;
    z_Kd = 0.4;
    pid_controller('dep', 0, 0, dt, z_Kp, z_Ki, z_Kd, true);

    % plot initial
    figure;
    set(gcf,'unit','normalize','position',[0.6 0.3 0.35 0.3])
    plot3(path(:,1), path(:,2), path(:,3), 'r','LineWidth',1.5); hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    legend('Path','AUV trajectory');
    title('AUV LOS Path Following (Yaw+Pitch)');
    grid on; axis equal;
    % plot AUV
    r = 1;
    [X, Y, Z] = sphere(20);  % 分辨率为 20
    auvPlot = surf(r*X, r*Y, r*Z, 'FaceColor', 'b', 'EdgeColor', 'none');
    close_p = surf(r*X, r*Y, r*Z, 'FaceColor', 'r', 'EdgeColor', 'none');
    % los = surf(r*X, r*Y, r*Z, 'FaceColor', 'g', 'EdgeColor', 'none');

    %% Simulation

    for i = 1:length(t)
        % 当前状态
        x_pos = x(7); y_pos = x(8); z_pos = x(9);
        theta = x(11); psi = x(12); 

        % 找到最近点
        dists = sqrt((path(:,1) - x_pos).^2 + (path(:,2) - y_pos).^2 + (path(:,3) - z_pos).^2);
        [~, closest_idx] = min(dists);

        % 找 LOS 目标点
        los_idx = closest_idx;
        for j = closest_idx:length(path)
            dx = path(j,1) - path(closest_idx,1);
            dy = path(j,2) - path(closest_idx,2);
            if sqrt(dx^2 + dy^2) >= L
                los_idx = j;
                break;
            end
        end

        xt = path(los_idx, 1);
        yt = path(los_idx, 2);
        zt = path(los_idx, 3);

        % 横向误差 d（signed）
        d = sqrt((path(closest_idx,1) - x_pos)^2 + (path(closest_idx,2) - y_pos)^2);
        l_or_r = atan2(path(closest_idx,2) - y_pos, path(closest_idx,1) - x_pos);
        d = sign(l_or_r) * d;

        % 计算航向误差 alpha
        alpha = atan2(yt - y_pos, xt - x_pos) - psi;
        alpha = wrapToPi(alpha);

        % Yaw控制量
        flag_rudder = 2 * sin(alpha) + 0.01 * d;

        % Pitch控制（目标z）
        delta_b = pid_controller('dep',zt, z_pos, dt, z_Kp, z_Ki, z_Kd); % PID 控制计算 delta_b（输出为弧度）

        % 控制映射
        delta_r = K_yaw * flag_rudder;

        % 推进控制（如果离终点足够远）
        %goal_dist = norm([xt - x_pos, yt - y_pos, zt - z_pos]);
        n = 1100;
        ui(1) = delta_r;   % rudder
        ui(2) = delta_b;   % port and starboard stern plane
        ui(3) = delta_b;   % top and bottom bow plane
        ui(4) = -delta_b;  % port bow
        ui(5) = -delta_b;  % starboard bow
        ui(6) = n;         % rpm

        % 状态更新
        xdot = auv(x, ui);
        x = x + dt * xdot;
        states(i,:) = x';

        % 动画更新
        if mod(i,50) == 0
            set(auvPlot,'XData',r*X+x(7),'YData',r*Y+x(8),"ZData",r*Z+x(9));
            set(close_p,'XData',r*X+path(closest_idx,1),'YData',r*Y+path(closest_idx,2),"ZData",r*Z+path(closest_idx,3));
            % set(los,'XData',xt,'YData',yt,"ZData",zt);
            % plot3(x(7),x(8),x(9),'b');
            drawnow;
        end

    end

    %% 绘图 Plot
    
    % plot3(path(:,1), path(:,2), path(:,3), 'r','LineWidth',1.5); hold on;
    plot3(states(:,7), states(:,8), states(:,9), 'b','LineWidth',2);
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    legend('Path','AUV','Closest Point','AUV trajectory');
    % title('AUV LOS Path Following (Yaw+Pitch)');
    % grid on; axis equal;
end
