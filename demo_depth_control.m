function depth_pid_control()
    close();
    % ========== 参数设置 ==========
    Ts = 0.1;             % 采样时间
    T_end = 300;          % 总仿真时间
    t = 0:Ts:T_end;
    N = length(t);

    % 初始状态
    x = zeros(12,1);
    x(1) = 1.5;     % 初始前进速度
    spd_ref = 1.25;  % 期望速度
    ui = [0; 0; 0; 0; 0; 1100];  % 固定推进器转速

    % 深度PID 参数
    z_Kp = 0.05;
    z_Ki = 0;
    z_Kd = 0.4;

    % 初始化数据记录
    Z = zeros(1,N);
    % SPD = zeros(1,N);
    U = zeros(1,N);  % 控制输入 delta_b (deg)

    % 清除 PID 控制器历史
    pid_controller('dep', 0, 0, Ts, z_Kp, z_Ki, z_Kd, true);

    % ========== 主循环 ==========
    for k = 1:N
        % 当前状态
        z = x(9);  %深度
        spd = x(1);%速度

        % 深度控制
        if k*Ts < 50 
            z_ref = 0;
            delta_b = pid_controller('dep',z_ref, z, Ts, z_Kp, z_Ki, z_Kd); % PID 控制计算 delta_b（输出为弧度)
        elseif 50<=k*Ts %&& k*Ts<200
            z_ref = 10;
            delta_b = pid_controller('dep',z_ref, z, Ts, z_Kp, z_Ki, z_Kd); % PID 控制计算 delta_b（输出为弧度）
        % else
        %     z_ref = 0;
        %     delta_b = pid_controller('dep',z_ref, z, Ts, z_Kp, z_Ki, z_Kd); % PID 控制计算 delta_b（输出为弧度）
        end

        % 限幅（+-20度）
        delta_b = max(min(delta_b, deg2rad(20)), deg2rad(-20));

        % 设置输入
        ui(2) = delta_b;   % port and starboard stern plane
        ui(3) = delta_b;   % top and bottom bow plane
        ui(4) = -delta_b;  % port bow
        ui(5) = -delta_b;  % starboard bow

        % 状态更新
        xdot = auv(x, ui);
        x = x + xdot * Ts;

        % 记录数据
        Z(k) = z;
        SPD(k) = spd;
        U(k) = rad2deg(delta_b);
    end

    % ========== 画图 ==========
    figure();
    set(gcf,'unit','normalize','position',[0.6 0.3 0.3 0.6])
    subplot(3,1,1);
    plot(t, Z, 'b', 'LineWidth', 2); hold on;
    yline(z_ref, 'r--', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('深度 z (m)');
    title('深度闭环控制阶跃响应');
    grid on;

    % 控制输入
    subplot(3,1,2);
    plot(t, U, 'm', 'LineWidth', 2);
    xlabel('时间 (s)');
    ylabel('艏翼角 \delta_b (deg)');
    title('控制输入：艏翼角变化');
    grid on;

    % 速度
    subplot(3,1,3);
    plot(t, SPD, 'b', 'LineWidth', 2);
    yline(spd_ref, 'r--', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('速度');
    title('AUV前向速度');
    grid on;

    % ========== 控制性能指标 ==========
    stepinfo_report(Z, t, z_ref);
end

