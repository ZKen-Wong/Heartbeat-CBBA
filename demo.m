function simpleAUVControl()
    % 仿真参数
    dt = 0.05;             % 时间步长 (秒)
    t_end = 100;            % 仿真总时长 (秒)
    t = 0:dt:t_end;

    % 初始状态
    x = zeros(12,1);       % 状态向量初值
    x(1) = 0.5;
    n_fixed = 1500;         % 固定 RPM

    % 初始化输入
    delta_b = 0;           % 初始艏翼角度 (rad)
    ui = [0; 0; delta_b; 0; 0; n_fixed];

    % 记录数据
    states = zeros(length(t),12);
    inputs = zeros(length(t));

    % 创建图形窗口并设置键盘输入
    fig = figure('Name','AUV Real-Time Control','KeyPressFcn',@keyPressFcn);
    xlim([0 100]);
    key_pressed = ' '; % 初始化按键状态

    % 回调函数：捕获键盘输入
    function keyPressFcn(~,event)
        key_pressed = event.Character;
    end

    xlabel('Time (s)'); ylabel('Pitch angle (deg)');
    grid on; hold on;

    disp('按数字键8增大艏翼角，数字键2减小艏翼角，按ESC退出仿真');

    for k = 1:length(t)
        switch key_pressed
            case '6' % 数字键8
                delta_b = delta_b + deg2rad(4);
                % disp(rad2deg(delta_b));
                key_pressed = ' '; % 重置按键状态
            case '4' % 数字键2
                delta_b = delta_b - deg2rad(4);
                % disp(rad2deg(delta_b));
                key_pressed = ' '; % 重置按键状态
            case char(27) % ESC退出
                disp('退出仿真！');
                break;
        end

        % 输入限幅
        delta_b = min(max(delta_b,-20*pi/180),20*pi/180);
        ui(1) = delta_b;
        % ui(2) = delta_b;
        % ui(3) = delta_b;

        % 状态更新（欧拉积分）
        xdot = auv(x,ui);
        x = x + xdot*dt;

        % 存储状态
        states(k,:) = x';
        inputs(k) = rad2deg(delta_b);

        % 实时绘制 pitch 姿态角
        plot(t(k),rad2deg(x(12)),'bo'); drawnow;
        plot(t(k),rad2deg(delta_b),'ro'); drawnow;

        % 打印状态（仅前6个或全部）
        clc;
        fprintf('仿真时间: %.2f 秒\n', t(k));
        fprintf('艏翼角 delta_b: %.2f deg\n', rad2deg(delta_b))
        labels = ["u","v","w","p","q","r","x","y","z","phi","theta","psi"];
        for i = 1:12
            val = x(i);
            if i >= 10
                val = rad2deg(val); % 姿态角用角度显示
            end
            fprintf('%6s: %8.4f\n', labels(i), val);
        end

        pause(dt/2); % 控制仿真速度
    end

    % 后处理绘图
    figure;
    plot(t(1:k),rad2deg(states(1:k,12)),'b','LineWidth',2);hold on;
    plot(t(1:k),inputs(1:k),'r','LineWidth',2);
    xlabel('Time (s)');
    ylabel('Pitch Angle (deg)');
    title('Pitch Angle vs Time');
    grid on;
end
