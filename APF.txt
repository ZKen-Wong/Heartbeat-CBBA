function ui = APF(x, goal, obstacle_list, pid_vector)
%% PID 由主程序来初始化
yaw_kp = pid_vector(1);
yaw_ki =pid_vector(2);
yaw_kd =pid_vector(3);
pitch_kp = pid_vector(4);
pitch_ki = pid_vector(5);
pitch_kd = pid_vector(6);
dt = pid_vector(7);
%% target and obstacles
obstacles = vertcat(obstacle_list.pos);
d0 = 80;             % 排斥力生效距离
eta = 50;           % 排斥力系数
ksi = 1;             % 吸引力系数
pos = x(7:9)';
pitch = x(11);
yaw = x(12);

%% 吸引力
F_att = ksi * (goal - pos) / norm(goal - pos);

%% 排斥力
F_rep = [0, 0, 0];
for i = 1:size(obstacles,1)
    obs = obstacles(i,:);
    dis = pos - obs;
    dis(3) = 3 * dis(3);
    dist = norm(dis);
    if dist < d0 && dist > 1e-3
        F_rep = F_rep + eta * (1/dist - 1/d0) / dist * dis;
    end
end

%% 合力 + 限制z轴速度
F_total = F_att + F_rep;

%% 转换为目标方向角
F_total = 30*F_total/norm(F_total);
target_yaw = atan2(F_total(2), F_total(1));
target_pitch = atan2(-F_total(3), norm(F_total(1:2)));  % z向下为正

%% 控制器输出
if (abs(target_yaw - yaw) > pi)
    target_yaw = target_yaw - sign(target_yaw) * 2*pi;
end
% if round(target_yaw) == round(yaw)
%     target_yaw = target_yaw + 0.5*pi;
% end
delta_r = -pid_controller('yaw_main', target_yaw, yaw, dt, yaw_kp, yaw_ki, yaw_kd);
delta_b = -pid_controller('pitch_main', target_pitch, pitch, dt, pitch_kp, pitch_ki, pitch_kd);
% 限幅
if (abs(delta_r) > 10*pi/180)
    delta_r = sign(delta_r)*10*pi/180;
end
if (abs(delta_b) > 20)
    delta_b = sign(delta_b)*10*pi/180;
end

%% rpm控制，终点减速，速度不足就加速
w = [1, 1, 0.5];  % xyz 方向的权重（z方向较轻）
dif = (pos - goal) .* sqrt(w); 
goal_dist = norm(dif);
if goal_dist < 20
    v_scale = 1;  % 线性降速：从1.0降至0.5
else
    if x(1) < 0.8
        delta_r = 0;
        delta_b = 0;
    end
    v_scale = 1;
end

%% 输入组合
n = 1100;
ui(1) = delta_r;   % rudder
ui(2) = delta_b;   % port and starboard stern plane
ui(3) = delta_b;   % top and bottom bow plane
ui(4) = -delta_b;  % port bow
ui(5) = -delta_b;  % starboard bow
ui(6) = v_scale * n;         % rpm