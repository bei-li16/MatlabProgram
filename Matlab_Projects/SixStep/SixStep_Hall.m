%% 有 Hall 传感器的六步换向控制（完整代码）
clc; clear; close all;

% 电机参数
PolePairs = 4;       % 极对数
Vdc = 24;            % 直流母线电压 (V)
R = 0.1;             % 相电阻 (Ω)
L = 0.001;           % 相电感 (H)
Ke = 0.01;           % 反电势系数 (V/(rad/s))
J = 0.001;           % 转动惯量 (kg·m²)
B = 0.001;           % 阻尼系数 (N·m·s/rad)
TL = 0;              % 负载转矩 (N·m)

% 仿真参数
Ts = 1e-6;           % 仿真步长 (s)
Tfinal = 0.1;        % 总仿真时间 (s)
t = 0:Ts:Tfinal;

% 初始化变量
theta_e = zeros(size(t));    % 电角度 (rad)
omega_e = zeros(size(t));    % 电角速度 (rad/s)
Hall_State = zeros(size(t)); % Hall 状态 (0-5)
Ia = zeros(size(t));         % 相电流 (A)
Ib = zeros(size(t));
Ic = zeros(size(t));
Te = zeros(size(t));         % 电磁转矩 (N·m)

% 六步换向表 (根据 Hall 状态选择导通相)
CommutationTable = [ 
    1, 0, -1;   % Hall 状态 0: A+ B- C off
    1, -1, 0;   % Hall 状态 1: A+ C- B off
    0, -1, 1;   % Hall 状态 2: B- C+ A off
    -1, 0, 1;   % Hall 状态 3: A- C+ B off
    -1, 1, 0;   % Hall 状态 4: A- B+ C off
    0, 1, -1;   % Hall 状态 5: B+ C- A off
];

% 主循环
for k = 2:length(t)
    % 模拟电机旋转 (电角度动态更新)
    omega_e(k) = omega_e(k-1) + (Te(k-1) - B*omega_e(k-1) - TL)/J * Ts;
    theta_e(k) = theta_e(k-1) + omega_e(k) * Ts;
    
    % 生成 Hall 信号 (每60度电角度变化一次)
    Hall_State(k) = floor(mod(theta_e(k), 2*pi) / (pi/3));
    
    % 根据 Hall 状态选择导通相
    State = mod(Hall_State(k), 6) + 1; % 映射到 1-6
    [Sa, Sb, Sc] = deal(CommutationTable(State, 1), ...
                     CommutationTable(State, 2), ...
                     CommutationTable(State, 3));
    
    % 生成线电压
    Vab = Vdc * (Sa - Sb);
    Vbc = Vdc * (Sb - Sc);
    Vca = Vdc * (Sc - Sa);
    
    % 电机动态方程（简化三相模型）
    dIa = (Vab - R*Ia(k-1) - Ke*omega_e(k)) / L * Ts;
    dIb = (Vbc - R*Ib(k-1) - Ke*omega_e(k)) / L * Ts;
    dIc = (Vca - R*Ic(k-1) - Ke*omega_e(k)) / L * Ts;
    
    Ia(k) = Ia(k-1) + dIa;
    Ib(k) = Ib(k-1) + dIb;
    Ic(k) = Ic(k-1) + dIc;
    
    % 计算电磁转矩
    Te(k) = (Ke * (Ia(k) + Ib(k) + Ic(k))) / PolePairs;
end

% 绘图
figure;
subplot(3,1,1);
plot(t, theta_e, 'b'); title('电角度 (rad)');
subplot(3,1,2);
stairs(t, Hall_State, 'r'); title('Hall 状态');
subplot(3,1,3);
plot(t, Ia, 'g', t, Ib, 'b', t, Ic, 'r'); 
title('相电流 (A)'); legend('Ia', 'Ib', 'Ic');