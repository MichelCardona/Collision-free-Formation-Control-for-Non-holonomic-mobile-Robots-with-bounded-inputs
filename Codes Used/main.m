clear all
close all
clc

%%
rosshutdown
rosinit("http://192.168.50.222:11311")
bot_Sub{1} = rossubscriber("/vrpn_client_node/Turtle_1/pose","geometry_msgs/PoseStamped");
bot_Sub{2} = rossubscriber("/vrpn_client_node/Turtle_2/pose","geometry_msgs/PoseStamped");
bot_Sub{3} = rossubscriber("/vrpn_client_node/Turtle_3/pose","geometry_msgs/PoseStamped");
bot_Sub{4} = rossubscriber("/vrpn_client_node/Turtle_4/pose","geometry_msgs/PoseStamped");

bot{1} = TurtleBot3_WafflePI('http://192.168.50.103:11311',eye(4,4));
bot{2} = TurtleBot3_WafflePI('http://192.168.50.106:11311',eye(4,4));
bot{3} = TurtleBot3_WafflePI('http://192.168.50.107:11311',eye(4,4));
bot{4} = TurtleBot3_WafflePI('http://192.168.50.105:11311',eye(4,4));

%% Cargar Ks
load('K1.mat')
load('K2.mat')
load('K3.mat')
load('K4.mat')
load('K5.mat')
load('K6.mat')
%%
R = 4; % Editar N de bots
p1_plot = zeros(3,58073);
p2_plot = zeros(3,58073);
p3_plot = zeros(3,58073);
p4_plot = zeros(3,58073);

pv1_plot = zeros(2,58073);
pv2_plot = zeros(2,58073);
pv3_plot = zeros(2,58073);
pv4_plot = zeros(2,58073);

U1_plot = zeros(2,58073);
U2_plot = zeros(2,58073);
U3_plot = zeros(2,58073);
U4_plot = zeros(2,58073);

D = 0.08;
p = zeros(3,R);
ph = zeros(2,R);
S = 80;
delta=0.1;
alpha = 0.05/4;
betha = 0.04/4;

alpha1 = 0.05;
betha1 = 0.04;
k=K6;
j=1;
tic;
while toc<=S
    t=toc
    t_plot(j) = t;
    ps1=size(t_plot(:,1:j));
    t1=ps1(2);
    for i=1:R
        p(:,i) = Get_Pose(bot_Sub{i});
        ph(:,i) = p(1:2,i) + D*[cos(p(3,i)); sin(p(3,i))];
    end

    x = ph(:);
    p1 = p(3,1); p2 = p(3,2); p3 = p(3,3); p4 = p(3,4);
    x1 = [x(1);x(2)]; x2 = [x(3);x(4)]; x3 = [x(5);x(6)]; x4 = [x(7);x(8)];
% w_ij(x)
   %% wx_ij = Wx_ij(Kx,xi,xj)   Pesos Wx
    % Primer agente 
    wx_12 = Wx_ij(k(1),x1(1),x2(1),delta);
    wx_13 = Wx_ij(k(2),x1(1),x3(1),delta);
    wx_14 = Wx_ij(k(3),x1(1),x4(1),delta);

    % Segundo agenete
    wx_21 = Wx_ij(k(4),x2(1),x1(1),delta);
    wx_23 = Wx_ij(k(5),x2(1),x3(1),delta);
    wx_24 = Wx_ij(k(6),x2(1),x4(1),delta);

    % Tercer agente
    wx_31 = Wx_ij(k(7),x3(1),x1(1),delta);
    wx_32 = Wx_ij(k(8),x3(1),x2(1),delta);
    wx_34 = Wx_ij(k(9),x3(1),x4(1),delta);

    % Cuarto agente
    wx_41 = Wx_ij(k(10),x4(1),x1(1),delta);
    wx_42 = Wx_ij(k(11),x4(1),x2(1),delta);
    wx_43 = Wx_ij(k(12),x4(1),x3(1),delta);
   
    %% wx_ij = Wy_ij(Ky,yi,yj)   Pesos Wy
    % Primer agente
    wy_12 = Wy_ij(k(13),x1(2),x2(2),delta);
    wy_13 = Wy_ij(k(14),x1(2),x3(2),delta);
    wy_14 = Wy_ij(k(15),x1(2),x4(2),delta);

    % Segundo agente
    wy_21 = Wy_ij(k(16),x2(2),x1(2),delta);
    wy_23 = Wy_ij(k(17),x2(2),x3(2),delta);
    wy_24 = Wy_ij(k(18),x2(2),x4(2),delta);
    
    % Tercer agente
    wy_31 = Wy_ij(k(19),x3(2),x1(2),delta);
    wy_32 = Wy_ij(k(20),x3(2),x2(2),delta);
    wy_34 = Wy_ij(k(21),x3(2),x4(2),delta);

    % Cuarto agente
    wy_41 = Wy_ij(k(22),x4(2),x1(2),delta);
    wy_42 = Wy_ij(k(23),x4(2),x2(2),delta);
    wy_43 = Wy_ij(k(24),x4(2),x3(2),delta);

        % Linea
    % U1 = [alpha1*-(wx_12) 0; 0 betha1*-(wy_14)]*[tanh(x1(1)-x2(1)); tanh(x1(2)-x4(2))];
    % U2 = [alpha1*-(wx_21) 0; 0 betha1*-(wy_23)]*[tanh(x2(1)-x1(1)); tanh(x2(2)-x3(2))];
    % U3 = [alpha1*-(wx_34) 0; 0 betha1*-(wy_32)]*[tanh(x3(1)-x4(1)); tanh(x3(2)-x2(2))];
    % U4 = [alpha1*-(wx_43) 0; 0 betha1*-(wy_41)]*[tanh(x4(1)-x3(1)); tanh(x4(2)-x1(2))];
    % 
    U1 = [alpha*-(wx_12+wx_13) 0; 0 betha*-(wy_13+wy_14)]*[tanh(x1(1)-x2(1))+tanh(x1(1)-x3(1)); tanh(x1(2)-x4(2))+tanh(x1(2)-x3(2))];
    U2 = [alpha1*-(wx_21) 0; 0 betha1*-(wy_23)]*[tanh(x2(1)-x1(1)); tanh(x2(2)-x3(2))];
    U3 = [alpha*-(wx_31+wx_34) 0; 0 betha*-(wy_32+wy_31)]*[tanh(x3(1)-x1(1))+tanh(x3(1)-x4(1)); tanh(x3(2)-x1(2))+tanh(x3(2)-x2(2))];
    U4 = [alpha1*-(wx_43) 0; 0 betha1*-(wy_41)]*[tanh(x4(1)-x3(1)); tanh(x4(2)-x1(2))];

    % U1 = [alpha*-(wx_12+wx_13) 0; 0 betha*-(wy_13+wy_14)]*[tanh(x1(1)-x2(1))+tanh(x1(1)-x3(1)); tanh(x1(2)-x4(2))+tanh(x1(2)-x3(2))];
    % U2 = [alpha*-(wx_21+wx_24) 0; 0 betha*-(wy_23+wy_24)]*[tanh(x2(1)-x1(1))+tanh(x2(1)-x4(1)); tanh(x2(2)-x3(2))+tanh(x2(2)-x4(2))];
    % U3 = [alpha*-(wx_31+wx_34) 0; 0 betha*-(wy_32+wy_31)]*[tanh(x3(1)-x1(1))+tanh(x3(1)-x4(1)); tanh(x3(2)-x1(2))+tanh(x3(2)-x2(2))];
    % U4 = [alpha*-(wx_43+wx_42) 0; 0 betha*-(wy_42+wy_41)]*[tanh(x4(1)-x3(1))+tanh(x4(1)-x2(1)); tanh(x4(2)-x2(2))+tanh(x4(2)-x1(2))];


    U1_plot(:,j) = U1;
    U2_plot(:,j) = U2;
    U3_plot(:,j) = U3;
    U4_plot(:,j) = U4;

    u = [U1;U2;U3;U4];
    
    U = reshape(u,[2 R]);

    vw_1 = Cinematica_Movil(U(:,1),p(:,1));

    bot{1}.Set_Velocity(vw_1(1),vw_1(2));

    vw_2 = Cinematica_Movil(U(:,2),p(:,2));
    bot{2}.Set_Velocity(vw_2(1),vw_2(2));

    vw_3 = Cinematica_Movil(U(:,3),p(:,3));
    bot{3}.Set_Velocity(vw_3(1),vw_3(2));

    vw_4 = Cinematica_Movil(U(:,4),p(:,4));
    bot{4}.Set_Velocity(vw_4(1),vw_4(2));

    p1_plot(:,j) = [x1; p1];
    p2_plot(:,j) = [x2; p2];
    p3_plot(:,j) = [x3; p3];
    p4_plot(:,j) = [x4; p4];

    pv1_plot(:,j) = vw_1;
    pv2_plot(:,j) = vw_2;
    pv3_plot(:,j) = vw_3;
    pv4_plot(:,j) = vw_4;

    j=j+1;
end

for i=1:R
    bot{i}.Set_Velocity(0,0);
end

figure
hold on
plot(p1_plot(1,1),p1_plot(2,1), 'o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerEdgeColor', [0.85 0.33 0.1])
plot(p2_plot(1,1),p2_plot(2,1), 'o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerEdgeColor', [0 0.45 0.74])
plot(p3_plot(1,1),p3_plot(2,1), 'o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerEdgeColor', [0.49 0.18 0.56])
plot(p4_plot(1,1),p4_plot(2,1), 'o', 'LineWidth', 2, 'MarkerSize', 10, 'MarkerEdgeColor', [0.93 0.69 0.13]) 

plot(p1_plot(1,1:t1),p1_plot(2,1:t1),'LineWidth',2,'Color', [0.85 0.33 0.1])
plot(p2_plot(1,1:t1),p2_plot(2,1:t1),'LineWidth',2,'Color', [0 0.45 0.74])
plot(p3_plot(1,1:t1),p3_plot(2,1:t1),'LineWidth',2,'Color', [0.49 0.18 0.56])
plot(p4_plot(1,1:t1),p4_plot(2,1:t1),'LineWidth',2,'Color', [0.93 0.69 0.13])

Dibujar_Movil_1R(p1_plot(:,t1))
Dibujar_Movil_2B(p2_plot(:,t1))
Dibujar_Movil_3M(p3_plot(:,t1))
Dibujar_Movil_4N(p4_plot(:,t1))
title('Formación Diagonal');
legend('Agente 1','Agente 2','Agente 3','Agente 4')
grid on

% Velocidades
figure
subplot(4,1,1);
hold on
grid on
plot(t_plot(1:t1),pv1_plot(:,1:t1),'LineWidth',2)
title('Velocidades de los agentes');
legend('V1','W1')

subplot(4,1,2);
hold on
grid on
plot(t_plot(1:t1),pv2_plot(:,1:t1),'LineWidth',2)
legend('V2','W2')

subplot(4,1,3);
hold on
grid on
plot(t_plot(1:t1),pv3_plot(:,1:t1),'LineWidth',2)
legend('V3','W3')

subplot(4,1,4);
hold on
grid on
plot(t_plot(1:t1),pv4_plot(:,1:t1),'LineWidth',2)
legend('V4','W4')

% U
figure
subplot(4,1,1);
hold on
grid on
plot(t_plot(1:t1),U1_plot(:,1:t1),'LineWidth',2)
title('Acción de control');
legend('U1')

subplot(4,1,2);
hold on
grid on
plot(t_plot(1:t1),U2_plot(:,1:t1),'LineWidth',2)
legend('U2')

subplot(4,1,3);
hold on
grid on
plot(t_plot(1:t1),U3_plot(:,1:t1),'LineWidth',2)
legend('U3')

subplot(4,1,4);
hold on
grid on
plot(t_plot(1:t1),U4_plot(:,1:t1),'LineWidth',2)
legend('U4')


%%
function wx_ij = Wx_ij (Kx,xi,xj,deltax)
    argu=(F(xi-xj)-F(deltax))/Kx;
    wx_ij = (-(1/F(Kx))*csch(argu)^2)+1;
end

function wy_ij = Wy_ij (Ky,yi,yj,deltay)
    argu=(F(yi-yj)-F(deltay))/Ky;
    wy_ij = (-(1/F(Ky))*csch(argu)^2)+1;
end

function f=F(x)
   f=log(cosh(x));
end

function qp = Cinematica_Movil (u,p)
    D = 0.08;

    qp = inv([cos(p(3)) -D*sin(p(3)); sin(p(3)) D*cos(p(3))])*u;
end