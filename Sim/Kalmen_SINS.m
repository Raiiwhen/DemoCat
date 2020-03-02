clc;
close all;

% 变量区
T=1;
N=80/T;

X=zeros(4,N);
X(:,1)=[-100,2,200,20];%[x,vx,y,vy]'系统状态矩阵
F = [1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];%x1=x0+v*T状态转移矩阵

Z=zeros(2,N);
Z(:,1)=[X(1,1),X(3,1)];%[x,y]'观测矩阵
H=[1,0,0,0;0,0,1,0];%将状态矩阵转移到观测矩阵再做和

Q = 1e-2 * diag([.5,1,.5,1]);%过程噪声
R = 100 * eye(2);%观测噪声

for t=2:N
    X(:,t)=F*X(:,t-1) + sqrtm(Q)*randn(4,1);
    Z(:,t)=H*X(:,t) + sqrtm(R)*randn(2,1);
end

figure;
subplot(1,2,1);
plot(X(1,:),X(3,:),'b');
hold on;
plot(Z(1,:),Z(2,:),'r');

%Xkf 初始化
X_kf = zeros(4,N);
X_kf(:,1) = X(:,1);
P0 = eye(4);

%Xkf 开始滤波
for t=2:N
    Xn = F * X_kf(:,t-1);%根据系统方程递推（预测）
    P1 = F * P0 * F' + Q;%预测误差协方差
    K = P1 * H' * inv(H*P1*H'+R);%计算卡尔曼增益
    X_kf(:,t)=Xn + K*(Z(:,t)-H*Xn);%测值加递推值
    P0 = (eye(4) - K * H) *  P1;%误差协方差更新
end

subplot(1,2,2);
plot(X(1,:),X(3,:),'b');
hold on;
plot(X_kf(1,:),X_kf(3,:),'r');
