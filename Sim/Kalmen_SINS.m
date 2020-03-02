clc;
close all;

% ������
T=1;
N=80/T;

X=zeros(4,N);
X(:,1)=[-100,2,200,20];%[x,vx,y,vy]'ϵͳ״̬����
F = [1,T,0,0;0,1,0,0;0,0,1,T;0,0,0,1];%x1=x0+v*T״̬ת�ƾ���

Z=zeros(2,N);
Z(:,1)=[X(1,1),X(3,1)];%[x,y]'�۲����
H=[1,0,0,0;0,0,1,0];%��״̬����ת�Ƶ��۲����������

Q = 1e-2 * diag([.5,1,.5,1]);%��������
R = 100 * eye(2);%�۲�����

for t=2:N
    X(:,t)=F*X(:,t-1) + sqrtm(Q)*randn(4,1);
    Z(:,t)=H*X(:,t) + sqrtm(R)*randn(2,1);
end

figure;
subplot(1,2,1);
plot(X(1,:),X(3,:),'b');
hold on;
plot(Z(1,:),Z(2,:),'r');

%Xkf ��ʼ��
X_kf = zeros(4,N);
X_kf(:,1) = X(:,1);
P0 = eye(4);

%Xkf ��ʼ�˲�
for t=2:N
    Xn = F * X_kf(:,t-1);%����ϵͳ���̵��ƣ�Ԥ�⣩
    P1 = F * P0 * F' + Q;%Ԥ�����Э����
    K = P1 * H' * inv(H*P1*H'+R);%���㿨��������
    X_kf(:,t)=Xn + K*(Z(:,t)-H*Xn);%��ֵ�ӵ���ֵ
    P0 = (eye(4) - K * H) *  P1;%���Э�������
end

subplot(1,2,2);
plot(X(1,:),X(3,:),'b');
hold on;
plot(X_kf(1,:),X_kf(3,:),'r');
