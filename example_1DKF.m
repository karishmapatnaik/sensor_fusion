clc
clear all
close all

rng(4);

% state equations, discrete system
muQ = 0;
Q = [0.01 0; 0 0.03]; % we need to know this: noise covariance of state 
A = [1.0000 0.0010;
         0    1.0000];
B = 1.0e-03 * [0.0005; 1.0000];
wk = normrnd(muQ,Q);

% output equations
H = [1 0; 0 1];
muR = 0;
R = [0.5 0;0 0.5]; % we need to know this: noise covariance of output
vk = normrnd(muR, R);

% simulation parameters:
t0 = 0;
dt = 0.001;
tend = 10;
x = [0 0]'; % initial state
z = x; % initial measurement;
len = length(t0:dt:tend);
xfs = zeros(len, length(x));
xfs(1,:) = x';
zfs(1,:) = z';
u = 1;

% apriori x
xhat_minus = [0 0]';
xhat = [0 0]';
xhatfs = xfs;
P = [ 0.1 0.2; 0.3 0.1];

% simulation
for i = 1:len
    % actual states are evolving like this
    wk = normrnd(muQ,Q);
    x = A*x + B*u + [wk(1,1); wk(2,2)];
    xfs(i+1,:) = x';
    % but we know only this
    vk = normrnd(muR, R);
    z = H*x + [vk(1,1); vk(2,2)];
    zfs(i+1,:) = z';
    
    %% so kalman filter
    % time update
    xhat_minus = A*xhat_minus + B*u;
    P_minus = A*P*A' + Q;
    % measurement update
    K = P_minus*H'*(H*P_minus*H' + R)^(-1);
    xhat = xhat_minus + K*(z - H*xhat_minus);
    P = (eye(2) - K*H)*P_minus;
    xhatfs(i+1,:) = xhat';   
end

%% plots
tfs = linspace(t0,tend,length(xfs));
figure(1)
hold on
plot(tfs,zfs(:,1),'--g','LineWidth',2)
plot(tfs,zfs(:,2),'--c','LineWidth',2)
plot(tfs,xfs(:,1),'-r','LineWidth',2)
plot(tfs,xfs(:,2),'-b','LineWidth',2)
grid on
legend('position measured','velocity measured','position real','velocity real')

errors = xfs - zfs;
figure(2)
grid on
hold on
plot(tfs,errors(:,1),'Linewidth',2)
plot(tfs,errors(:,2),'Linewidth',2)

figure(3)
hold on
grid on
plot(tfs,xfs(:,1),'r',tfs,xhatfs(:,1),'-.c','LineWidth',2)
plot(tfs,xfs(:,2),'b',tfs,xhatfs(:,2),'-.m','LineWidth',2)
