clear
close all
clf
yalmip('clear')

%% create synthetic data for estimating value of x, given y and a
x = [1 2 3 4 5 6]';
t = (0:0.02:2*pi)';
A = [sin(t) sin(2*t) sin(3*t) sin(4*t) sin(5*t) sin(6*t)];
ey = (-4+8*rand(length(t),1));
ey(100:115) = 30;
y = A*x+ey;

%% declare the decision variable
xhat = sdpvar(6,1);

%% linear programming regression
e = y-A*xhat;

% just simply minimizing the L1 norm:
bound = sdpvar(length(e),1);
Constraints = [-bound <= e <= bound];
optimize(Constraints,sum(bound));
x_L1 = value(xhat);

% just minimizing L-infinity norm:
bound = sdpvar(1,1);
Constraints  = [-bound <= e <= bound];
optimize(Constraints,bound);
x_Linf = value(xhat);

%% quadratic programming
% quadratic is when we optimize the L-2 norm
optimize([],e'*e);
x_L2 = value(xhat);

% simply writing as a QP in ters of bound
bound = sdpvar(length(e),1);
Constraints = [-bound <= e <= bound];
optimize(Constraints,e'*e + sum(bound));

%% plotting all estimates
plot(t,[y A*x_L1 A*x_L2 A*x_Linf]);
legend('Measurement','L1','L2','Linf')
