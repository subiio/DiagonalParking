yalmip('clear')
clear all

% vehicle parameter
lr = 0.13;
dr = 0.06;
lf = 0.13;
df = 0.07;
w = 0.095;
Lf = lf + df;
Lr = lr + dr;
L_a = lf;
L_b = lr;
length = Lf+Lr;
width = w*2;
dt = 0.2;
% Initial Pose
x0 = [-2; -0.5; 0; 0];

% Target Pose
xT = [4.5; -0.5; 0; 0];

x_ref = xT(1);
y_ref = xT(2);
psi_ref = xT(3);
v_ref = xT(4);

xI = [0.6556; -0.5; 0; 0];
% parking Pose
xI1 = [1.5216; -1.3082; -pi/6; 0];

% back Pose
xB = [0.6556; -0.8082; -pi/6; 0];

% Parking Spot definition
x_lu = 1;
y_lu = -1;
x_ru = 1.34;
y_ru = -1;
x_ld = 1.77;
y_ld = -1.445;
x_rd = 1.85;
y_rd = -1.295;

% constraints

zmax = [10;10;4*pi;10];
zmin = [-10;-10;-4*pi;-10];
umax = [0.6;1.5*dt];
x_min = zmin(1);
y_min = zmin(2);
psi_min = zmin(3);
v_min = zmin(4);
x_max = zmax(1);
y_max = zmax(2);
psi_max = zmax(3);
v_max = zmax(4);

nx = 4;
nu = 2;
M = 1000;

%% First maneuver

% Model Constraints
a_max = 1.5;
a_min = -1.3;
v_max = 2;
v_min = -1*v_max;
d_f_max = pi/6;
d_f_min = -1*d_f_max;

%MPC data
N = 7;
dt = 0.2;
T = 35;

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% d = binvar(repmat(9,1,N),repmat(1,1,N));

objective = 0;
constraints = [];

for k = 1:N
    constraints = [constraints,
                    x{k+1}(1)    == x{k}(1) + dt*(x{k}(4)*cos(x{k}(3) + atan(L_b/(L_a+L_b)*tan(u{k}(1))))),
                    x{k+1}(2)    == x{k}(2) + dt*(x{k}(4)*sin(x{k}(3) + atan(L_b/(L_a+L_b)*tan(u{k}(1))))),
                    x{k+1}(3)    == x{k}(3) + dt*(x{k}(4)*cos(atan(L_b/(L_a+L_b)*tan(u{k}(1))))/(L_a+L_b)*tan(u{k}(1))),
                    x{k+1}(4)    == x{k}(4) + dt*u{k}(2),
                    d_f_min <= u{k}(1) <= d_f_max,
                    a_min <= u{k}(2) <= a_max,
                    zmin <= x{k} <= zmax
                   ];




    constraints = [constraints, implies((1 <= x{k}(1) + Lf*cos(x{k}(3)) - width/2*sin(x{k}(3)) <= 1.8), ((x{k}(2) + Lf*sin(x{k}(3)) + width/2*cos(x{k}(3)))) - (-0.725*(x{k}(1) + Lf*cos(x{k}(3)) - width/2*sin(x{k}(3)) - 0.275))>= 0)];
    constraints = [constraints, implies((1 <= x{k}(1) + Lf*cos(x{k}(3)) + width/2*sin(x{k}(3)) <= 1.8), ((x{k}(2) + Lf*sin(x{k}(3)) - width/2*cos(x{k}(3)))) - (-0.725*(x{k}(1) + Lf*cos(x{k}(3)) + width/2*sin(x{k}(3)) - 0.275))>= 0)];
    constraints = [constraints, implies((1 <= x{k}(1) - Lr*cos(x{k}(3)) + width/2*sin(x{k}(3)) <= 1.8), ((x{k}(2) - Lr*sin(x{k}(3)) - width/2*cos(x{k}(3)))) - (-0.725*(x{k}(1) - Lr*cos(x{k}(3)) + width/2*sin(x{k}(3)) - 0.275))>= 0)];
    constraints = [constraints, implies((1 <= x{k}(1) - Lr*cos(x{k}(3)) - width/2*sin(x{k}(3)) <= 1.8), ((x{k}(2) - Lr*sin(x{k}(3)) + width/2*cos(x{k}(3)))) - (-0.725*(x{k}(1) - Lr*cos(x{k}(3)) - width/2*sin(x{k}(3)) - 0.275))>= 0)];
     
    constraints = [constraints, implies((1.38 <= x{k}(1) + Lf*cos(x{k}(3)) - width/2*sin(x{k}(3)) <= 1.9), ((x{k}(2) + Lf*sin(x{k}(3)) + width/2*cos(x{k}(3)))) - (-0.711*(x{k}(1) + Lf*cos(x{k}(3)) - width/2*sin(x{k}(3)) - 0.01773)) <= 0)];
    constraints = [constraints, implies((1.38 <= x{k}(1) + Lf*cos(x{k}(3)) + width/2*sin(x{k}(3)) <= 1.9), ((x{k}(2) + Lf*sin(x{k}(3)) - width/2*cos(x{k}(3)))) - (-0.711*(x{k}(1) + Lf*cos(x{k}(3)) + width/2*sin(x{k}(3)) - 0.01773)) <= 0)];
    constraints = [constraints, implies((1.38 <= x{k}(1) - Lr*cos(x{k}(3)) + width/2*sin(x{k}(3)) <= 1.9), ((x{k}(2) - Lr*sin(x{k}(3)) - width/2*cos(x{k}(3)))) - (-0.711*(x{k}(1) - Lr*cos(x{k}(3)) + width/2*sin(x{k}(3)) - 0.01773)) <= 0)];
    constraints = [constraints, implies((1.38 <= x{k}(1) - Lr*cos(x{k}(3)) - width/2*sin(x{k}(3)) <= 1.9), ((x{k}(2) - Lr*sin(x{k}(3)) + width/2*cos(x{k}(3)))) - (-0.711*(x{k}(1) - Lr*cos(x{k}(3)) - width/2*sin(x{k}(3)) - 0.01773)) <= 0)];
     
     
    
    constraints = [constraints, y_min <= x{k}(2) + Lf*sin(x{k}(3)) + width/2*cos(x{k}(3)) <= y_max,
        y_min <= x{k}(2) + Lf*sin(x{k}(3)) - width/2*cos(x{k}(3)) <= y_max,
        y_min <= x{k}(2) - Lr*sin(x{k}(3)) - width/2*cos(x{k}(3)) <= y_max,
        y_min <= x{k}(2) - Lr*sin(x{k}(3)) + width/2*cos(x{k}(3)) <= y_max];
    
    
    constraints = [constraints, x_min <= x{k}(1) + Lf*cos(x{k}(3)) - width/2*sin(x{k}(3)) <= x_max,
        x_min <= x{k}(1) + Lf*cos(x{k}(3)) + width/2*sin(x{k}(3)) <= x_max,
        x_min <= x{k}(1) - Lr*cos(x{k}(3)) + width/2*sin(x{k}(3)) <= x_max,
        x_min <= x{k}(1) - Lr*cos(x{k}(3)) - width/2*sin(x{k}(3)) <= x_max];
end

    


objective = objective + (x{N+1}(1) - xI(1))^2 + (x{N+1}(2) - xI(2))^2 + ...
                        (x{N+1}(3) - xI(3))^2 + (x{N+1}(4) - xI(4))^2;


parameters_in = {x{1}};
solutions_out = {[u{:}], [x{:}]};

controller = optimizer(constraints, objective,sdpsettings('solver','IPOPT'),parameters_in,solutions_out);


oldx = x0;


%use these for animation purposes
x_vals = [];
y_vals = [];
psi_vals = [];
v_vals = [];
axis([-5 5 -5 5]);
hold on
for t1 = 1:T
    [solutions,diagnostics] = controller{{oldx}};
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    iteration = t1
    
    X = solutions{2};
    oldx = X(:,2);
    
    x_vals = [x_vals, oldx(1)];
    y_vals = [y_vals, oldx(2)];
    psi_vals = [psi_vals, oldx(3)];
    
    x_vertices = [x_vals(t1) + Lf*cos(psi_vals(t1)) - width/2*sin(psi_vals(t1));
                  x_vals(t1) + Lf*cos(psi_vals(t1)) + width/2*sin(psi_vals(t1));
                  x_vals(t1) - Lr*cos(psi_vals(t1)) + width/2*sin(psi_vals(t1));
                  x_vals(t1) - Lr*cos(psi_vals(t1)) - width/2*sin(psi_vals(t1));
                  x_vals(t1) + Lf*cos(psi_vals(t1)) - width/2*sin(psi_vals(t1));
                  ];
    y_vertices = [y_vals(t1) + Lf*sin(psi_vals(t1)) + width/2*cos(psi_vals(t1));
                  y_vals(t1) + Lf*sin(psi_vals(t1)) - width/2*cos(psi_vals(t1));
                  y_vals(t1) - Lr*sin(psi_vals(t1)) - width/2*cos(psi_vals(t1));
                  y_vals(t1) - Lr*sin(psi_vals(t1)) + width/2*cos(psi_vals(t1));
                  y_vals(t1) + Lf*sin(psi_vals(t1)) + width/2*cos(psi_vals(t1));
                  ];
           
    plot(x_vertices, y_vertices, 'k');
    pause(dt);
    hold on;
    
    if norm(oldx - xI, 2) <= 0.2
        break;
    end
end

%% Second maneuver

% Model Constraints
a_max = 1.5;
a_min = -1.3;
v_max = 2;
v_min = -1*v_max;
d_f_max = pi/6;
d_f_min = -1*d_f_max;

%MPC data
N = 7;
dt = 0.2;
T = 35;

u4 = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x4 = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% d = binvar(repmat(9,1,N),repmat(1,1,N));

objectiveI1 = 0;
constraints4 = [];

for k = 1:N
    constraints4 = [constraints4,
                    x4{k+1}(1)    == x4{k}(1) + dt*(x4{k}(4)*cos(x4{k}(3) + atan(L_b/(L_a+L_b)*tan(u4{k}(1))))),
                    x4{k+1}(2)    == x4{k}(2) + dt*(x4{k}(4)*sin(x4{k}(3) + atan(L_b/(L_a+L_b)*tan(u4{k}(1))))),
                    x4{k+1}(3)    == x4{k}(3) + dt*(x4{k}(4)*cos(atan(L_b/(L_a+L_b)*tan(u4{k}(1))))/(L_a+L_b)*tan(u4{k}(1))),
                    x4{k+1}(4)    == x4{k}(4) + dt*u4{k}(2),
                    d_f_min <= u4{k}(1) <= d_f_max,
                    a_min <= u4{k}(2) <= a_max,
                    zmin <= x4{k} <= zmax
                   ];




    constraints4 = [constraints4, implies((1 <= x4{k}(1) + Lf*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) <= 1.8), ((x4{k}(2) + Lf*sin(x4{k}(3)) + width/2*cos(x4{k}(3)))) - (-0.725*(x4{k}(1) + Lf*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) - 0.275))>= 0)];
    constraints4 = [constraints4, implies((1 <= x4{k}(1) + Lf*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) <= 1.8), ((x4{k}(2) + Lf*sin(x4{k}(3)) - width/2*cos(x4{k}(3)))) - (-0.725*(x4{k}(1) + Lf*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) - 0.275))>= 0)];
    constraints4 = [constraints4, implies((1 <= x4{k}(1) - Lr*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) <= 1.8), ((x4{k}(2) - Lr*sin(x4{k}(3)) - width/2*cos(x4{k}(3)))) - (-0.725*(x4{k}(1) - Lr*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) - 0.275))>= 0)];
    constraints4 = [constraints4, implies((1 <= x4{k}(1) - Lr*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) <= 1.8), ((x4{k}(2) - Lr*sin(x4{k}(3)) + width/2*cos(x4{k}(3)))) - (-0.725*(x4{k}(1) - Lr*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) - 0.275))>= 0)];
     
    constraints4 = [constraints4, implies((1.38 <= x4{k}(1) + Lf*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) <= 1.9), ((x4{k}(2) + Lf*sin(x4{k}(3)) + width/2*cos(x4{k}(3)))) - (-0.711*(x4{k}(1) + Lf*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) - 0.01773)) <= 0)];
    constraints4 = [constraints4, implies((1.38 <= x4{k}(1) + Lf*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) <= 1.9), ((x4{k}(2) + Lf*sin(x4{k}(3)) - width/2*cos(x4{k}(3)))) - (-0.711*(x4{k}(1) + Lf*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) - 0.01773)) <= 0)];
    constraints4 = [constraints4, implies((1.38 <= x4{k}(1) - Lr*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) <= 1.9), ((x4{k}(2) - Lr*sin(x4{k}(3)) - width/2*cos(x4{k}(3)))) - (-0.711*(x4{k}(1) - Lr*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) - 0.01773)) <= 0)];
    constraints4 = [constraints4, implies((1.38 <= x4{k}(1) - Lr*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) <= 1.9), ((x4{k}(2) - Lr*sin(x4{k}(3)) + width/2*cos(x4{k}(3)))) - (-0.711*(x4{k}(1) - Lr*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) - 0.01773)) <= 0)];
     
     
    
    constraints4 = [constraints4, y_min <= x4{k}(2) + Lf*sin(x4{k}(3)) + width/2*cos(x4{k}(3)) <= y_max,
        y_min <= x4{k}(2) + Lf*sin(x4{k}(3)) - width/2*cos(x4{k}(3)) <= y_max,
        y_min <= x4{k}(2) - Lr*sin(x4{k}(3)) - width/2*cos(x4{k}(3)) <= y_max,
        y_min <= x4{k}(2) - Lr*sin(x4{k}(3)) + width/2*cos(x4{k}(3)) <= y_max];
    
    
    constraints4 = [constraints4, x_min <= x4{k}(1) + Lf*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) <= x_max,
        x_min <= x4{k}(1) + Lf*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) <= x_max,
        x_min <= x4{k}(1) - Lr*cos(x4{k}(3)) + width/2*sin(x4{k}(3)) <= x_max,
        x_min <= x4{k}(1) - Lr*cos(x4{k}(3)) - width/2*sin(x4{k}(3)) <= x_max];
end

    


objectiveI1 = objectiveI1 + (x4{N+1}(1) - xI1(1))^2 + (x4{N+1}(2) - xI1(2))^2 + ...
                        (x4{N+1}(3) - xI1(3))^2 + (x4{N+1}(4) - xI1(4))^2;


parameters_in = {x4{1}};
solutions_out = {[u4{:}], [x4{:}]};

controllerI1 = optimizer(constraints4, objectiveI1,sdpsettings('solver','IPOPT'),parameters_in,solutions_out);


oldx = xI;


%use these for animation purposes
for t4 = 1:T
    [solutions,diagnostics] = controllerI1{oldx};
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    iteration = t4
    
    X = solutions{2};
    oldx = X(:,2);
    
    x_vals = [x_vals, oldx(1)];
    y_vals = [y_vals, oldx(2)];
    psi_vals = [psi_vals, oldx(3)];
    v_vals = [v_vals,oldx(4)];
   
    x_vertices = [x_vals(t1+t4) + Lf*cos(psi_vals(t1+t4)) - width/2*sin(psi_vals(t1+t4));
                  x_vals(t1+t4) + Lf*cos(psi_vals(t1+t4)) + width/2*sin(psi_vals(t1+t4));
                  x_vals(t1+t4) - Lr*cos(psi_vals(t1+t4)) + width/2*sin(psi_vals(t1+t4));
                  x_vals(t1+t4) - Lr*cos(psi_vals(t1+t4)) - width/2*sin(psi_vals(t1+t4));
                  x_vals(t1+t4) + Lf*cos(psi_vals(t1+t4)) - width/2*sin(psi_vals(t1+t4));
                  ];
    y_vertices = [y_vals(t1+t4) + Lf*sin(psi_vals(t1+t4)) + width/2*cos(psi_vals(t1+t4));
                  y_vals(t1+t4) + Lf*sin(psi_vals(t1+t4)) - width/2*cos(psi_vals(t1+t4));
                  y_vals(t1+t4) - Lr*sin(psi_vals(t1+t4)) - width/2*cos(psi_vals(t1+t4));
                  y_vals(t1+t4) - Lr*sin(psi_vals(t1+t4)) + width/2*cos(psi_vals(t1+t4));
                  y_vals(t1+t4) + Lf*sin(psi_vals(t1+t4)) + width/2*cos(psi_vals(t1+t4));
                  ];
           
    plot(x_vertices, y_vertices, 'k');
    pause(dt);
    hold on;
    
    if norm(oldx - xI1,2) <= 0.1
        break;
    end
end

%% Second maneuver

% Model Constraints
a_max = 1.5;
a_min = -1.3;
v_max = 2;
v_min = -1*v_max;
d_f_max = pi/6;
d_f_min = -1*d_f_max;

%MPC data
N = 7;
dt = 0.2;
T = 35;

u3 = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x3 = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% d = binvar(repmat(9,1,N),repmat(1,1,N));

objectiveP = 0;
constraints3 = [];

for k = 1:N
    constraints3 = [constraints3,
                    x3{k+1}(1)    == x3{k}(1) + dt*(x3{k}(4)*cos(x3{k}(3) + atan(L_b/(L_a+L_b)*tan(u3{k}(1))))),
                    x3{k+1}(2)    == x3{k}(2) + dt*(x3{k}(4)*sin(x3{k}(3) + atan(L_b/(L_a+L_b)*tan(u3{k}(1))))),
                    x3{k+1}(3)    == x3{k}(3) + dt*(x3{k}(4)*cos(atan(L_b/(L_a+L_b)*tan(u3{k}(1))))/(L_a+L_b)*tan(u3{k}(1))),
                    x3{k+1}(4)    == x3{k}(4) + dt*u3{k}(2),
                    d_f_min <= u3{k}(1) <= d_f_max,
                    a_min <= u3{k}(2) <= a_max,
                    zmin <= x3{k} <= zmax
                   ];




    constraints3 = [constraints3, implies((1 <= x3{k}(1) + Lf*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) <= 1.8), ((x3{k}(2) + Lf*sin(x3{k}(3)) + width/2*cos(x3{k}(3)))) - (-0.725*(x3{k}(1) + Lf*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) - 0.275))>= 0)];
    constraints3 = [constraints3, implies((1 <= x3{k}(1) + Lf*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) <= 1.8), ((x3{k}(2) + Lf*sin(x3{k}(3)) - width/2*cos(x3{k}(3)))) - (-0.725*(x3{k}(1) + Lf*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) - 0.275))>= 0)];
    constraints3 = [constraints3, implies((1 <= x3{k}(1) - Lr*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) <= 1.8), ((x3{k}(2) - Lr*sin(x3{k}(3)) - width/2*cos(x3{k}(3)))) - (-0.725*(x3{k}(1) - Lr*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) - 0.275))>= 0)];
    constraints3 = [constraints3, implies((1 <= x3{k}(1) - Lr*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) <= 1.8), ((x3{k}(2) - Lr*sin(x3{k}(3)) + width/2*cos(x3{k}(3)))) - (-0.725*(x3{k}(1) - Lr*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) - 0.275))>= 0)];
     
    constraints3 = [constraints3, implies((1.38 <= x3{k}(1) + Lf*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) <= 1.9), ((x3{k}(2) + Lf*sin(x3{k}(3)) + width/2*cos(x3{k}(3)))) - (-0.711*(x3{k}(1) + Lf*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) - 0.01773)) <= 0)];
    constraints3 = [constraints3, implies((1.38 <= x3{k}(1) + Lf*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) <= 1.9), ((x3{k}(2) + Lf*sin(x3{k}(3)) - width/2*cos(x3{k}(3)))) - (-0.711*(x3{k}(1) + Lf*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) - 0.01773)) <= 0)];
    constraints3 = [constraints3, implies((1.38 <= x3{k}(1) - Lr*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) <= 1.9), ((x3{k}(2) - Lr*sin(x3{k}(3)) - width/2*cos(x3{k}(3)))) - (-0.711*(x3{k}(1) - Lr*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) - 0.01773)) <= 0)];
    constraints3 = [constraints3, implies((1.38 <= x3{k}(1) - Lr*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) <= 1.9), ((x3{k}(2) - Lr*sin(x3{k}(3)) + width/2*cos(x3{k}(3)))) - (-0.711*(x3{k}(1) - Lr*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) - 0.01773)) <= 0)];
     
     
    
    constraints3 = [constraints3, y_min <= x3{k}(2) + Lf*sin(x3{k}(3)) + width/2*cos(x3{k}(3)) <= y_max,
        y_min <= x3{k}(2) + Lf*sin(x3{k}(3)) - width/2*cos(x3{k}(3)) <= y_max,
        y_min <= x3{k}(2) - Lr*sin(x3{k}(3)) - width/2*cos(x3{k}(3)) <= y_max,
        y_min <= x3{k}(2) - Lr*sin(x3{k}(3)) + width/2*cos(x3{k}(3)) <= y_max];
    
    
    constraints3 = [constraints3, x_min <= x3{k}(1) + Lf*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) <= x_max,
        x_min <= x3{k}(1) + Lf*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) <= x_max,
        x_min <= x3{k}(1) - Lr*cos(x3{k}(3)) + width/2*sin(x3{k}(3)) <= x_max,
        x_min <= x3{k}(1) - Lr*cos(x3{k}(3)) - width/2*sin(x3{k}(3)) <= x_max];
end

    


objectiveP = objectiveP + (x3{N+1}(1) - xB(1))^2 + (x3{N+1}(2) - xB(2))^2 + ...
                        (x3{N+1}(3) - xB(3))^2 + (x3{N+1}(4) - xB(4))^2;


parameters_in = {x3{1}};
solutions_out = {[u3{:}], [x3{:}]};

controllerP = optimizer(constraints3, objectiveP,sdpsettings('solver','IPOPT'),parameters_in,solutions_out);


oldx = xI1;


%use these for animation purposes
for t3 = 1:T
    [solutions,diagnostics] = controllerP{oldx};
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    iteration = t3
    
    X = solutions{2};
    oldx = X(:,2);
    
    x_vals = [x_vals, oldx(1)];
    y_vals = [y_vals, oldx(2)];
    psi_vals = [psi_vals, oldx(3)];
    v_vals = [v_vals,oldx(4)];
    
    x_vertices = [x_vals(t1+t4+t3) + Lf*cos(psi_vals(t1+t4+t3)) - width/2*sin(psi_vals(t1+t4+t3));
                  x_vals(t1+t4+t3) + Lf*cos(psi_vals(t1+t4+t3)) + width/2*sin(psi_vals(t1+t4+t3));
                  x_vals(t1+t4+t3) - Lr*cos(psi_vals(t1+t4+t3)) + width/2*sin(psi_vals(t1+t4+t3));
                  x_vals(t1+t4+t3) - Lr*cos(psi_vals(t1+t4+t3)) - width/2*sin(psi_vals(t1+t4+t3));
                  x_vals(t1+t4+t3) + Lf*cos(psi_vals(t1+t4+t3)) - width/2*sin(psi_vals(t1+t4+t3));
                  ];
    y_vertices = [y_vals(t1+t4+t3) + Lf*sin(psi_vals(t1+t4+t3)) + width/2*cos(psi_vals(t1+t4+t3));
                  y_vals(t1+t4+t3) + Lf*sin(psi_vals(t1+t4+t3)) - width/2*cos(psi_vals(t1+t4+t3));
                  y_vals(t1+t4+t3) - Lr*sin(psi_vals(t1+t4+t3)) - width/2*cos(psi_vals(t1+t4+t3));
                  y_vals(t1+t4+t3) - Lr*sin(psi_vals(t1+t4+t3)) + width/2*cos(psi_vals(t1+t4+t3));
                  y_vals(t1+t4+t3) + Lf*sin(psi_vals(t1+t4+t3)) + width/2*cos(psi_vals(t1+t4+t3));
                  ];
           
    plot(x_vertices, y_vertices, 'k');
    pause(dt);
    hold on;
    
    if norm(oldx - xB,2) <= 0.1
        break;
    end
end



%% third maneuver

N2 = 7;
T2 = 100;


u2 = sdpvar(repmat(nu,1,N2),repmat(1,1,N2));
x2 = sdpvar(repmat(nx,1,N2+1),repmat(1,1,N2+1));
d = binvar(repmat(13,1,N2),repmat(1,1,N2));

constraints2 = [];

for k = 1:N2
    constraints2 = [constraints2,
                    x2{k+1}(1)    == x2{k}(1) + dt*(x2{k}(4)*cos(x2{k}(3) + atan(L_b/(L_a+L_b)*tan(u2{k}(1))))),
                    x2{k+1}(2)    == x2{k}(2) + dt*(x2{k}(4)*sin(x2{k}(3) + atan(L_b/(L_a+L_b)*tan(u2{k}(1))))),
                    x2{k+1}(3)    == x2{k}(3) + dt*(x2{k}(4)*cos(atan(L_b/(L_a+L_b)*tan(u2{k}(1))))/(L_a+L_b)*tan(u2{k}(1))),
                    x2{k+1}(4)    == x2{k}(4) + dt*u2{k}(2),
                    -umax <= u2{k} <= umax,
                    zmin <= x2{k} <= zmax,
                   ];

     
    constraints2 = [constraints2, implies((1 <= x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= 1.8), d{k}(1)*((x2{k}(2) + Lf*sin(x2{k}(3)) + width/2*cos(x2{k}(3)))) - (-0.725*(x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) - 0.275))>= 0), d{k}(1) == 1];
    constraints2 = [constraints2, implies((1 <= x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= 1.8), d{k}(2)*((x2{k}(2) + Lf*sin(x2{k}(3)) - width/2*cos(x2{k}(3)))) - (-0.725*(x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) - 0.275))>= 0), d{k}(2) == 1];
    constraints2 = [constraints2, implies((1 <= x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= 1.8), d{k}(3)*((x2{k}(2) - Lr*sin(x2{k}(3)) - width/2*cos(x2{k}(3)))) - (-0.725*(x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) - 0.275))>= 0), d{k}(3) == 1];
    constraints2 = [constraints2, implies((1 <= x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= 1.8), d{k}(4)*((x2{k}(2) - Lr*sin(x2{k}(3)) + width/2*cos(x2{k}(3)))) - (-0.725*(x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) - 0.275))>= 0), d{k}(4) == 1];
     
     
     
    constraints2 = [constraints2, implies((1.34 <= x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= 1.85), d{k}(5)*((x2{k}(2) + Lf*sin(x2{k}(3)) + width/2*cos(x2{k}(3)))) - (-0.711*(x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) - 0.01773)) <= 0), d{k}(5) == 1];
    constraints2 = [constraints2, implies((1.34 <= x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= 1.85), d{k}(6)*((x2{k}(2) + Lf*sin(x2{k}(3)) - width/2*cos(x2{k}(3)))) - (-0.711*(x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) - 0.01773)) <= 0), d{k}(6) == 1];
    constraints2 = [constraints2, implies((1.34 <= x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= 1.85), d{k}(7)*((x2{k}(2) - Lr*sin(x2{k}(3)) - width/2*cos(x2{k}(3)))) - (-0.711*(x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) - 0.01773)) <= 0), d{k}(7) == 1];
    constraints2 = [constraints2, implies((1.34 <= x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= 1.85), d{k}(8)*((x2{k}(2) - Lr*sin(x2{k}(3)) + width/2*cos(x2{k}(3)))) - (-0.711*(x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) - 0.01773)) <= 0), d{k}(8) == 1];
     
    constraints2= [constraints2, implies((1.77<= x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= 1.85),d{k}(9)*((x2{k}(2) + Lf*sin(x2{k}(3)) + width/2*cos(x2{k}(3)))) - (2.1*(x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) - 5.36)) >=0),  d{k}(9) == 1];
    constraints2= [constraints2, implies((1.77<= x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= 1.85),d{k}(10)*((x2{k}(2) + Lf*sin(x2{k}(3)) - width/2*cos(x2{k}(3)))) - (2.1*(x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) - 5.36)) >=0),  d{k}(10) == 1];
    constraints2= [constraints2, implies((1.77<= x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= 1.85),d{k}(11)*((x2{k}(2) - Lr*sin(x2{k}(3)) - width/2*cos(x2{k}(3)))) - (2.1*(x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) - 5.36)) >=0),  d{k}(11) == 1];
    constraints2= [constraints2, implies((1.77<= x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= 1.85),d{k}(12)*((x2{k}(2) - Lr*sin(x2{k}(3)) + width/2*cos(x2{k}(3)))) - (2.1*(x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) - 5.36)) >=0),  d{k}(12) == 1];
    
    % constraints2 = [constraints2, y_min <= x2{k}(2) + Lf*sin(x2{k}(3)) + width/2*cos(x2{k}(3)) <= y_max,
    %     y_min <= x2{k}(2) + Lf*sin(x2{k}(3)) - width/2*cos(x2{k}(3)) <= y_max,
    %     y_min <= x2{k}(2) - Lr*sin(x2{k}(3)) - width/2*cos(x2{k}(3)) <= y_max,
    %     y_min <= x2{k}(2) - Lr*sin(x2{k}(3)) + width/2*cos(x2{k}(3)) <= y_max];
    % 
    % 
    % constraints2 = [constraints2, x_min <= x2{k}(1) + Lf*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= x_max,
    %     x_min <= x2{k}(1) + Lf*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= x_max,
    %     x_min <= x2{k}(1) - Lr*cos(x2{k}(3)) + width/2*sin(x2{k}(3)) <= x_max,
    %     x_min <= x2{k}(1) - Lr*cos(x2{k}(3)) - width/2*sin(x2{k}(3)) <= x_max];
end
    


objectiveT = 100*((x2{N2+1}(1) - xT(1))^2 + (x2{N2+1}(2) - xT(2))^2 + ...
                        (x2{N2+1}(3) - xT(3))^2) + (x2{N2+1}(4) - xT(4))^2;


parameters_in2 = {x2{1}};
solutions_out2 = {[u2{:}], [x2{:}]};

controllerT = optimizer(constraints2, objectiveT , sdpsettings('solver','ipopt'),parameters_in2,solutions_out2);

oldx = xB;

for t2 = 1:T2
    [solutions,diagnostics] = controllerT{{oldx}};
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    iteration = t2
    
    X = solutions{2};
    oldx = X(:,2);
    
    x_vals = [x_vals, oldx(1)];
    y_vals = [y_vals, oldx(2)];
    psi_vals = [psi_vals, oldx(3)];
    v_vals = [v_vals,oldx(4)];
    
    x_vertices = [x_vals(t1+t3+t4+t2) + Lf*cos(psi_vals(t1+t3+t4+t2)) - width/2*sin(psi_vals(t1+t3+t4+t2));
                  x_vals(t1+t3+t4+t2) + Lf*cos(psi_vals(t1+t3+t4+t2)) + width/2*sin(psi_vals(t1+t3+t4+t2));
                  x_vals(t1+t3+t4+t2) - Lr*cos(psi_vals(t1+t3+t4+t2)) + width/2*sin(psi_vals(t1+t3+t4+t2));
                  x_vals(t1+t3+t4+t2) - Lr*cos(psi_vals(t1+t3+t4+t2)) - width/2*sin(psi_vals(t1+t3+t4+t2));
                  x_vals(t1+t3+t4+t2) + Lf*cos(psi_vals(t1+t3+t4+t2)) - width/2*sin(psi_vals(t1+t3+t4+t2));
                  ];
    y_vertices = [y_vals(t1+t3+t4+t2) + Lf*sin(psi_vals(t1+t3+t4+t2)) + width/2*cos(psi_vals(t1+t3+t4+t2));
                  y_vals(t1+t3+t4+t2) + Lf*sin(psi_vals(t1+t3+t4+t2)) - width/2*cos(psi_vals(t1+t3+t4+t2));
                  y_vals(t1+t3+t4+t2) - Lr*sin(psi_vals(t1+t3+t4+t2)) - width/2*cos(psi_vals(t1+t3+t4+t2));
                  y_vals(t1+t3+t4+t2) - Lr*sin(psi_vals(t1+t3+t4+t2)) + width/2*cos(psi_vals(t1+t3+t4+t2));
                  y_vals(t1+t3+t4+t2) + Lf*sin(psi_vals(t1+t3+t4+t2)) + width/2*cos(psi_vals(t1+t3+t4+t2));
                  ];
           
    plot(x_vertices, y_vertices, 'k');
    pause(dt);
    hold on;
    
    if norm(oldx - xT,2) <= 0.1
        break;
    end
end


%% Animation
% 차량 모델의 참조 및 실제 좌표 계산을 위한 초기 설정
x_vertices_ref = [x_ref + length/2*cos(psi_ref) - width/2*sin(psi_ref);
                  x_ref + length/2*cos(psi_ref) + width/2*sin(psi_ref);
                  x_ref - length/2*cos(psi_ref) + width/2*sin(psi_ref);
                  x_ref - length/2*cos(psi_ref) - width/2*sin(psi_ref);
                  x_ref + length/2*cos(psi_ref) - width/2*sin(psi_ref)];
y_vertices_ref = [y_ref + length/2*sin(psi_ref) + width/2*cos(psi_ref);
                  y_ref + length/2*sin(psi_ref) - width/2*cos(psi_ref);
                  y_ref - length/2*sin(psi_ref) - width/2*cos(psi_ref);
                  y_ref - length/2*sin(psi_ref) + width/2*cos(psi_ref);
                  y_ref + length/2*sin(psi_ref) + width/2*cos(psi_ref)]; 

figure()

axis([-5 5 -5 5]);
hold on;

% 사선 주차장 경계선 그리기
% 주어진 점 정의
x = [0.91, 1.8, 2.05, 1.6];
y = [-1.2, -1.7, -1.4, -1.2];

fill(x, y, 'b'); % 'b'는 파란색을 의미


% x가 1.6 이상일 때의 선 그리기
x1 = [1.6, 1000];
y1 = [-1.2, -1.2];
line(x1, y1, 'Color', 'blue', 'LineWidth', 2);

% x가 1 이하일 때의 선 그리기
x2 = [-1000, 1];
y2 = [-1.2, -1.2];
line(x2, y2, 'Color', 'blue', 'LineWidth', 2);

grid on;

% 라벨 추가
xlabel('X-coordinate');
ylabel('Y-coordinate');
title('Parking Space Boundary');



% 차량 이동 애니메이션
for j = 1:t1+t3+t2+t4
    x_vertices = [x_vals(j) + Lf*cos(psi_vals(j)) - width/2*sin(psi_vals(j));
                  x_vals(j) + Lf*cos(psi_vals(j)) + width/2*sin(psi_vals(j));
                  x_vals(j) - Lr*cos(psi_vals(j)) + width/2*sin(psi_vals(j));
                  x_vals(j) - Lr*cos(psi_vals(j)) - width/2*sin(psi_vals(j));
                  x_vals(j) + Lf*cos(psi_vals(j)) - width/2*sin(psi_vals(j))];
    y_vertices = [y_vals(j) + Lf*sin(psi_vals(j)) + width/2*cos(psi_vals(j));
                  y_vals(j) + Lf*sin(psi_vals(j)) - width/2*cos(psi_vals(j));
                  y_vals(j) - Lr*sin(psi_vals(j)) - width/2*cos(psi_vals(j));
                  y_vals(j) - Lr*sin(psi_vals(j)) + width/2*cos(psi_vals(j));
                  y_vals(j) + Lf*sin(psi_vals(j)) + width/2*cos(psi_vals(j))];

    plot(x_vertices, y_vertices, 'k');
    pause(dt);
end
