%% Algorithmic differentiation
import casadi.*

% y = norm2(x)

% create scalar or matrix symbols
x = MX.sym('x',5);

% compose into expressions 
y = norm(x,2);

% sensitivity of expression -> new expression
y_grad = gradient(y,x);

% create a Function (expression graph) to evaluate expression
f = Function('f', {x}, {y_grad});

% evaluate function numerically
y_grad_numerical = f([1;2;3;4;5])

%% Dynamic systems
import casadi.*

% initial value problems
x = MX.sym('x',2); 

% ODE rhs
z = 1-x(2)^2; % hvorfor z?
rhs = [z*x(1) - x(2);
       x(1)];

ode = struct; % declaration
ode.x = x;
ode.ode = rhs;

% create solver instance Function that integrates over 4s
S = integrator('f', 'cvodes', ode, 0,4);

% start from x0 = 0,1
x0 = [0;
      1];
res = S('x0', x0);

disp(res.xf);

% sensitivity wrt initial state
res = S('x0', x);
y = jacobian(res.xf,x);
S = Function('S', {x}, {y});

disp(S(x0));

%% Nonlinear programs (NLPs)
% min f(x,p) s.t. <x< , <g(x,p)<
% ex: min x^2 + 100*z^2 st. z + (1-x)^2 - y = 0

import casadi.*

% symbols + expressions
x = MX.sym('x');
y = MX.sym('y');
z = MX.sym('z');

f = x^2 + 100*z^2 + 2;
g = z + (1-x)^2 - y;

nlp = struct;            % NLP declaration
nlp.x = [x;y;z];         % decision vars
nlp.f = f;               % objective
nlp.g = g;               % constraints

% create solver instance
S = nlpsol('F', 'ipopt', nlp);

% solve progrem using a guess for x0 + lower and upper bounds on g
S('x0', [2.5, 3.0, 0.75], 'ubg', 0, 'lbg', 0)

%% QP high-level, just replace nlpsol with qpsol and solver plugin to qpOASES
import casadi.*

x = MX.sym('x');
y = MX.sym('y');

f = x^2 + y^2;
g = x + y - 10;

qp = struct('x', [x;y], ...
            'f', f, ...
            'g', g);

S_qp = qpsol('S_qp', 'qpoases', qp)
%S_qp('x0', [2.5, 3.0, 0.75], 'ubg', 0, 'lbg', 0)
r = S_qp('lbg', 0)
x_opt = r.x
%% Quadratic programming (QPs) low level
import casadi.*

H = 2*DM.eye(2);
A = DM.ones(1,2);
g = DM.zeros(2);
lba = 10;

% creating solver instance for QP by passing sparsity patterns of H and A,
% instead of symbolic MX.sym() expresssions for the QP.
% CM type : can query the sparsity patterns

qp = struct;
qp.h = H.sparsity();
qp.a = A.sparsity();
S = conic('S', 'qpoases', qp);
disp(S)

r = S('h', H, 'g', g, ...
      'a', A, 'lba', lba);
x_opt = r.x;
disp(x_opt)

%% For loop
import casadi.*

N = 4;
X = MX.sym('X', 1,N); % 1x4 matrix

disp(f)

ys = {};
% compute f on all columns of matrix X, and putting the results in Y
for i = 1:N
    ys{end+1} = f(X(:,i));
end

Y = [ys{:}];
F = Function('F', {X}, {Y});
disp(F)

%% Basic tests
x = casadi.SX.sym('x');
y = casadi.SX.sym('y',2,2);

disp(sin(y) - x)    %element-wise

disp(y.*y)          % element-wise
disp(y*y)           % matrix multiplication

%% MPC single multiple shooting
% Implement tasks that need to be performed only once, 
% such as pre-computed constants.

T = 10; % Time horizon for the optimization
N = 20; % number of control intervals

% Symbolic declaration of model parameters x and u
x = casadi.SX.sym('x', 13); % p, v, q, w
u = casadi.SX.sym('u', 4); % pitch, yaw, speed_avg, speed_diff

% Objective function
L = norm(x,2) + norm(u,2);

% Model equations
x_dot = state_dynamics_f(x,u, zeros(3,1), zeros(3,1));



% Formulate discrete time dynamics
% Fixed step Runge-Kutta 4 integrator

% CasADi Funtion for continuous time dynamics and objective
f = casadi.Function('f', {x,u}, {x_dot, L}, {'x', 'u'}, {'x_dot', 'L'});

M = 4; % RK4 steps per interval
DT = T/N/M;
X0 = casadi.MX.sym('X0', 13);
U = casadi.MX.sym('U', 4);
X = X0;
Q = 0; % Objective function accumulator
for j=1:M
      [k1, k1_q] = f(X, U);
      [k2, k2_q] = f(X + DT/2 * k1, U);
      [k3, k3_q] = f(X + DT/2 * k2, U);
      [k4, k4_q] = f(X + DT * k3, U);
      X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
      Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = casadi.Function('F', {X0, U}, {X, Q}, {'x0','u'}, {'xf', 'qf'});


% Evaluate at a test point
Fk = F('x0', [0,0,4,0,0,0,1,0,0,0,0,0,0], 'u', [0,0,0.1,0])
disp(Fk.xf)
disp(Fk.qf)
%%
% Start with an empty NLP
% Decision variables
w={};
w0 = [];
lbw = [];
ubw = [];

%objective
J = 0;

% constraints
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions
X0 = casadi.MX.sym('X0', 13);
w = {w{:}, X0};
% lbw = [lbw; 0; 1];
% ubw = [ubw; 0; 1];
% w0 = [w0; 0; 1];
w0 = zeros(13, 1);  % Initial guess for the state
lbw = -inf(13, 1);  % Lower bounds for the state
ubw = inf(13, 1);   % Upper bounds for the state

% Formulate the NLP
Xk = X0;
for k=0:N-1
      % New NLP control variables for each interval
      Uk = casadi.MX.sym(['U_' num2str(k)], 4);
      w = {w{:}, Uk};
      % lbw = [lbw; -1];
      % ubw = [ubw;  1];
      % w0 = [w0;  0];
      % Settinhg bounds and initial guesses for control and state variables
      lbw = [lbw; -ones(4, 1)];  % Append bounds for U and next X
      ubw = [ubw; ones(4, 1)];    % Append bounds for U and next X
      w0 = [w0; zeros(4, 1)];   % Append initial guesses for U and next X

      % Integrate till the end of the interval
      Fk = F('x0', Xk, 'p', Uk);
      Xk_end = Fk.xf;
      J=J+Fk.qf;

      % New NLP state variable at end of interval/for next interval
      Xk = casadi.MX.sym(['X_' num2str(k+1)], 13);
      w = {w{:}, Xk};
      % lbw = [lbw; -0.25; -inf];
      % ubw = [ubw;  inf;  inf];
      % w0 = [w0; 0; 0];
      lbw = [lbw; -inf(13, 1)];
      ubw = [ubw; inf(13, 1)];
      w0 = [w0; zeros(13, 1)];

      % Add equality constraints, so bounds are zero
      g = {g{:}, Xk_end-Xk};
      lbg = [lbg; zeros(13,1)];
      ubg = [ubg; zeros(13,1)];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
solver = casadi.nlpsol('solver', 'ipopt', prob, options);

obj.casadi_solver = solver;
obj.x0 = w0;
obj.lbx = lbw;
obj.ubx = ubw;
obj.lbg = lbg;
obj.ubg = ubg;