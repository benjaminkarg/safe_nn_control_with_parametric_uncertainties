clear all;
close all;

%% Params
compute_MRCI = true;
saveloc = './data/';
iter_max = 1000;
roh = 0.1;


%% load parametric system descriptions
load('./data/parametric_system_matrices.mat');
n_sys = size(As, 1);


%% Admissible state space
H_x = [eye(2); -eye(2)];
h_x = ones(4,1) * 5;
X = Polyhedron(H_x, h_x);


%% Admissible control input space
H_u = [1; -1];
h_u = [1; 1];
U = Polyhedron(H_u, h_u);


 %% Approximation error set
H_delta = [ 1; -1];
h_delta = [0.01; 0.01];
Delta = Polyhedron(H_delta, h_delta);

    
%% Matrices of quadratic objective function
Q = eye(2);
R = 1;


%% Compute things
MRCI = X.copy();
figure();
% plot(MRCI, 'alpha', 0.05, 'color', 'blue');
hold on;

for i = 1:n_sys
    
    % Get current A
    A = squeeze(As(i, :, :));

    % Compute error set w.r.t. current B
    if length(size(Bs)) == 2
        B = Bs(i, :)';
    else
        B = squeeze(Bs(i, :, :));
    end
    W = affineMap(Delta, B); % transform from input to state space
    H_w = W.A;
    h_w = W.b;
    
    % Compute MRCI
    if compute_MRCI
        addpath('./../../auxiliary_funs/');
        MRCI_i = compute_MRCI_rungger_tabuada(X, W, U, A, B, iter_max, roh);
        plot(MRCI_i, 'alpha', 0.05, 'color', 'blue')
        MRCI = intersect(MRCI, MRCI_i);
    end

    
    
end


%% Save resulting system matrices and disturbance set
if not(isfolder('data'))
    mkdir('data')
end
save('./data/system_and_problem_matrices.mat', 'Q', 'R', 'H_x', 'h_x', 'H_u', 'h_u', 'H_delta', 'h_delta', 'H_w', 'h_w');
H_MRCI = MRCI.A;
h_MRCI = MRCI.b;
V_MRCI = MRCI.V;
save('./data/MRCI.mat', 'H_MRCI', 'h_MRCI', 'V_MRCI');