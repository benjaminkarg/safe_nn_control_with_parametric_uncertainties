clear all;
close all;
addpath('../../auxiliary_funs/');


%% Params
u_ub =  1.0; % lower bound of control input
u_lb = -1.0; % upper bound of control input
r_max = 10;  % maximum number of iterations


%% Load the neural network
load('./data/nn_controller.mat');
network = make_network_input_admissible(network, u_lb, u_ub);


%% load parametric system descriptions
load('./data/parametric_system_matrices.mat');
n_sys = size(As, 1);


%% Load system
load('./data/system_and_problem_matrices.mat');


%% Load and create sets for verification

% Create candidate set for r-step invariance
% in this case the MRCI via rungger-tabuada
load('./data/MRCI.mat');
X_s = Polyhedron(H_MRCI, h_MRCI);

% hyperplanes to be considered for over-approximation of the one-step
% reachable sets, this case the same hyperplanes as for the MRCI
Hp = H_MRCI;

% admissible state space
X = Polyhedron(H_x, h_x);


%% check if r-step admissible set exists
success_vec = zeros(n_sys, 1);
all_sets = cell(n_sys, 1);
tic;
for i = 1:n_sys
    
    % Get current A
    A = squeeze(As(i, :, :));

    % Compute error set w.r.t. current B
    if length(size(Bs)) == 2
        B = Bs(i, :)';
    else
        B = squeeze(Bs(i, :, :));
    end
    
    % Check r-step invariance
    [r, sets, success] = r_step_invariance(network, Hp, X, X_s, r_max, A, B);
    
    if success
        success_vec(i) = 1;
    else
       disp('Verification was not successful!');
       break;
    end

    %% Plot result
    if success
        figure();
        % Plot admissible state space
        plot(X, 'color', 'red', 'alpha', 0.1);
        hold on;
        % Plot sets of the iteration (starting with the set to be verified)
        for j = 1:length(sets)
            plot(sets(j,1), 'color', 'blue', 'alpha', 0.1);
        end
    end

    all_sets{i} = sets(end);
    
end
comp_time = toc;


%% Save results
% if all(success_vec)
%     H = {};
%     h = {};
%     for i = 1:length(sets)
%         H{i} = sets(i).A;
%         h{i} = sets(i).b;
%     end
%     save('./data/verification_MRCI.mat', 'H', 'h', 'comp_time');
% end
