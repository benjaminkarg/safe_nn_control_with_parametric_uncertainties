#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import do_mpc
import scipy.io as sio


def template_mpc(model):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)

    data = sio.loadmat('../data/system_and_problem_matrices.mat')
    data_MRCI = sio.loadmat('../data/MRCI.mat')
    invA = np.array(data_MRCI['H_MRCI'], dtype='float')
    invb = np.array(data_MRCI['h_MRCI'], dtype='float')
    setup_mpc = {
        'n_robust': 1,
        'n_horizon': 10,
        't_step': 1.0,
        'store_full_solution':True,
    }

    mpc.set_param(**setup_mpc)

    _x = model.x
    _u = model.u

    Q = np.array(data['Q'],dtype='float')
    R = np.array(data['R'],dtype='float')

    mterm = _x['x'].T @ Q @ _x['x']                                 # terminal cost
    lterm = _x['x'].T @ Q @ _x['x'] +  _u['u'].T @ R @ _u['u']   # stage cost

    mpc.set_objective(mterm=mterm, lterm=lterm)
    mpc.set_rterm(u=0.0)

    max_x = np.array([[5.0], [5.0]])

    mpc.bounds['lower','_x','x'] = -max_x
    mpc.bounds['upper','_x','x'] =  max_x

    mpc.bounds['lower','_u','u'] = -1.0
    mpc.bounds['upper','_u','u'] =  1.0

    mpc.set_nl_cons('X_bar', invA @ _x['x'], ub = invb)

    # Uncertainties
    e_var = np.array([-0.01, 0.01])
    d1_var = np.array([0.9,  1.0])
    d2_var = np.array([0.9,  1.0])
    d3_var = np.array([0.4, 0.6])
    d4_var = np.array([0.9, 1.0])

    mpc.set_uncertainty_values(e = e_var, d1 = d1_var, d2 = d2_var, d3 = d3_var, d4 = d4_var)

    mpc.setup()

    return mpc
