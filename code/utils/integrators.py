

def euler(f_xdot, start_state, control, sim_step=0.01):
  xd = f_xdot(start_state, control)
  return start_state + xd * sim_step

def rk45(f_xdot, start_state, control, sim_step=0.01):
  # _state_space->copy_to(_yn);

  # _h = simulation_step;
  # const double h_d2{ _h / 2.0 };
  # const Eigen::VectorXd hv{ _h * Eigen::VectorXd::Ones(_dim) };
  # const Eigen::VectorXd hvd2{ h_d2 * Eigen::VectorXd::Ones(_dim) };
  # const Eigen::VectorXd yn_hv{ _yn + hv };
  # const Eigen::VectorXd yn_hvd2{ _yn + hvd2 };

  k1 = f_xdot(start_state, control)
  # _compute_derivative();
  # _derivative_space->copy_to(_k1);
  ss_2 = sim_step / 2.0
  # k2 = f_xdot(xn + h/2, yn + (h/2) * k1)

  x1 = start_state + k1 * ss_2
  k2 = f_xdot(x1, control)
  # // ss <- yn + 0.5 * h * k1;
  # _state_space->integrate(yn_hvd2, _derivative_space, h_d2);
  # _compute_derivative();
  # _derivative_space->copy_to(_k2);

  # // Compute k3 = f(xn + h/2, yn + (h/2) * k2)
  x2 = start_state + k2 * ss_2
  k3 = f_xdot(x2, control)

  # // Compute k4 = f(xn + h, yn + h * k3)
  x3= start_state + k3 * sim_step
  k4 = f_xdot(x3, control)

  # // ss <- yn + h * k14
  result = start_state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * sim_step / 6.0;

  return result