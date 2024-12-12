#include <iostream>
#include <polynomial/polynomial_traj.h>

namespace fast_planner {
void PolynomialTraj::oneSegmentTraj(const Eigen::Vector3d &start_pos,
                                    const Eigen::Vector3d &end_pos,
                                    const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &end_vel,
                                    const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_acc, const double &time,
                                    PolynomialTraj &poly_traj) {
  Eigen::MatrixXd Ab = Eigen::MatrixXd::Zero(6, 6);
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(6);
  double T = time;
  // clang-format off
  Ab << 1, 0, 0, 0, 0, 0, \
        1, pow(T, 1), pow(T, 2), pow(T, 3), pow(T, 4), pow(T, 5), \
        0, 1, 0, 0, 0, 0, \
        0, 1, 2 * pow(T, 1), 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4), \
        0, 0, 2, 0, 0, 0, \
        0, 0, 2, 6 * pow(T, 1), 12 * pow(T, 2), 20 * pow(T, 3);
  // clang-format on
  Dx << start_pos(0), end_pos(0), start_vel(0), end_vel(0), start_acc(0), end_acc(0);
  Dy << start_pos(1), end_pos(1), start_vel(1), end_vel(1), start_acc(1), end_acc(1);
  Dz << start_pos(2), end_pos(2), start_vel(2), end_vel(2), start_acc(2), end_acc(2);
  
  Eigen::VectorXd Px, Py, Pz;
  Px = Eigen::VectorXd::Zero(6);
  Py = Eigen::VectorXd::Zero(6);
  Pz = Eigen::VectorXd::Zero(6);
  Px = Ab.colPivHouseholderQr().solve(Dx);
  Py = Ab.colPivHouseholderQr().solve(Dy);
  Pz = Ab.colPivHouseholderQr().solve(Dz);
  poly_traj.reset();
  Polynomial poly(Px, Py, Pz, time);
  poly_traj.addSegment(poly);
}

void PolynomialTraj::waypointsTraj(const Eigen::MatrixXd& positions, const Eigen::Vector3d& start_vel,
                                   const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                                   const Eigen::Vector3d& end_acc, const Eigen::VectorXd& times,
                                   PolynomialTraj& poly_traj) {
  const int seg_num = times.size();

  // Helper to construct the mapping matrix
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  // Boundary derivatives of each polynomial segment, the x,y,z axes are independent
  // { {p0, p1, v0, v1, a0, a1}, {p1, p2, v1, v2, a1, a2}... }
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  for (int k = 0; k < seg_num; k++) {
    Dx(k * 6) = positions(k, 0);
    Dx(k * 6 + 1) = positions(k + 1, 0);
    Dy(k * 6) = positions(k, 1);
    Dy(k * 6 + 1) = positions(k + 1, 1);
    Dz(k * 6) = positions(k, 2);
    Dz(k * 6 + 1) = positions(k + 1, 2);

    if (k == 0) {
      Dx(k * 6 + 2) = start_vel(0);
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0);
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    } else if (k == seg_num - 1) {
      Dx(k * 6 + 3) = end_vel(0);
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }

  // Mapping matrix that transform the coefficient into boundary state
  // A*p = d
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
  for (int k = 0; k < seg_num; k++) {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++) {
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(times(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  // Selection Matrix, which map the fixed and free derivatives into original derivative
  // d = Ct * [df, dp]
  Eigen::MatrixXd Ct, C;

  int num_f = 2 * seg_num + 4;  // 2*seg_num for position, 4 for start/end vel/acc
  int num_p = 2 * seg_num - 2;  // (seg_num - 1)
  int num_d = 6 * seg_num;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1;  // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1;  // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1;  // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1;  // Stack the end point

  for (int j = 2; j < seg_num; j++) {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();

  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  // Matrix mapping coefficent to jerk
  //  J = pTQp
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
    for (int i = 3; i < 6; i++)
      for (int j = 3; j < 6; j++) {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(times(k), (i + j - 5));
      }

  // Matrix that maps d'=[df,dp] to jerk
  // J = d'T R d'
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);

  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  // Solve the optimal free derivative in closed form

  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

  // Transform the derivatives back to coefficient
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  poly_traj.reset();
  for (int i = 0; i < seg_num; i++) {
    Polynomial poly(Px.segment<6>(i * 6), Py.segment<6>(i * 6), Pz.segment<6>(i * 6), times[i]);
    poly_traj.addSegment(poly);
    // poly_coeff.block(i, 0, 1, 6) = ;
    // poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    // poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }
}
}