#include "agt_extrinsic_calibration/optimizer.hpp"
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace agt_extrinsic_calibration {
namespace {
struct Residual {
  Eigen::Vector3d wt, lt;
  Eigen::Quaterniond wq, lq;
  template <typename T> bool operator()(const T *const t, const T *const q, T *r) const {
    Eigen::Quaternion<T> e_q(q[0], q[1], q[2], q[3]);
    Eigen::Matrix<T, 3, 1> e_t(t[0], t[1], t[2]);
    Eigen::Quaternion<T> wheel_q(T(this->wq.w()), T(this->wq.x()), T(this->wq.y()), T(this->wq.z()));
    Eigen::Quaternion<T> lio_q(T(this->lq.w()), T(this->lq.x()), T(this->lq.y()), T(this->lq.z()));
    const auto pred_q = e_q.conjugate() * wheel_q * e_q;
    const auto pred_t = e_q.conjugate() * (wheel_q * e_t + Eigen::Matrix<T,3,1>(T(wt.x()),T(wt.y()),T(wt.z())) - e_t);
    const auto err_q = lio_q.conjugate() * pred_q;
    r[0] = pred_t.x() - T(lt.x()); r[1] = pred_t.y() - T(lt.y()); r[2] = pred_t.z() - T(lt.z());
    r[3] = T(2.0) * err_q.x(); r[4] = T(2.0) * err_q.y(); r[5] = T(2.0) * err_q.z();
    return true;
  }
};

double rmse(const std::vector<PosePair> &pairs, const double *t, const double *qv) {
  double sum = 0.0;
  for (const auto &p : pairs) {
    Eigen::Quaterniond e(qv[0],qv[1],qv[2],qv[3]); Eigen::Vector3d et(t[0],t[1],t[2]);
    Eigen::Quaterniond q = e.conjugate() * Eigen::Quaterniond(p.wheel_delta.rotation()) * e;
    Eigen::Vector3d v = e.conjugate() * (Eigen::Quaterniond(p.wheel_delta.rotation()) * et + p.wheel_delta.translation() - et);
    Eigen::Quaterniond eq = Eigen::Quaterniond(p.lio_delta.rotation()).conjugate() * q;
    Eigen::Matrix<double, 6, 1> r;
    r.head<3>() = v - p.lio_delta.translation();
    r.tail<3>() = 2.0 * eq.vec();
    sum += r.squaredNorm();
  }
  return std::sqrt(sum / std::max<size_t>(1, pairs.size() * 6));
}
}  // namespace

OptimizationReport optimize_extrinsic(const std::vector<PosePair> &pairs) {
  OptimizationReport report; report.pairs = static_cast<int>(pairs.size());
  if (pairs.empty()) { report.message = "No usable motion pairs"; return report; }
  double t[3] = {0, 0, 0}; double q[4] = {1, 0, 0, 0};
  report.initial_rmse = rmse(pairs, t, q);
  ceres::Problem problem;
  for (const auto &p : pairs) {
    auto *cost = new ceres::AutoDiffCostFunction<Residual, 6, 3, 4>(new Residual{
      p.wheel_delta.translation(), p.lio_delta.translation(),
      Eigen::Quaterniond(p.wheel_delta.rotation()), Eigen::Quaterniond(p.lio_delta.rotation())});
    problem.AddResidualBlock(cost, new ceres::HuberLoss(0.1), t, q);
  }
  problem.SetParameterization(q, new ceres::QuaternionParameterization());
  ceres::Solver::Options options; options.max_num_iterations = 100; options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary; ceres::Solve(options, &problem, &summary);
  report.final_rmse = rmse(pairs, t, q); report.iterations = static_cast<int>(summary.iterations.size());
  report.success = summary.IsSolutionUsable(); report.message = summary.BriefReport();
  report.transform.linear() = Eigen::Quaterniond(q[0],q[1],q[2],q[3]).normalized().toRotationMatrix();
  report.transform.translation() = Eigen::Vector3d(t[0],t[1],t[2]);
  return report;
}

void write_result(const OptimizationReport &r, const std::string &yaml, const std::string &md) {
  const auto q = Eigen::Quaterniond(r.transform.rotation());
  std::ofstream y(yaml); y << std::setprecision(16) << "translation:\n  x: " << r.transform.translation().x()
    << "\n  y: " << r.transform.translation().y() << "\n  z: " << r.transform.translation().z()
    << "\nrotation:\n  qx: " << q.x() << "\n  qy: " << q.y() << "\n  qz: " << q.z() << "\n  qw: " << q.w() << "\n";
  std::ofstream m(md); m << "# Extrinsic calibration report\n\n"
    << "- Status: " << (r.success ? "success" : "failed") << "\n"
    << "- Motion pairs: " << r.pairs << "\n"
    << "- Initial RMSE: " << r.initial_rmse << "\n"
    << "- Final RMSE: " << r.final_rmse << "\n"
    << "- Iterations: " << r.iterations << "\n\n"
    << "Solver: " << r.message << "\n";
}
}  // namespace agt_extrinsic_calibration
