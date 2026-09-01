#include "agt_extrinsic_calibration/optimizer.hpp"
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <cmath>

namespace agt_extrinsic_calibration {
namespace {

struct MotionResidual {
  Eigen::Vector3d wt, lt;
  Eigen::Quaterniond wq, lq;

  template <typename T>
  bool operator()(const T *const t, const T *const q, T *r) const {
    Eigen::Quaternion<T> e_q(q[0], q[1], q[2], q[3]);
    Eigen::Matrix<T,3,1> e_t(t[0], t[1], t[2]);
    Eigen::Quaternion<T> wheel_q(T(wq.w()), T(wq.x()), T(wq.y()), T(wq.z()));
    Eigen::Quaternion<T> lio_q(T(lq.w()), T(lq.x()), T(lq.y()), T(lq.z()));

    auto pred_q = e_q.conjugate() * wheel_q * e_q;
    auto pred_t = e_q.conjugate() *
        (wheel_q * e_t + Eigen::Matrix<T,3,1>(T(wt.x()),T(wt.y()),T(wt.z())) - e_t);

    auto err_q = lio_q.conjugate() * pred_q;
    r[0] = pred_t.x() - T(lt.x());
    r[1] = pred_t.y() - T(lt.y());
    r[2] = pred_t.z() - T(lt.z());
    r[3] = T(2.0) * err_q.x();
    r[4] = T(2.0) * err_q.y();
    r[5] = T(2.0) * err_q.z();
    return true;
  }
};

struct PriorResidual {
  Eigen::Vector3d pt;
  Eigen::Quaterniond pq;
  double ts;
  double rs;

  template <typename T>
  bool operator()(const T *const t, const T *const q, T *r) const {
    r[0] = (t[0]-T(pt.x()))/T(ts);
    r[1] = (t[1]-T(pt.y()))/T(ts);
    r[2] = (t[2]-T(pt.z()))/T(ts);

    Eigen::Quaternion<T> est(q[0],q[1],q[2],q[3]);
    Eigen::Quaternion<T> prior(T(pq.w()),T(pq.x()),T(pq.y()),T(pq.z()));
    Eigen::Quaternion<T> dq = prior.conjugate()*est;
    r[3] = T(2.0)*dq.x()/T(rs);
    r[4] = T(2.0)*dq.y()/T(rs);
    r[5] = T(2.0)*dq.z()/T(rs);
    return true;
  }
};

} // namespace

OptimizationReport optimize_extrinsic(const std::vector<PosePair> &pairs) {
  ExtrinsicPrior prior;
  return optimize_extrinsic(pairs, prior);
}

OptimizationReport optimize_extrinsic(const std::vector<PosePair> &pairs,
                                      const ExtrinsicPrior &prior) {
  OptimizationReport report;
  report.pairs = static_cast<int>(pairs.size());

  if (pairs.empty()) {
    report.message = "No usable motion pairs";
    return report;
  }

  double t[3] = {0,0,0};
  double q[4] = {1,0,0,0};

  if (prior.enabled) {
    t[0]=prior.transform.translation().x();
    t[1]=prior.transform.translation().y();
    t[2]=prior.transform.translation().z();
    Eigen::Quaterniond pq(prior.transform.rotation());
    q[0]=pq.w(); q[1]=pq.x(); q[2]=pq.y(); q[3]=pq.z();
  }

  ceres::Problem problem;

  for (const auto &p: pairs) {
    auto *cost = new ceres::AutoDiffCostFunction<MotionResidual,6,3,4>(
        new MotionResidual{p.wheel_delta.translation(),
                           p.lio_delta.translation(),
                           Eigen::Quaterniond(p.wheel_delta.rotation()),
                           Eigen::Quaterniond(p.lio_delta.rotation())});
    problem.AddResidualBlock(cost,new ceres::HuberLoss(0.1),t,q);
  }

  if(prior.enabled){
    auto *cost = new ceres::AutoDiffCostFunction<PriorResidual,6,3,4>(
        new PriorResidual{prior.transform.translation(),
                          Eigen::Quaterniond(prior.transform.rotation()),
                          prior.translation_sigma,
                          prior.rotation_sigma_rad});
    problem.AddResidualBlock(cost,nullptr,t,q);
  }

  problem.SetParameterization(q,new ceres::QuaternionParameterization());

  ceres::Solver::Options options;
  options.max_num_iterations=100;
  options.linear_solver_type=ceres::DENSE_QR;

  ceres::Solver::Summary summary;
  ceres::Solve(options,&problem,&summary);

  report.success=summary.IsSolutionUsable();
  report.iterations=static_cast<int>(summary.iterations.size());
  report.message=summary.BriefReport();

  report.transform.linear()=
      Eigen::Quaterniond(q[0],q[1],q[2],q[3]).normalized().toRotationMatrix();
  report.transform.translation()=Eigen::Vector3d(t[0],t[1],t[2]);

  return report;
}

void write_result(const OptimizationReport &r,const std::string &yaml,
                  const std::string &md){
  Eigen::Quaterniond q(r.transform.rotation());
  std::ofstream y(yaml);
  y<<std::setprecision(16)
   <<"translation:\n  x: "<<r.transform.translation().x()
   <<"\n  y: "<<r.transform.translation().y()
   <<"\n  z: "<<r.transform.translation().z()
   <<"\nrotation:\n  qx: "<<q.x()
   <<"\n  qy: "<<q.y()
   <<"\n  qz: "<<q.z()
   <<"\n  qw: "<<q.w()<<"\n";

  std::ofstream m(md);
  m<<"# Extrinsic calibration report\n\n"
   <<"success: "<<(r.success?"true":"false")<<"\n"
   <<"pairs: "<<r.pairs<<"\n"
   <<"iterations: "<<r.iterations<<"\n"
   <<"message: "<<r.message<<"\n";
}

} // namespace agt_extrinsic_calibration
