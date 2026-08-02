/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 * Modifier: Hyungtae Lim
 * See LICENSE for the license information
 */

#pragma once

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <omp.h>

// TODO(jshi): might be a good idea to template Eigen::Vector3f and Eigen::VectorXf such that later
// on we can decide to use double if we want. Double vs float might give nontrivial differences..

namespace kiss_matcher {

/**
 * Struct to hold solution to a registration problem
 */
struct RegistrationSolution {
  bool valid                  = false;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation    = Eigen::Matrix3d::Identity();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Abstract virtual class for decoupling specific rotation estimation method implementations with
 * interfaces.
 */
class AbstractRotationSolver {
 public:
  virtual ~AbstractRotationSolver() {}

  /**
   * Pure virtual method for solving rotation. Different implementations may have different
   * assumptions about input data.
   * @param src
   * @param dst
   * @return estimated rotation matrix (R)
   */
  virtual void solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                                const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                                Eigen::Matrix3d* rotation,
                                Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) = 0;
};

/**
 * Abstract virtual class for decoupling specific translation estimation method implementations with
 * interfaces.
 */
class AbstractTranslationSolver {
 public:
  virtual ~AbstractTranslationSolver() {}

  /**
   * Pure virtual method for solving translation. Different implementations may have different
   * assumptions about input data.
   * @param src
   * @param dst
   * @return estimated translation vector
   */
  virtual void solveForTranslation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                                   const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                                   Eigen::Vector3d* translation,
                                   Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) = 0;
};

/**
 * Performs scalar truncated least squares estimation
 */
class ScalarTLSEstimator {
 public:
  ScalarTLSEstimator() = default;
  /**
   * Use truncated least squares method to estimate true x given measurements X
   * TODO: call measurements Z or Y to avoid confusion with x
   * TODO: specify which type/size is x and X in the comments
   * @param X Available measurements
   * @param ranges Maximum admissible errors for measurements X
   * @param estimate (output) pointer to a double holding the estimate
   * @param inliers (output) pointer to a Eigen row vector of inliers
   */
  void estimate(const Eigen::RowVectorXd& X,
                const Eigen::RowVectorXd& ranges,
                double* estimate,
                Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers);

  /**
   * A slightly different implementation of TLS estimate. Use loop tiling to achieve potentially
   * faster performance.
   * @param X Available measurements
   * @param ranges Maximum admissible errors for measurements X
   * @param s scale for tiling
   * @param estimate (output) pointer to a double holding the estimate
   * @param inliers (output) pointer to a Eigen row vector of inliers
   */
  void estimate_tiled(const Eigen::RowVectorXd& X,
                      const Eigen::RowVectorXd& ranges,
                      const int& s,
                      double* estimate,
                      Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers);
};

/**
 * Perform translation estimation using truncated least-squares (TLS)
 */
class TLSTranslationSolver : public AbstractTranslationSolver {
 public:
  TLSTranslationSolver() = delete;

  explicit TLSTranslationSolver(double noise_bound, double cbar2)
      : noise_bound_(noise_bound), cbar2_(cbar2) {}

  /**
   * Estimate translation between src and dst points. Assume dst = src + t.
   * @param src
   * @param dst
   * @param translation output parameter for the translation vector
   * @param inliers output parameter for detected outliers
   */
  void solveForTranslation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                           const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                           Eigen::Vector3d* translation,
                           Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) override;

 private:
  double noise_bound_;
  double cbar2_;  // maximal allowed residual^2 to noise bound^2 ratio
  ScalarTLSEstimator tls_estimator_;
};

/**
 * Base class for GNC-based rotation solvers
 */
class GNCRotationSolver : public AbstractRotationSolver {
 public:
  struct Params {
    size_t max_iterations;
    double cost_threshold;
    double gnc_factor;
    double noise_bound;
  };

  explicit GNCRotationSolver(Params params) : params_(params) {}

  Params getParams() { return params_; }

  void setParams(Params params) { params_ = params; }

  /**
   * Return the cost of the GNC solver at termination. Details of the cost function is dependent on
   * the specific solver implementation.
   *
   * @return cost at termination of the GNC solver. Undefined if run before running the solver.
   */
  double getCostAtTermination() { return cost_; }

 protected:
  Params params_;
  double cost_;
};

/**
 * Use GNC-TLS to solve rotation estimation problems.
 *
 * For more information, please refer to:
 * H. Yang, P. Antonante, V. Tzoumas, and L. Carlone, “Graduated Non-Convexity for Robust Spatial
 * Perception: From Non-Minimal Solvers to Global Outlier Rejection,” arXiv:1909.08605 [cs, math],
 * Sep. 2019.
 */
class GNCTLSRotationSolver : public GNCRotationSolver {
 public:
  GNCTLSRotationSolver() = delete;

  /**
   * Parametrized constructor
   * @param params
   */
  explicit GNCTLSRotationSolver(Params params) : GNCRotationSolver(params) {}

  /**
   * Helper function to use SVD to estimate rotation.
   * Method described here: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
   * @param X
   * @param Y
   * @return a rotation matrix R
   */
  inline Eigen::Matrix3d svdRot(const Eigen::Matrix<double, 3, Eigen::Dynamic>& X,
                                const Eigen::Matrix<double, 3, Eigen::Dynamic>& Y,
                                const Eigen::Matrix<double, 1, Eigen::Dynamic>& W) {
    // Assemble the correlation matrix H = X * Y'
    Eigen::Matrix3d H = X * W.asDiagonal() * Y.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
      V.col(2) *= -1;
    }

    return V * U.transpose();
  }

  /**
   * Estimate rotation between src & dst using GNC-TLS method
   * @param src
   * @param dst
   * @param rotation
   * @param inliers
   */
  void solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                        const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                        Eigen::Matrix3d* rotation,
                        Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) override;
};

/**
 * Use Quatro to solve for pairwise registration problems avoiding degeneracy
 *
 * For more information, please see the original paper on Quatro:
 * H. Lim et al., "A Single Correspondence Is Enough: Robust Global Registration
 * to Avoid Degeneracy in Urban Environments," in Robotics - ICRA 2022,
 * Accepted. To appear. arXiv:2203.06612 [cs], Mar. 2022.
 * Quatro and TEASER++ differ in the estimation of rotation. Quatro forgoes roll and pitch
 * estimation, yet it is empirically found that it makes the algorithm more robust against
 * degeneracy.
 */
class QuatroSolver : public GNCRotationSolver {
 public:
  /**
   * Remove default constructor
   */
  QuatroSolver() = delete;

  /**
   * Parametrized constructor
   * @param params
   * @param rotation_only
   */
  explicit QuatroSolver(Params params) : GNCRotationSolver(params) {}

  /**
   * Modified helper function to use svd to estimate SO(2) rotation.
   * Method described here: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
   * @param X
   * @param Y
   * @return a rotation matrix R whose dimension is 2D
   */
  inline Eigen::Matrix2d svdRot2d(const Eigen::Matrix<double, 2, Eigen::Dynamic>& X,
                                  const Eigen::Matrix<double, 2, Eigen::Dynamic>& Y,
                                  const Eigen::Matrix<double, 1, Eigen::Dynamic>& W) {
    // Assemble the correlation matrix H = X * Y'
    Eigen::Matrix2d H = X * W.asDiagonal() * Y.transpose();

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
      V.col(1) *= -1;
    }

    return V * U.transpose();
  }

  /**
   * Solve a pairwise registration problem given two sets of points.
   * Notice that we assume no scale difference between v1 & v2.
   * @param src
   * @param dst
   * @return a RegistrationSolution struct.
   */
  void solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                        const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst,
                        Eigen::Matrix3d* rotation,
                        Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) override;
};

/**
 * Solve registration problems robustly.
 *
 * For more information, please refer to:
 * H. Yang, J. Shi, and L. Carlone, “TEASER: Fast and Certifiable Point Cloud Registration,”
 * arXiv:2001.07715 [cs, math], Jan. 2020.
 */
class RobustRegistrationSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * An enum class representing the available GNC rotation estimation algorithms.
   *
   * GNC_TLS: see H. Yang, P. Antonante, V. Tzoumas, and L. Carlone, “Graduated Non-Convexity for
   * Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection,”
   * arXiv:1909.08605 [cs, math], Sep. 2019.
   *
   * QUATRO: H. Lim et al., "A Single Correspondence Is Enough: Robust Global Registration
   * to Avoid Degeneracy in Urban Environments," in Robotics -
   * ICRA 2022, pp. 8010-8017
   * arXiv:2203.06612 [cs], Mar. 2022.
   */
  enum class ROTATION_ESTIMATION_ALGORITHM {
    GNC_TLS = 0,
    QUATRO  = 1,
  };

  /**
   * A struct representing params for initializing the RobustRegistrationSolver
   *
   * Note: the default values needed to be changed accordingly for best performance.
   */
  struct Params {
    /**
     * A bound on the noise of each provided measurement.
     */
    double noise_bound = 0.01;

    /**
     * Square of the ratio between acceptable noise and noise bound. Usually set to 1.
     */
    double cbar2 = 1;

    /**
     * Which algorithm to use to estimate rotations.
     */
    ROTATION_ESTIMATION_ALGORITHM rotation_estimation_algorithm =
        ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;

    /**
     * Factor to multiple/divide the control parameter in the GNC algorithm.
     *
     * For FGR: the algorithm divides the control parameter by the factor every iteration.
     * For GNC-TLS: the algorithm multiples the control parameter by the factor every iteration.
     */
    double rotation_gnc_factor = 1.4;

    /**
     * Maximum iterations allowed for the GNC rotation estimators.
     */
    size_t rotation_max_iterations = 100;

    /**
     * Cost threshold for the GNC rotation estimators.
     *
     * For FGR / GNC-TLS algorithm, the cost thresholds represent different meanings.
     * For FGR: the cost threshold compares with the computed cost at each iteration
     * For GNC-TLS: the cost threshold compares with the difference between costs of consecutive
     * iterations.
     */
    double rotation_cost_threshold = 1e-6;
  };

  RobustRegistrationSolver() = default;

  /**
   * A constructor that takes in parameters and initialize the estimators accordingly.
   *
   * This is the preferred way of initializing the different estimators, instead of setting
   * each estimator one by one.
   * @param params
   */
  explicit RobustRegistrationSolver(const Params& params);

  /**
   * Given a 3-by-N matrix representing points, return Translation Invariant Measurements (TIMs)
   * @param v a 3-by-N matrix
   * @return a 3-by-(N-1)*N matrix representing TIMs
   */
  Eigen::Matrix<double, 3, Eigen::Dynamic> computeTIMs(
      const Eigen::Matrix<double, 3, Eigen::Dynamic>& v,
      Eigen::Matrix<int, 2, Eigen::Dynamic>* map);

  /**
   * Solve for scale, translation and rotation.
   *
   * @param src_cloud source point cloud (to be transformed)
   * @param dst_cloud target point cloud (after transformation)
   * @param correspondences A vector of tuples representing the correspondences between pairs of
   * points in the two clouds
   */
  // RegistrationSolution solve(const teaser::PointCloud& src_cloud,
  //                            const teaser::PointCloud& dst_cloud,
  //                            const std::vector<std::pair<int, int>> correspondences);

  /**
   * Solve for scale, translation and rotation. Assumes dst is src after transformation.
   * @param src
   * @param dst
   */
  RegistrationSolution solve(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                             const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst);

  /**
   * Solve for translation.
   * @param v1
   * @param v2
   */
  Eigen::Vector3d solveForTranslation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& v1,
                                      const Eigen::Matrix<double, 3, Eigen::Dynamic>& v2);

  /**
   * Solve for rotation. Assume v2 = R * v1, this function estimates find R.
   * @param v1
   * @param v2
   */
  Eigen::Matrix3d solveForRotation(const Eigen::Matrix<double, 3, Eigen::Dynamic>& v1,
                                   const Eigen::Matrix<double, 3, Eigen::Dynamic>& v2);

  /**
   * Return the cost at termination of the GNC rotation solver. Can be used to
   * assess the quality of the solution.
   *
   * @return cost at termination of the GNC solver. Undefined if run before running the solver.
   */
  inline double getGNCRotationCostAtTermination() {
    return rotation_solver_->getCostAtTermination();
  }

  /**
   * Return the solution to the registration problem.
   * @return
   */
  inline RegistrationSolution getSolution() { return solution_; }

  /**
   * Set the rotation estimator used.
   *
   * Note: due to the fact that rotation estimator takes in a noise bound that is dependent on the
   * estimated scale, make sure to set the correct params before calling solve.
   * @param estimator
   */
  inline void setRotationEstimator(std::unique_ptr<GNCRotationSolver> estimator) {
    rotation_solver_ = std::move(estimator);
  }

  /**
   * Set the translation estimator used.
   * @param estimator
   */
  inline void setTranslationEstimator(std::unique_ptr<AbstractTranslationSolver> estimator) {
    translation_solver_ = std::move(estimator);
  }

  /**
   * Return a boolean Eigen row vector indicating whether specific measurements are inliers
   * according to scales.
   *
   * @return a 1-by-(number of TIMs) boolean Eigen matrix
   */
  inline Eigen::Matrix<bool, 1, Eigen::Dynamic> getScaleInliersMask() {
    return scale_inliers_mask_;
  }

  /**
   * Return the index map for scale inliers (equivalent to the index map for TIMs).
   *
   * @return a 2-by-(number of TIMs) Eigen matrix. Entries in one column represent the indices of
   * the two measurements used to calculate the corresponding TIM.
   */
  inline Eigen::Matrix<int, 2, Eigen::Dynamic> getScaleInliersMap() { return src_tims_map_; }

  /**
   * Return inlier TIMs from scale estimation
   *
   * @return a vector of tuples. Entries in each tuple represent the indices of
   * the two measurements used to calculate the corresponding TIM: measurement at indice 0 minus
   * measurement at indice 1.
   */
  inline std::vector<std::tuple<int, int>> getScaleInliers() {
    std::vector<std::tuple<int, int>> result;
    for (size_t i = 0; i < static_cast<size_t>(scale_inliers_mask_.cols()); ++i) {
      if (scale_inliers_mask_(i)) {
        result.emplace_back(src_tims_map_(0, i), src_tims_map_(1, i));
      }
    }
    return result;
  }

  /**
   * Return a boolean Eigen row vector indicating whether specific measurements are inliers
   * according to the rotation solver.
   *
   * @return a 1-by-(size of TIMs used in rotation estimation) boolean Eigen matrix. It is
   * equivalent to a binary mask on the TIMs used in rotation estimation, with true representing
   * that the measurement is an inlier after rotation estimation.
   */
  inline Eigen::Matrix<bool, 1, Eigen::Dynamic> getRotationInliersMask() {
    return rotation_inliers_mask_;
  }

  /**
   * Return the index map for translation inliers (equivalent to max clique).
   * /TODO: This is obsolete now. Remove or update
   *
   * @return a 1-by-(size of max clique) Eigen matrix. Entries represent the indices of the original
   * measurements.
   */
  inline Eigen::Matrix<size_t, 1, Eigen::Dynamic> getRotationInliersMap() {
    Eigen::Matrix<size_t, 1, Eigen::Dynamic> map =
        Eigen::Map<Eigen::Matrix<size_t, 1, Eigen::Dynamic>>(indices_.data(), 1, indices_.size());
    return map;
  }

  /**
   * Return inliers from rotation estimation
   *
   * @return a vector of indices of TIMs passed to rotation estimator deemed as inliers by rotation
   * estimation. Note that depending on the rotation_tim_graph parameter, number of TIMs passed to
   * rotation estimation will be different.
   */
  inline std::vector<size_t> getRotationInliers() { return rotation_inliers_; }

  /**
   * Return a boolean Eigen row vector indicating whether specific measurements are inliers
   * according to translation measurements.
   *
   * @return a 1-by-(size of max clique) boolean Eigen matrix. It is equivalent to a binary mask on
   * the inlier max clique, with true representing that the measurement is an inlier after
   * translation estimation.
   */
  inline Eigen::Matrix<bool, 1, Eigen::Dynamic> getTranslationInliersMask() {
    return translation_inliers_mask_;
  }

  /**
   * Return the index map for translation inliers (equivalent to max clique).
   *
   * @return a 1-by-(size of max clique) Eigen matrix. Entries represent the indices of the original
   * measurements.
   */
  inline Eigen::Matrix<size_t, 1, Eigen::Dynamic> getTranslationInliersMap() {
    Eigen::Matrix<size_t, 1, Eigen::Dynamic> map =
        Eigen::Map<Eigen::Matrix<size_t, 1, Eigen::Dynamic>>(indices_.data(), 1, indices_.size());
    return map;
  }

  /**
   * Return inliers from translation estimation
   *
   * @return a vector of indices of measurements deemed as inliers by translation estimation
   */
  inline std::vector<size_t> getTranslationInliers() { return translation_inliers_; }

  /**
   * Return input-ordered inliers from translation estimation
   *
   * @return a vector of indices of given input correspondences deemed as inliers
   * by translation estimation.
   */
  inline std::vector<size_t> getInputOrderedTranslationInliers() {
    std::vector<size_t> translation_inliers;
    translation_inliers.reserve(translation_inliers_.size());
    for (const auto& i : translation_inliers_) {
      translation_inliers.emplace_back(indices_[i]);
    }
    return translation_inliers;
  }

  /**
   * Get TIMs built from source point cloud.
   * @return
   */
  inline Eigen::Matrix<double, 3, Eigen::Dynamic> getSrcTIMs() { return src_tims_; }

  /**
   * Get TIMs built from target point cloud.
   * @return
   */
  inline Eigen::Matrix<double, 3, Eigen::Dynamic> getDstTIMs() { return dst_tims_; }

  /**
   * Get src TIMs built after max clique pruning.
   * @return
   */
  inline Eigen::Matrix<double, 3, Eigen::Dynamic> getMaxCliqueSrcTIMs() { return pruned_src_tims_; }

  /**
   * Get dst TIMs built after max clique pruning.
   * @return
   */
  inline Eigen::Matrix<double, 3, Eigen::Dynamic> getMaxCliqueDstTIMs() { return pruned_dst_tims_; }

  /**
   * Get the index map of the TIMs built from source point cloud.
   * @return
   */
  inline Eigen::Matrix<int, 2, Eigen::Dynamic> getSrcTIMsMap() { return src_tims_map_; }

  /**
   * Get the index map of the TIMs built from target point cloud.
   * @return
   */
  inline Eigen::Matrix<int, 2, Eigen::Dynamic> getDstTIMsMap() { return dst_tims_map_; }

  /**
   * Reset the solver using the provided params
   * @param params a Params struct
   */
  void reset(const Params& params) {
    params_ = params;

    // Initialize the rotation estimator
    kiss_matcher::GNCRotationSolver::Params rotation_params{params_.rotation_max_iterations,
                                                            params_.rotation_cost_threshold,
                                                            params_.rotation_gnc_factor,
                                                            params_.noise_bound};
    switch (params_.rotation_estimation_algorithm) {
      case ROTATION_ESTIMATION_ALGORITHM::GNC_TLS: {  // GNC-TLS method
        setRotationEstimator(std::make_unique<kiss_matcher::GNCTLSRotationSolver>(rotation_params));
        break;
      }
      case ROTATION_ESTIMATION_ALGORITHM::QUATRO: {  // Quatro method
        setRotationEstimator(std::make_unique<kiss_matcher::QuatroSolver>(rotation_params));
        break;
      }
    }

    // Initialize the translation estimator
    setTranslationEstimator(
        std::make_unique<kiss_matcher::TLSTranslationSolver>(params_.noise_bound, params_.cbar2));

    // Clear member variables
    indices_.clear();
    rotation_inliers_.clear();
    translation_inliers_.clear();
  }

  /**
   * Return the params
   * @return a Params struct
   */
  Params getParams() { return params_; }

 private:
  Params params_;
  RegistrationSolution solution_;

  // Inlier Binary Vectors
  Eigen::Matrix<bool, 1, Eigen::Dynamic> scale_inliers_mask_;
  Eigen::Matrix<bool, 1, Eigen::Dynamic> rotation_inliers_mask_;
  Eigen::Matrix<bool, 1, Eigen::Dynamic> translation_inliers_mask_;

  // TIMs
  // TIMs used for scale estimation/pruning
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_tims_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> dst_tims_;
  // TIMs used for rotation estimation
  Eigen::Matrix<double, 3, Eigen::Dynamic> pruned_src_tims_;
  Eigen::Matrix<double, 3, Eigen::Dynamic> pruned_dst_tims_;

  // TIM maps
  // for scale estimation
  Eigen::Matrix<int, 2, Eigen::Dynamic> src_tims_map_;
  Eigen::Matrix<int, 2, Eigen::Dynamic> dst_tims_map_;

  std::vector<size_t> indices_;

  // Inliers after rotation estimation
  std::vector<size_t> rotation_inliers_;

  // Inliers after translation estimation (final inliers)
  std::vector<size_t> translation_inliers_;

  // Ptrs to Solvers
  std::unique_ptr<GNCRotationSolver> rotation_solver_;
  std::unique_ptr<AbstractTranslationSolver> translation_solver_;
};

}  // namespace kiss_matcher
