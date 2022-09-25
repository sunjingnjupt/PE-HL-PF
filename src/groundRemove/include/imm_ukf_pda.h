#ifndef OBJECT_TRACKING_IMM_UKF_JPDAF_H
#define OBJECT_TRACKING_IMM_UKF_JPDAF_H

#include <vector>
#include <chrono>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <array>
#include "param.h"
#include <math.h>
#include "ukf.h"
#include "Eigen/Dense"
#include <stack>
#include "groundRemove/include/box_fitting.h"

using std::stringstream;
using std::array;
using std::vector;
using std::ofstream;
using std::stack;

class ImmUkfPda
{
  // 跟踪对象里面 callBack 一直在跟新输入对象， 历史过程就存储在本 ImmUkfPad 类中
private:
  int target_id_;
  bool init_;
  // 保留上一个时刻的时间戳， 当新的数据输入时， 根据俩帧时间戳相减就可以得到对应的时间消耗
  double timestamp_;

  std::vector<UKF> targets_;  // 保留当前每一个 ukf 对象

  // probabilistic data association params
  double gating_threshold_;
  double gate_probability_;
  double detection_probability_;

  // object association param
  int life_time_threshold_;

  // static classification param
  double static_velocity_threshold_;
  int static_num_history_threshold_;

  // switch sukf and ImmUkfPda
  bool use_sukf_;

  // whether if benchmarking tracking result
  bool is_benchmark_;
  int frame_count_;
  std::string kitti_data_dir_;

  // for benchmark
  std::string result_file_path_;

  // prevent explode param for ukf
  double prevent_explosion_threshold_;

  // for vectormap assisted tarcking
  bool use_vectormap_;
  bool has_subscribed_vectormap_;
  double lane_direction_chi_threshold_;
  double nearest_lane_distance_threshold_;
  std::string vectormap_frame_;
  double merge_distance_threshold_;
  //distance to consider centroids the same
  const double CENTROID_DISTANCE = 0.2;

  std::string input_topic_;
  std::string output_topic_;

  std::string tracking_frame_;

  // add by yyg
  params param;

  // 记录但前处理到那一帧点云对象了
  int currentFrame_ = -1;
  double dt_ = 0.001f;
  int trackId_;
  Eigen::Matrix3f transG2L_, transL2G_;
  vector<float> timestampVec;
	vector<std::array<float, 3>> selfCarPose;
  // id 资源分配问题， 主要逻辑是， 如果当前栈中有值， 那么使用栈顶值提供新生的 ukf
  // 如果当前栈中无可分配ID， 则说明所有ID使用完毕， 总的 ukf 数量加一， 分配 ID 为总的 ukf 数量 加一，
  // 同时当死亡发生时候， 回收 ID 给栈， 同时总的 ukf 数量减少
  stack<int> idStack_;
  int numMaxUKF_ = 0;
  // -------------------------
  // 更新转换矩阵
  void updateTransMatrix(const int & currentFrame);
  void updateTransMatrix(const Eigen::Matrix3f & transL2G);
  point transPointG2L(const point & input);
  point transPointL2G(const point & input);
  void transformPoseToGlobal(const std::vector<BBox>& input,
                             std::vector<BBox>& transformed_input,
                             const size_t & currentFrame);

  // 将结果转换到局部， 并返回给 mian 函数， 方便可视化
  void transformPoseToLocal(std::vector<BBox>& detected_objects_output, 
                            const size_t & currentFrame,
                            vector<Cloud::Ptr> & trackerBBox);

  void measurementValidation(const std::vector<BBox>& input, UKF& target, const bool second_init,
                             const Eigen::VectorXd& max_det_z, const Eigen::MatrixXd& max_det_s,
                             std::vector<BBox>& object_vec, std::vector<bool>& matching_vec);
  BBox getNearestObject(UKF& target,
                                                 const std::vector<BBox>& object_vec);
  void updateBehaviorState(const UKF& target, const bool use_sukf, BBox& object);

  void initTracker(const std::vector<BBox>& input, double timestamp);
  void secondInit(UKF& target, const std::vector<BBox>& object_vec, double dt);

  void updateTrackingNum(const std::vector<BBox>& object_vec, UKF& target);

  bool probabilisticDataAssociation(const std::vector<BBox>& input, const double dt,
                                    std::vector<bool>& matching_vec,
                                    std::vector<BBox>& object_vec, UKF& target);
  void makeNewTargets(const double timestamp, const std::vector<BBox>& input,
                      const std::vector<bool>& matching_vec);

  void staticClassification();

  void makeOutput(const std::vector<BBox>& input,
                  const std::vector<bool>& matching_vec,
                  std::vector<BBox>& detected_objects_output);

  void removeUnnecessaryTarget();

  void dumpResultText(std::vector<BBox>& detected_objects);

  void tracker(const std::vector<BBox>& transformed_input,
               std::vector<BBox>& detected_objects_output,
               const size_t & currentFrame,
               const double & ts,
               Cloud::Ptr & connectPoints,
               std::vector<rotateEllipse> & roateEllipseVec);

  bool updateDirection(const double smallest_nis, const BBox& in_object,
                           BBox& out_object, UKF& target);

//   bool storeObjectWithNearestLaneDirection(const BBox& in_object,
//                                       BBox& out_object);


  std::vector<BBox>
  removeRedundantObjects(const std::vector<BBox>& in_detected_objects,
                         const std::vector<size_t> in_tracker_indices);

  std::vector<BBox>
  forwardNonMatchedObject(const std::vector<BBox>& tmp_objects,
                          const std::vector<BBox>&  input,
                          const std::vector<bool>& matching_vec);

  bool
  arePointsClose(const point& in_point_a,
                 const point& in_point_b,
                 float in_radius);

  bool
  arePointsEqual(const point& in_point_a,
                 const point& in_point_b);

  bool
  isPointInPool(const std::vector<point>& in_pool,
                const point& in_point);

  void updateTargetWithAssociatedObject(const std::vector<BBox>& object_vec,
                                        UKF& target);

  BBox makeNewBBoxSize(const UKF & target,
                       const float & yaw);
  point getNearestPoint(const BBox & box, const double & x, const double & y);

public:
  ImmUkfPda();
  void callback(const std::vector<BBox>& input, 
                const size_t & currentFrame,
                vector<Cloud::Ptr> & trackerBBox,
                const double & ts,
                const int & trackID,
                Cloud::Ptr & connectPoints,
                std::vector<rotateEllipse> & roateEllipseVec,
                const Eigen::Matrix3f & transL2G);
};

#endif /* OBJECT_TRACKING_IMM_UKF_JPDAF_H */
