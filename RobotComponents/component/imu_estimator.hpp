#ifndef STEERED_IMU_ESTIMATOR_HPP_
#define STEERED_IMU_ESTIMATOR_HPP_
#include <cmath>
#include <cstdint>
#include <limits>
#include "mahony.hpp"

namespace hello_world::imu {
// 只处理采样、校准和姿态，不包含遥控许可、机器人状态或电机输出。
class ImuEstimator {
 public:
  enum class State { Uninitialized, Calibrating, Aligning, Ready, Fault };
  // 只读诊断快照，单位分别为 m/s² 和 rad/s；不参与校准、就绪或控制许可。
  struct SampleDiagnostics {
    float acc[3],gyro[3],acc_norm,gyro_norm;
  };
  struct Params {
    uint32_t calibration_samples = 2000;
    uint32_t alignment_samples = 500;
    uint32_t period_us = 1000;
    uint32_t stale_us = 5000;
    float gravity = 9.80665f;
    float gravity_tolerance = 0.5f;
    float stationary_gyro = 0.08f;
    float gyro_std_limit = 0.01f;
    float acc_std_limit = 0.10f;
    float alignment_gyro = 0.025f;
    float alignment_gravity_error = 0.03f;
    float mahony_kp = 1.0f;
    float mahony_ki = 0.0f;
  };

  // 构造时不启动校准或采样，初始姿态不可用。
  ImuEstimator() = default;

  // 保存并校验参数；成功后进入静止校准，失败后保持 Fault，返回初始化结果。
  bool begin(const Params &params) {
    // 显式重启时清除旧记录；NaN 模长表示本轮尚未计算出有效模长。
    for(int i=0;i<3;++i)last_acc_[i]=last_gyro_[i]=0;
    last_acc_norm_=last_gyro_norm_=std::numeric_limits<float>::quiet_NaN();
    params_ = params;
    const float positive[] = {params.gravity, params.gravity_tolerance, params.stationary_gyro,
        params.gyro_std_limit, params.acc_std_limit, params.alignment_gyro,
        params.alignment_gravity_error};
    for (float v : positive) if (!std::isfinite(v) || v <= 0) return fault();
    if (params.calibration_samples < 2 || params.calibration_samples > 100000 ||
        params.alignment_samples == 0 || params.alignment_samples > 100000 ||
        params.period_us < 500 || params.period_us > 10000 ||
        params.stale_us <= 2 * params.period_us || params.stale_us > 1000000 ||
        params.gravity_tolerance >= params.gravity ||
        !std::isfinite(params.mahony_kp) || params.mahony_kp < 0 ||
        !std::isfinite(params.mahony_ki) || params.mahony_ki < 0) return fault();
    clearCalibration();
    for (int i=0;i<3;++i) bias_[i]=angles_[i]=0;
    timed_ = false;
    aligned_ = 0;
    state_ = State::Calibrating;
    return true;
  }

  // 显式重启校准；先撤销原姿态许可，禁止继续沿用旧零漂。
  bool recalibrate() { return begin(params_); }

  // 标记传感器、时间或数值故障，清除可用姿态；必须显式重新初始化才能恢复。
  bool fault() {
    state_=State::Fault;
    for (float &v:angles_) v=0;
    return false;
  }

  // 接收已经过硬件检查的新采样；单位为 m/s²、rad/s、微秒。
  // 返回 true 表示本次采样已处理；是否可用于控制还必须查询 ready(now_us)。
  bool pushSample(const float acc[3], const float gyro[3], uint32_t now_us) {
    if (state_==State::Uninitialized || state_==State::Fault) return false;
    // 在原检查前留存输入；故障后的入口直接返回，因此故障样本不会被后续调用覆盖。
    // 空指针或非有限输入仍走原故障路径，诊断记录不修正或替换实际采样。
    const float unavailable=std::numeric_limits<float>::quiet_NaN();
    for(int i=0;i<3;++i) {last_acc_[i]=acc?acc[i]:unavailable;last_gyro_[i]=gyro?gyro[i]:unavailable;}
    last_acc_norm_=last_gyro_norm_=unavailable;
    if (!finiteVector(acc) || !finiteVector(gyro)) return fault();
    const float an=norm(acc), gn=norm(gyro);
    // 复用原计算结果，不增加滤波或改变原有量程、采样时序判断。
    last_acc_norm_=an;last_gyro_norm_=gn;
    if (!std::isfinite(an) || !std::isfinite(gn) || an<0.1f || an>35.0f || gn>17.0f) return fault();
    if (timed_) {
      const uint32_t elapsed=now_us-last_us_;
      // 重复或过早调用不累计样本；过晚调用不能按固定采样频率继续积分。
      if (elapsed<params_.period_us*3/4) return false;
      if (elapsed>params_.period_us*5/4) return fault();
    }
    timed_=true; last_us_=now_us;
    const bool still=std::fabs(an-params_.gravity)<=params_.gravity_tolerance &&
        gn<=params_.stationary_gyro;
    if (state_==State::Calibrating) {
      if (!still) { clearCalibration(); return true; }
      ++count_;
      // Welford 累积均值和方差，既检查陀螺噪声，也检查加速度方向是否稳定。
      for(int i=0;i<3;++i) {
        const float dg=gyro[i]-gyro_mean_[i];
        gyro_mean_[i]+=dg/count_; gyro_m2_[i]+=dg*(gyro[i]-gyro_mean_[i]);
        const float da=acc[i]-acc_mean_[i];
        acc_mean_[i]+=da/count_; acc_m2_[i]+=da*(acc[i]-acc_mean_[i]);
      }
      if(count_>=params_.calibration_samples) finishCalibration();
      return true;
    }
    float corrected[3];
    for(int i=0;i<3;++i) corrected[i]=gyro[i]-bias_[i];
    if(state_==State::Aligning && (!still || norm(corrected)>params_.alignment_gyro)) {
      clearCalibration(); aligned_=0; state_=State::Calibrating;
      return true;
    }
    // 明显线性加速时暂不使用重力校正；仍以有效角速度积分姿态。
    const float zero[3]={0,0,0};
    ahrs_.update(std::fabs(an-params_.gravity)<=params_.gravity_tolerance?acc:zero,corrected);
    float q[4]; ahrs_.getQuat(q);
    if(!normalizeQuat(q)) return fault();
    if(state_==State::Aligning) {
      const float predicted[3]={2*(q[1]*q[3]-q[0]*q[2]),
          2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])};
      float mismatch[3]; for(int i=0;i<3;++i) mismatch[i]=acc[i]/an-predicted[i];
      aligned_=norm(mismatch)<=params_.alignment_gravity_error?aligned_+1:0;
      if(aligned_>=params_.alignment_samples) {
        // 完成静止对齐时记录起始四元数；输出为相对于该起始姿态的旋转。
        for(int i=0;i<4;++i) reference_[i]=q[i];
        state_=State::Ready;
      }
    }
    if(state_==State::Ready) return updateRelativeAngles(q);
    return true;
  }

  // 姿态完成校准/对齐且最后采样尚未过期才返回 true；不代表允许电机动作。
  bool ready(uint32_t now_us) const {
    return state_==State::Ready && timed_ && now_us-last_us_<params_.stale_us;
  }
  // 返回当前阶段，便于上层区分未初始化、校准、对齐、可用和故障。
  State state() const { return state_; }
  // 按值读取最近一次样本诊断，供核验和观察使用；修改返回副本不影响内部记录。
  SampleDiagnostics sampleDiagnostics() const {
    return {{last_acc_[0],last_acc_[1],last_acc_[2]},
            {last_gyro_[0],last_gyro_[1],last_gyro_[2]},last_acc_norm_,last_gyro_norm_};
  }
  // 返回本轮连续静止校准样本数，晃动或方差过大时归零。
  uint32_t calibrationCount() const { return count_; }
  // 返回已完成的姿态对齐样本数，供调试观察收敛进度。
  uint32_t alignmentCount() const { return aligned_; }
  // 获取指定轴零漂，单位 rad/s；非法轴号返回零，不访问数组。
  float bias(int axis) const { return axis>=0 && axis<3?bias_[axis]:0; }
  // 输出相对起始姿态的 roll，未就绪或过期返回零，单位 rad。
  float roll(uint32_t now_us) const { return ready(now_us)?angles_[0]:0; }
  // 输出相对起始姿态的 pitch，未就绪或过期返回零，单位 rad。
  float pitch(uint32_t now_us) const { return ready(now_us)?angles_[1]:0; }
  // 输出相对起始姿态的 yaw，未就绪或过期返回零，范围约为 [-π,π]。
  float yaw(uint32_t now_us) const { return ready(now_us)?angles_[2]:0; }

 private:
  // 检查三轴向量是否非空且各分量均为有限数。
  static bool finiteVector(const float *v) {
    return v && std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
  }
  // 计算三轴向量模长；上层同时检查结果是否溢出。
  static float norm(const float v[3]) { return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
  // 清空本轮统计量；不通过等待或循环阻塞来采集静止数据。
  void clearCalibration() {
    count_=0;
    for(int i=0;i<3;++i) gyro_mean_[i]=gyro_m2_[i]=acc_mean_[i]=acc_m2_[i]=0;
  }
  // 检验静止方差，通过后保存零漂并由重力方向初始化 Mahony，进入对齐阶段。
  void finishCalibration() {
    for(int i=0;i<3;++i) {
      if(gyro_m2_[i]/(count_-1)>params_.gyro_std_limit*params_.gyro_std_limit ||
          acc_m2_[i]/(count_-1)>params_.acc_std_limit*params_.acc_std_limit) {
        clearCalibration(); return;
      }
    }
    for(int i=0;i<3;++i) bias_[i]=gyro_mean_[i];
    const float r=std::atan2(acc_mean_[1],acc_mean_[2]);
    const float p=std::atan2(-acc_mean_[0],std::sqrt(acc_mean_[1]*acc_mean_[1]+acc_mean_[2]*acc_mean_[2]));
    const float cr=std::cos(r/2),sr=std::sin(r/2),cp=std::cos(p/2),sp=std::sin(p/2);
    const float q[4]={cr*cp,sr*cp,cr*sp,-sr*sp};
    ahrs_.init(q,1000000.0f/params_.period_us,params_.mahony_kp,params_.mahony_ki);
    aligned_=0; state_=State::Aligning;
  }
  // 对原有算法输出四元数再精确归一化，避免快速倒平方根误差影响欧拉角定义域。
  static bool normalizeQuat(float q[4]) {
    float n=0;for(int i=0;i<4;++i) n+=q[i]*q[i];
    if(!std::isfinite(n) || n<0.25f || n>4.0f) return false;
    n=std::sqrt(n);for(int i=0;i<4;++i) q[i]/=n;
    return true;
  }
  // 计算 conj(起始姿态)×当前姿态，输出相对欧拉角；非有限结果撤销就绪。
  bool updateRelativeAngles(const float q[4]) {
    const float w=reference_[0],x=-reference_[1],y=-reference_[2],z=-reference_[3];
    const float a=w*q[0]-x*q[1]-y*q[2]-z*q[3];
    const float b=w*q[1]+x*q[0]+y*q[3]-z*q[2];
    const float c=w*q[2]-x*q[3]+y*q[0]+z*q[1];
    const float d=w*q[3]+x*q[2]-y*q[1]+z*q[0];
    float sine=2*(a*c-b*d); if(sine>1)sine=1;if(sine< -1)sine=-1;
    angles_[0]=std::atan2(2*(a*b+c*d),1-2*(b*b+c*c));
    angles_[1]=std::asin(sine);
    angles_[2]=std::atan2(2*(a*d+b*c),1-2*(c*c+d*d));
    return finiteVector(angles_)?true:fault();
  }
  Params params_{};
  ahrs::Mahony ahrs_{};
  State state_=State::Uninitialized;
  uint32_t count_=0,aligned_=0,last_us_=0;
  bool timed_=false;
  // Ozone 可直接展开这些成员；Fault 后保留，只有显式 begin/重校准才重新清除。
  float last_acc_[3]{},last_gyro_[3]{};
  float last_acc_norm_=std::numeric_limits<float>::quiet_NaN();
  float last_gyro_norm_=std::numeric_limits<float>::quiet_NaN();
  float gyro_mean_[3]{},gyro_m2_[3]{},acc_mean_[3]{},acc_m2_[3]{};
  float bias_[3]{},angles_[3]{},reference_[4]{1,0,0,0};
};
}  // namespace hello_world::imu
#endif
