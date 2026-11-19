#pragma once
#include "gimbal_chassis_comm.hpp"

namespace robot_diagnostics {
// 整车舵角和速度目标及实测值，active 表示该组控制当前有效。
struct LoopDiagnostics {
  uint32_t active=0,run_count=0;
  float position_target_rad=0,position_feedback_rad=0,position_error_rad=0;
  float speed_target_rad_s=0,speed_feedback_rad_s=0,speed_error_rad_s=0;
  float current_command_a=0;
};
// IMU缓存和前台采样节奏的只读观测，区分采样断档、底层未就绪与对端传播失效。
struct RuntimeHealth {
  uint32_t board=0,poll_count=0,ready_updates=0,imu_cache_ready=0,imu_age_ms=0;
  uint32_t max_poll_gap_ms=0,imu_drops=0,last_imu_drop_age_ms=0,last_imu_drop_cache_ready=0;
  uint32_t control_us=0,max_control_us=0;
  uint32_t imu_state=0,imu_last_interval_us=0,imu_max_interval_us=0,imu_timing_fault=0;
  uint32_t imu_recalibrations=0;
};
}

// 四轮下标依次为左前、左后、右后、右前。仅供读取，不参与控制。
extern "C" {inline volatile robot_diagnostics::LoopDiagnostics robot_four_steer_diagnostics[4]{};}
extern "C" {inline volatile robot_diagnostics::RuntimeHealth robot_runtime_health{};}
// 默认主程序专用只读镜像；不通过修改诊断字段改变许可、目标或PID。
struct RobotMainDiagnostics {
  uint32_t board=0,life=0,control=0,ready=0,imu=0,peer=0,axis_valid=0;
  uint32_t axis_action=0,axis_enabled=0,axis_status=0,prepared=0,prepare_failed=0;
  uint32_t peer_life=0,peer_ready=0,peer_imu=0,peer_axis_valid=0,peer_command_valid=0,peer_fault=0;
  float yaw_target_rad=0,pitch_target_rad=0,axis_angle_rad=0,axis_speed_rad_s=0,axis_speed_target_rad_s=0,torque_nm=0;
  float axis_position_target_rad=0,axis_position_error_rad=0;
  float axis_feedback_torque_nm=0,feedforward_nm=0,pid_torque_nm=0;
  uint32_t axis_feedback_torque_valid=0,feedforward_endpoint=0,torque_limited=0;
  // 两板同名pitch只读实测镜像；底盘来自独立CAN观测帧，valid=0时三项归零。
  float pitch_feedback_angle_rad=0,pitch_feedback_speed_rad_s=0,pitch_feedback_torque_nm=0;
  uint32_t pitch_feedback_valid=0,pitch_feedback_age_ms=0xffffffffu,pitch_feedback_count=0;
  // 与上述pitch反馈来自同一组快照；目标为串级两环真正使用的值，不是未限速的摇杆积分目标。
  float pitch_position_target_rad=0,pitch_speed_target_rad_s=0,pitch_torque_nm=0;
  float pitch_feedforward_nm=0,pitch_pid_torque_nm=0,pitch_position_error_rad=0;
  uint32_t pitch_control=0,pitch_torque_limited=0,pitch_feedforward_endpoint=0;
  // 底盘跟随观测：目标相对航向为零，PID转速经变化率限制后传给四轮逆解。
  uint32_t chassis_mode=0,follow_active=0;
  float follow_error_rad=0,follow_pid_rpm=0,chassis_target_rpm=0,chassis_feedback_rpm=0;
  float yaw_world_feedback_rpm=0,yaw_chassis_compensation_rpm=0;
  // 跟随时同一底盘IMU参考下的对地目标、实测角与最短误差。
  uint32_t yaw_world_target_valid=0;
  float yaw_world_target_rad=0,yaw_world_angle_rad=0,yaw_world_error_rad=0;
  // 分离模式车头独立锁向的有效标记、目标、底盘实测角和最短误差。
  uint32_t chassis_heading_hold=0;
  float chassis_heading_target_rad=0,chassis_heading_actual_rad=0,chassis_heading_error_rad=0;
};
extern "C" {inline volatile RobotMainDiagnostics robot_main_diagnostics{};}
// 一次准备流程的首个停止现场；右下保留，新一轮有效右中准备才允许记录下一次。
struct RobotMainStopDiagnostics {
  uint32_t count=0,time_ms=0,source=0,life=0,control=0,axis_action=0,axis_enabled=0,axis_valid=0,axis_status=0;
  uint32_t peer_valid=0,peer_life=0,peer_ready=0,peer_imu=0,peer_axis_valid=0,peer_command_valid=0,peer_fault=0,commands_usable=0;
  uint32_t imu=0,parameters=0,prepared=0,prepare_failed=0;
  uint32_t remote_valid=0,remote_released=0,remote_arm=0,remote_run=0,remote_neutral=0;
  float dt_s=0;
  uint32_t peer_age_ms=0,rx_completed=0,crc_errors=0,format_errors=0,order_errors=0,assembly_timeouts=0,replays=0;
  robot::GimbalChassisComm::RxEvent rx_event{};
  uint32_t last_peer_life=0,last_peer_ready=0,last_peer_imu=0,last_peer_axis_valid=0,last_peer_command_valid=0,last_peer_fault=0;
  float axis_angle_rad=0,axis_velocity_rad_s=0,last_peer_axis_rad=0;
};
extern "C" {inline volatile RobotMainStopDiagnostics robot_main_stop{};}
extern "C" {inline volatile robot_diagnostics::LoopDiagnostics robot_drive_diagnostics[4]{};}
namespace robot_diagnostics {
// 复制接收撤销事件到只读全局镜像；逐字段写入volatile，事件本身不参与实际许可判断。
inline void publishRxEvent(volatile robot::GimbalChassisComm::RxEvent &out,const robot::GimbalChassisComm::RxEvent &source) {
  out.reason=source.reason;out.time_ms=source.time_ms;out.page=source.page;out.expected_page=source.expected_page;
  out.seq=source.seq;out.expected_seq=source.expected_seq;out.state_byte=source.state_byte;out.flags_byte=source.flags_byte;
}
// 逐字段发布两环目标和真实反馈，调试器写镜像不会反向影响私有PID或动作许可。
inline void publishLoops(volatile LoopDiagnostics &out,const LoopDiagnostics &in) {
  out.active=in.active;out.run_count=in.run_count;
  out.position_target_rad=in.position_target_rad;out.position_feedback_rad=in.position_feedback_rad;out.position_error_rad=in.position_error_rad;
  out.speed_target_rad_s=in.speed_target_rad_s;out.speed_feedback_rad_s=in.speed_feedback_rad_s;out.speed_error_rad_s=in.speed_error_rad_s;
  out.current_command_a=in.current_command_a;
}
}
