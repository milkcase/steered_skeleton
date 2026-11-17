/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "tick.hpp"
#include "module_state.hpp"
#include "motor_debug_tuning.hpp"
#include "motor_debug_mode.hpp"

/* Exported macro ------------------------------------------------------------*/
namespace robot {
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 底盘↔云台双板 CAN 通讯协议类。底盘板与云台板各实例化一个，用 code_part 区分方向。 */
class GimbalChassisComm {
 public:
  /* 标记本实例运行在哪块板，决定 encode / decode 的方向。 */
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };

  static constexpr uint32_t kChassisTxId=0x210, kGimbalTxId=0x211;
  static constexpr uint32_t kOfflineMs=100, kAssemblyMs=10;
  // 当前调试版本比较模式/安全包络，增益只由底盘使用；速度或限流不同仍不能授权。
  static constexpr uint16_t kPowerProfile=robot_debug::mode::kProfile;
  static constexpr uint8_t kDebugTarget=robot_debug::mode::kTarget;
  enum class RxFailure : uint32_t {None,Crc,Header,SafetyPage,Order,Assembly,Payload};
  // 保存首次撤销完整快照时的页和安全字节；之后的连带缺页不覆盖，直到重新收齐有效快照。
  struct RxEvent {
    RxFailure reason=RxFailure::None;
    uint32_t time_ms=0,page=0,expected_page=0,seq=0,expected_seq=0,state_byte=0,flags_byte=0;
    uint32_t control_signal_valid=0,right=0,neutral=0; // 只记录撤销原因，不提供运行授权。
  };
  struct RxDiagnostics {
    uint32_t completed=0,crc_errors=0,format_errors=0,order_errors=0,assembly_timeouts=0,replays=0;
    RxEvent last_rejection{};
  };
  // 所有布尔量默认 false。ready 是本板自检结果，不依赖对端 Working，避免启动互等。
  struct Flags {
    bool ready=false, imu_ready=false, yaw_feedback_valid=false, pitch_feedback_valid=false;
    bool remote_valid=false, emergency_released=false, command_valid=false, fault=false;
    bool debug_preview=false;  // 仅标记零输出预演；不能作为普通Working控制许可。
    // 动力调试使用bit6诊断格式并另置bit7；两板必须同时匹配，旧预演版不能授权动力。
    bool debug_power=false;
  };
  struct DebugSession {
    uint32_t token=0;
    uint8_t phase=0,target=kDebugTarget;
    uint16_t profile=kPowerProfile;
  };

  /* 底盘→云台（C2G）帧数据：字段由新人按考核要求设计。 */
  struct ChassisData {
    /* TODO【新人任务】:
     * 在此定义 C2G 方向需要传输的语义字段，至少覆盖：
     *   - 底盘是否在线 / 就绪
     *   - 底盘当前工作模式
     *   - 底盘期望的云台 yaw / pitch 角度
     * （共享枚举建议定义在 module_state.hpp 并在本文件 include）
     */
    ModuleState state{};
    Flags flags{};
    GimbalMotion target{};    // 关节目标 rad，pitch 目标是否采用由后续 Robot 决定。
    GimbalMotion feedback{};  // yaw 是本板关节反馈；pitch 若非本板采样，必须明确有效性/来源。
    float chassis_yaw=0.0f;   // 底盘 IMU 相对起始姿态 yaw，不能当共同绝对航向。
    DebugSession session{};
  };
  /* 云台→底盘（G2C）帧数据：字段由新人按考核要求设计。 */
  struct GimbalData {
    /* TODO【新人任务】:
     * 在此定义 G2C 方向需要传输的语义字段，至少覆盖：
     *   - 云台是否在线 / 就绪
     *   - 云台当前工作模式
     *   - 云台实际 yaw / pitch 角度反馈
     */
    ModuleState state{};
    Flags flags{};
    ChassisMotion motion{};   // 云台系 vx/vy(m/s)、w(rad/s)。
    float yaw_target=0.0f;    // 发给底盘执行的 yaw 关节目标 rad。
    GimbalMotion feedback{};  // pitch 为本板反馈；yaw 若为转述，不能当成本板独立测量。
    // 零输出预演使用独立语义；标记置位时线上不再把这些字节解释成正常运动指令。
    struct PreviewRemote {
      float steer=0;
      uint8_t right=2,left=1;
      bool neutral=false,other_neutral=false,valid=false;
      float magnitude=0; // 二维径向幅度，与方向分开传输。
    } preview{};
    DebugSession session{};
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  /* 编码己方帧：把本地 ChassisData / GimbalData 打包进 8 字节。 */
  void encode(uint8_t tx_data[8]);
  // 本地排队失败或旧邮箱被撤销后放弃残余分页，下一次用新序号从第一页发送；不改变接收许可。
  void restartTx() {tx_page_=0;tx_control_active_=false;}
  /* 解码对方帧：把 8 字节还原成对方语义并更新本地缓存。 */
  void decode(const uint8_t rx_data[8]);

  /* 判断对方是否离线（超过阈值时长未收到新帧）。 */
  bool isOffline() const;

  // 返回最近接收数据能否用作控制指令；坏帧/不完整急停页可立即撤销，不能只看原始缓存。
  bool commandsUsable() const;
  // 云台板安全读取完整 C2G 快照；离线、校验失败或调用方向不符时清空输出并返回 false。
  bool readPeer(ChassisData &out) const;
  // 底盘板安全读取完整 G2C 快照；只复制完整新快照，不把碎片当完整指令。
  bool readPeer(GimbalData &out) const;
  // 返回本板发送 ID，供周期发送函数使用；不表示已初始化 CAN。
  uint32_t txId() const { return tx_id_; }
  // 返回本板应接收的对端 ID，供 CAN2 接收路由匹配。
  uint32_t rxId() const { return rx_id_; }
  // 返回最近冻结的发送快照是否通过本地编码检查；失败时线上发送安全故障快照。
  bool lastEncodeValid() const { return tx_valid_; }
  // 返回接收事件计数，只供观察；不以计数代替完整快照或控制许可。
  RxDiagnostics rxDiagnostics() const {return rx_diagnostics_;}
  // 返回最近完整快照年龄；尚未收齐过时用最大无符号数表示未知，不把第一页当续命。
  uint32_t peerAgeMs() const {return has_rx_?hello_world::tick::GetTickMs()-last_rx_tick_:0xFFFFFFFFu;}

  /* 数据访问（供 RobotModules / Task 读写）。 */
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  ///< 上次收到对方帧的 GetTickMs() 时间戳
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
  uint8_t tx_payload_[15]{}, rx_payload_[15]{};
  uint8_t tx_page_=0,tx_seq_=0,rx_seq_=0,rx_next_page_=0,last_seq_=0;
  uint32_t rx_start_ms_=0;
  bool tx_valid_=false,tx_control_active_=false,has_rx_=false,inhibit_=true;
  RxDiagnostics rx_diagnostics_{};
  // 仅在安全状态未丢失的同模式零输出预演拼包期间保留上一份新鲜完整快照。
  // 动力格式仅在同版本下复用原子读取；安全位下降、退出Working或新急停/回中第一页立即撤销。
  bool previewContinuation(const ModuleState &state,const Flags &flags,const uint8_t *page0) const;
  // 检查本地/对端状态与全部比例编码字段，拒绝未知枚举、非有限值和越量程。
  bool packLocal(uint8_t payload[15]) const;
  // 对完整 payload 解码并提交对端语义；不在此函数中更新生命状态或电机输入。
  bool unpackPeer(const uint8_t payload[15]);
  // 判断快照自己的控制许可位是否齐备；仍需结合本板反馈/模式与最终输出门控。
  static bool permitted(const ModuleState &state,const Flags &flags);
  // 立即禁止使用旧指令并放弃当前拼包，不刷新上次完整接收时间。
  void reject(RxFailure reason,const uint8_t *frame);
  // 在清抑制/页状态之前保存首次撤销事件，仅供诊断，不参与安全门控。
  void recordRxRejection(RxFailure reason,const uint8_t *frame);
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */
