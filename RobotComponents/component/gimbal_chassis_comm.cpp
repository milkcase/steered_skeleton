/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "gimbal_chassis_comm.hpp"
#include "crc8.hpp"
#include <cmath>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace {
using Comm=robot::GimbalChassisComm;
// 复用提供的 CRC-8，初值 0，多项式 0x07，覆盖前 7 字节，不包含 CRC 字节。
uint8_t frameCrc(const uint8_t *data) {
  hello_world::tools::crc8_check crc;
  return crc.getCrc8CheckNum(data,7,0);
}
// 将有限浮点数按比例四舍五入为大端 int16；保留 -32768 为非法编码。
bool writeNumber(uint8_t *out,float value,float scale) {
  if(!std::isfinite(value) || std::fabs(value)>32767.0f/scale)return false;
  const int number=static_cast<int>(std::round(value*scale));
  const uint16_t raw=static_cast<uint16_t>(number);
  out[0]=static_cast<uint8_t>(raw>>8);out[1]=static_cast<uint8_t>(raw);
  return true;
}
// 从大端有符号整数还原浮点数；不依赖超范围的有符号强制转换。
bool readNumber(const uint8_t *in,float scale,float &value) {
  const int raw=(in[0]<<8)|in[1];
  if(raw==0x8000)return false;
  value=(raw>=32768?raw-65536:raw)/scale;
  return true;
}
// 打包共享枚举和安全位；这里只说明本板状态，不生成任何动作许可。
bool writeCommon(uint8_t *out,const robot::ModuleState &s,const Comm::Flags &f) {
  if(!robot::isValid(s))return false;
  out[0]=static_cast<uint8_t>(s.life)|(static_cast<uint8_t>(s.chassis_mode)<<2)|
      (static_cast<uint8_t>(s.gimbal_mode)<<4)|(static_cast<uint8_t>(s.source)<<5);
  out[1]=(f.ready?1:0)|(f.imu_ready?2:0)|(f.yaw_feedback_valid?4:0)|(f.pitch_feedback_valid?8:0)|
      (f.remote_valid?16:0)|(f.emergency_released?32:0)|(f.command_valid?64:0)|(f.fault?128:0);
  return true;
}
// 检查保留位和生命状态后解码共享枚举及安全位，未知状态不进入缓存。
bool readCommon(const uint8_t *in,robot::ModuleState &s,Comm::Flags &f) {
  if(in[0]&0xC0)return false;
  s.life=static_cast<robot::LifeState>(in[0]&3);
  s.chassis_mode=static_cast<robot::ChassisMode>((in[0]>>2)&3);
  s.gimbal_mode=static_cast<robot::GimbalMode>((in[0]>>4)&1);
  s.source=static_cast<robot::ControlSource>((in[0]>>5)&1);
  f.ready=in[1]&1;f.imu_ready=in[1]&2;f.yaw_feedback_valid=in[1]&4;f.pitch_feedback_valid=in[1]&8;
  f.remote_valid=in[1]&16;f.emergency_released=in[1]&32;f.command_valid=in[1]&64;f.fault=in[1]&128;
  return robot::isValid(s);
}
}  // namespace

/* Exported function definitions ---------------------------------------------*/
// 保存方向、CAN ID 和初始时间；has_rx 初始为 false，因此构造后仍离线。
robot::GimbalChassisComm::GimbalChassisComm(robot::GimbalChassisComm::CodePart code_part,
                                            uint32_t tx_id, uint32_t rx_id) {
  /* TODO【新人任务】:
   * 保存 code_part / tx_id / rx_id，并把 last_rx_tick_ 初始化为当前 GetTickMs()。
   */
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs();
}

// 每次输出一页；三次调用组成同序快照，安全条件丢失时优先重发安全第一页。
void robot::GimbalChassisComm::encode(uint8_t tx_data[8]) {
  /* TODO【新人任务】:
   * 编码己方帧：把本地 ChassisData / GimbalData 按 hpp 里定的字段布局打包进 8 字节。
   *   - 由 code_part_ 决定打包哪个方向（Chassis → ChassisData，Gimbal → GimbalData）。
   *   - 浮点量按约定比例缩放成整数，避免丢位；收发两端比例必须一致。
   *   - 打包完成后由 Task/component/comm_task.cpp 用 tx_id_ 发送。
   */
  if(tx_data==nullptr) {tx_valid_=false;return;}
  uint8_t live[15]{};
  const bool valid=packLocal(live);
  if(!valid) {for(uint8_t &byte:live)byte=0;live[1]=128;}
  ModuleState state;Flags flags;
  const bool active=readCommon(live,state,flags) && permitted(state,flags);
  if(tx_page_!=0 && tx_control_active_ && !active)tx_page_=0;
  if(tx_page_==0) {
    for(int i=0;i<15;++i)tx_payload_[i]=live[i];
    ++tx_seq_;tx_valid_=valid;tx_control_active_=active;
  }
  tx_data[0]=(code_part_==CodePart::Chassis?0xA0:0xB0)|tx_page_;
  tx_data[1]=tx_seq_;
  for(int i=0;i<5;++i)tx_data[2+i]=tx_payload_[5*tx_page_+i];
  tx_data[7]=frameCrc(tx_data);
  tx_page_=static_cast<uint8_t>((tx_page_+1)%3);
}

// 验证方向、CRC、顺序和拼包时长；只提交完整快照，坏帧不刷新在线时间。
void robot::GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  /* TODO【新人任务】:
   * 解码对方帧：把 8 字节按同样布局还原成对方语义，更新本地缓存，并刷新 last_rx_tick_。
   *   - 严格与 encode() 对称；任何一端偏移写错都会让对面读到错乱数据。
   *   - 建议先写自测：同一结构 encode 后再 decode 应能还原原值。
   */
  if(rx_data==nullptr || frameCrc(rx_data)!=rx_data[7]) {reject();return;}
  if(code_part_!=CodePart::Chassis && code_part_!=CodePart::Gimbal) {reject();return;}
  const uint8_t prefix=code_part_==CodePart::Chassis?0xB0:0xA0;
  const uint8_t page=rx_data[0]&0x0F,seq=rx_data[1];
  if((rx_data[0]&0xF0)!=prefix || page>=3) {reject();return;}
  const uint32_t now=hello_world::tick::GetTickMs();
  if(page==0) {
    ModuleState state;Flags flags;
    if(!readCommon(rx_data+2,state,flags)) {reject();return;}
    if(!permitted(state,flags))inhibit_=true;
    const uint8_t advance=static_cast<uint8_t>(seq-last_seq_);
    if(has_rx_ && (advance==0 || (advance>=128 && (!isOffline() || permitted(state,flags))))) {
      rx_next_page_=0;return;
    }
    rx_seq_=seq;rx_start_ms_=now;rx_next_page_=1;
    for(int i=0;i<5;++i)rx_payload_[i]=rx_data[2+i];
    return;
  }
  if(rx_next_page_!=page || seq!=rx_seq_ || now-rx_start_ms_>=kAssemblyMs) {reject();return;}
  for(int i=0;i<5;++i)rx_payload_[page*5+i]=rx_data[2+i];
  ++rx_next_page_;
  if(page==2) {
    rx_next_page_=0;
    if(!unpackPeer(rx_payload_)) {reject();return;}
    last_seq_=seq;last_rx_tick_=now;has_rx_=true;inhibit_=false;
  }
}

// 上电未收到完整快照即离线；100 ms 没有新完整快照则过期，重复/坏帧不能续命。
bool robot::GimbalChassisComm::isOffline() const {
  /* TODO【新人任务】:
   * 用 GetTickMs() 与 last_rx_tick_ 的差值判断对方是否离线（超过阈值即离线）。
   * 阈值可自定义（如 100ms），供状态机 / 看门狗判断对方是否还活着。
   */
  return !has_rx_ || hello_world::tick::GetTickMs()-last_rx_tick_>=kOfflineMs;
}

/* Private function definitions ----------------------------------------------*/

// 将本板语义冻结为 15 字节 payload，不使用结构体内存布局或裸浮点 memcpy。
bool robot::GimbalChassisComm::packLocal(uint8_t out[15]) const {
  for(int i=0;i<15;++i)out[i]=0;
  if(code_part_==CodePart::Chassis) {
    if(tx_id_!=kChassisTxId || rx_id_!=kGimbalTxId)return false;
    const auto &d=chassis_data_;
    return writeCommon(out,d.state,d.flags) && writeNumber(out+2,d.target.yaw,10000) &&
        writeNumber(out+4,d.target.pitch,10000) && writeNumber(out+6,d.feedback.yaw,10000) &&
        writeNumber(out+8,d.feedback.pitch,10000) && writeNumber(out+10,d.chassis_yaw,10000);
  }
  if(code_part_==CodePart::Gimbal) {
    if(tx_id_!=kGimbalTxId || rx_id_!=kChassisTxId)return false;
    const auto &d=gimbal_data_;
    return writeCommon(out,d.state,d.flags) && writeNumber(out+2,d.motion.vx,1000) &&
        writeNumber(out+4,d.motion.vy,1000) && writeNumber(out+6,d.motion.w,1000) &&
        writeNumber(out+8,d.yaw_target,10000) && writeNumber(out+10,d.feedback.yaw,10000) &&
        writeNumber(out+12,d.feedback.pitch,10000);
  }
  return false;
}
// 以临时对象验证完整 payload，全部字段成功后才一次性替换对端缓存。
bool robot::GimbalChassisComm::unpackPeer(const uint8_t in[15]) {
  if(code_part_==CodePart::Chassis) {
    GimbalData d;
    if(in[14]!=0 || !readCommon(in,d.state,d.flags) ||
        !readNumber(in+2,1000,d.motion.vx) || !readNumber(in+4,1000,d.motion.vy) ||
        !readNumber(in+6,1000,d.motion.w) || !readNumber(in+8,10000,d.yaw_target) ||
        !readNumber(in+10,10000,d.feedback.yaw) || !readNumber(in+12,10000,d.feedback.pitch))return false;
    gimbal_data_=d;return true;
  }
  if(code_part_==CodePart::Gimbal) {
    ChassisData d;
    if(in[12]!=0 || in[13]!=0 || in[14]!=0 || !readCommon(in,d.state,d.flags) ||
        !readNumber(in+2,10000,d.target.yaw) || !readNumber(in+4,10000,d.target.pitch) ||
        !readNumber(in+6,10000,d.feedback.yaw) || !readNumber(in+8,10000,d.feedback.pitch) ||
        !readNumber(in+10,10000,d.chassis_yaw))return false;
    chassis_data_=d;return true;
  }
  return false;
}
// 只检查对端快照的必要控制位，不能替代本板急停、设备反馈及最终输出判断。
bool robot::GimbalChassisComm::permitted(const ModuleState &state,const Flags &flags) {
  return isValid(state) && state.life==LifeState::Working && state.source==ControlSource::RemoteControl &&
      (state.chassis_mode!=ChassisMode::Disabled || state.gimbal_mode!=GimbalMode::Disabled) &&
      flags.ready && flags.imu_ready && flags.remote_valid && flags.emergency_released &&
      flags.command_valid && !flags.fault;
}
// 坏帧立即禁止旧指令；接收计时不变，不能因为不断收到错误包而保持在线。
void robot::GimbalChassisComm::reject() { inhibit_=true;rx_next_page_=0; }
// 查询当前对端指令是否仍可使用，必须同时具备完整、新鲜、未被撤销的快照。
bool robot::GimbalChassisComm::commandsUsable() const {
  if(isOffline() || inhibit_)return false;
  if(code_part_==CodePart::Chassis)return permitted(gimbal_data_.state,gimbal_data_.flags);
  if(code_part_==CodePart::Gimbal)return permitted(chassis_data_.state,chassis_data_.flags);
  return false;
}
// 云台读取 C2G 的安全快照；原始 getter 仅供调试，不能据其非零值执行动作。
bool robot::GimbalChassisComm::readPeer(ChassisData &out) const {
  out={};if(code_part_!=CodePart::Gimbal || isOffline() || inhibit_)return false;
  out=chassis_data_;return true;
}
// 底盘读取 G2C 的安全快照；离线/拒收时清空运动和目标，防止残留控制量被复用。
bool robot::GimbalChassisComm::readPeer(GimbalData &out) const {
  out={};if(code_part_!=CodePart::Chassis || isOffline() || inhibit_)return false;
  out=gimbal_data_;return true;
}
