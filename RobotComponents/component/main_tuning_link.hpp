#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>

namespace main_tuning {
// 参数通道独立于运动指令：底盘CAN2发0x212，云台CAN1回0x213；均不用于电机控制。
inline constexpr uint32_t kParameterId=0x212,kParameterAckId=0x213;
enum Field : uint8_t {Vx,Vy,YawRate,PitchRate,AimAccel,
  PositionP,PositionI,PositionD,PositionIntegral,PositionSpeed,
  SpeedP,SpeedI,SpeedD,SpeedIntegral,Torque,TargetRate,TripSpeed,PitchMin,PitchMax,PitchGuard,Count};
struct Bundle {float value[Count]{};};
static_assert(sizeof(float)==4 && sizeof(Bundle)==Count*4,"参数协议要求32位float且无结构体填充");
// 对原始字节计算CRC32，完整参数组校验通过后才允许整体应用。
inline uint32_t checksum(const void *data,unsigned size) {
  const auto *bytes=static_cast<const uint8_t *>(data);uint32_t crc=0xffffffffu;
  for(unsigned i=0;i<size;++i) {crc^=bytes[i];for(int bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320u & (0u-(crc&1u)));}
  return ~crc;
}
// 每个CAN帧另带CRC8；校验范围包含类型、序号、事务号与四字节数据。
inline uint8_t frameCrc(const uint8_t *data) {
  uint8_t crc=0;for(int i=0;i<7;++i) {crc^=data[i];for(int bit=0;bit<8;++bit)crc=(crc&0x80)?(crc<<1)^0x07:crc<<1;}return crc;
}
// 固定小端写入32位，避免指针别名或结构体填充影响线协议。
inline void putWord(uint8_t *data,uint32_t word) {for(int i=0;i<4;++i)data[i]=static_cast<uint8_t>(word>>(8*i));}
// 固定小端读取32位，调用方先检查标准帧、DLC和CRC。
inline uint32_t getWord(const uint8_t *data) {uint32_t word=0;for(int i=0;i<4;++i)word|=uint32_t(data[i])<<(8*i);return word;}
// 拒绝非有限数、负增益和不能表示的速度；机械限位还要由实际pitch控制器再次验证。
inline bool valid(const Bundle &bundle) {
  const auto &v=bundle.value;for(float x:v)if(!std::isfinite(x))return false;
  for(int i=0;i<Count;++i)if(i!=PitchMin && i!=PitchMax && v[i]<0)return false;
  return v[Vx]>0 && v[Vx]<=32.767f && v[Vy]>0 && v[Vy]<=32.767f && v[YawRate]>0 && v[PitchRate]>0 &&
    v[PositionSpeed]>0 && v[Torque]>0 && v[TargetRate]>=v[PitchRate] &&
    v[TripSpeed]>v[PositionSpeed] && v[PitchMin]<0 && v[PitchMax]>0 &&
    v[PitchMin]>-3.14159265f && v[PitchMax]<3.14159265f && v[PitchGuard]<(v[PitchMax]-v[PitchMin])*0.5f;
}
// 参数下发源；发送失败不推进页号，ACK只确认整组CRC且有独立新鲜度。
class Source {
 public:
  // 只在启动时装入底盘编译所得参数，非法参数始终不发送、不就绪。
  void load(const Bundle &bundle) {bundle_=bundle;valid_=valid(bundle);hash_=checksum(&bundle_,sizeof(bundle_));
    loaded_=true;ack_=false;page_=0;++transaction_;}
  // 查询是否已装载，避免每个1ms周期重复计算整组CRC。
  bool loaded() const {return loaded_;}
  // 生成当前页而不推进，最后一帧为提交CRC；每组使用新的事务号。
  bool frame(uint8_t data[8]) const {
    if(!valid_)return false;
    data[0]=0xd1;data[1]=page_==0?0xfe:(page_==Count+1?0xff:page_-1);data[2]=transaction_;
    uint32_t word=hash_;if(page_>0 && page_<=Count)std::memcpy(&word,&bundle_.value[page_-1],4);
    putWord(data+3,word);data[7]=frameCrc(data);return true;
  }
  // 仅在HAL成功排队后推进；完整组循环下发，也为对端提供参数租约。
  void sent() {if(++page_==Count+2) {page_=0;++transaction_;}}
  // 接收完整组确认；错误CRC/版本/哈希不能续租。
  bool receive(const uint8_t data[8],uint32_t now) {
    if(!loaded_ || !valid_ || data[0]!=0xd2 || data[1]!=Count || data[2]!=1 || data[7]!=frameCrc(data) || getWord(data+3)!=hash_)return false;
    ack_ms_=now;ack_=true;return true;
  }
  // 两端都需新鲜；即使运动协议仍在线，参数ACK失联也撤销动力许可。
  bool ready(uint32_t now) const {return valid_ && ack_ && now-ack_ms_<200;}
  // 只读哈希用于Ozone核对，不从诊断镜像反向取许可。
  uint32_t hash() const {return hash_;}
 private:
  Bundle bundle_{};uint32_t hash_=0,ack_ms_=0;uint8_t page_=0,transaction_=0;bool valid_=false,loaded_=false,ack_=false;
};
// 参数接收端只缓存；CAN中断绝不修改PID，完整提交后等待控制周期右下急停应用。
class Sink {
 public:
  // 顺序拼装同事务参数，拒绝缺页、错序、超时、CRC错误与非有限参数。
  bool receive(const uint8_t data[8],uint32_t now) {
    if(data[0]!=0xd1 || data[7]!=frameCrc(data))return false;
    const uint8_t page=data[1],transaction=data[2];const uint32_t word=getWord(data+3);
    if(page==0xfe) {assembling_=true;expected_=0;transaction_=transaction;begin_ms_=now;assembly_hash_=word;
      desired_hash_=word;return true;}
    if(!assembling_ || transaction!=transaction_ || now-begin_ms_>180) {assembling_=false;return false;}
    if(page<Count && page==expected_) {std::memcpy(&assembly_.value[page],&word,4);++expected_;return true;}
    assembling_=false;
    if(page!=0xff || expected_!=Count || word!=assembly_hash_ || checksum(&assembly_,sizeof(assembly_))!=word || !valid(assembly_))return false;
    pending_=assembly_;pending_hash_=word;complete_ms_=now;have_pending_=true;return true;
  }
  // 仅返回尚未应用的完整组；重复相同配置只是续租，不重复清除积分。
  bool pending(Bundle &bundle,uint32_t &hash,uint32_t now) const {
    if(!have_pending_ || now-complete_ms_>=400 || pending_hash_!=desired_hash_ || (applied_ && pending_hash_==applied_hash_))return false;
    bundle=pending_;hash=pending_hash_;return true;
  }
  // 仅由右下急停且PID/驱动验证全部成功的控制周期调用。
  void applied(uint32_t hash) {applied_hash_=hash;applied_=true;}
  // 未应用、参数不匹配或整组租约过期均不能就绪。
  bool ready(uint32_t now) const {return applied_ && have_pending_ && desired_hash_==applied_hash_ && pending_hash_==applied_hash_ && now-complete_ms_<400;}
  // 仅对已经应用且新鲜的整组返回ACK，不把收到某一页等同于参数生效。
  bool frame(uint8_t data[8],uint32_t now) const {
    if(!ready(now))return false;
    data[0]=0xd2;data[1]=Count;data[2]=1;putWord(data+3,applied_hash_);data[7]=frameCrc(data);return true;
  }
  // 只读生效哈希，供现场确认两板一致。
  uint32_t hash() const {return applied_hash_;}
 private:
  Bundle assembly_{},pending_{};uint32_t begin_ms_=0,complete_ms_=0,assembly_hash_=0,desired_hash_=0,pending_hash_=0,applied_hash_=0;
  uint8_t transaction_=0,expected_=0;bool assembling_=false,have_pending_=false,applied_=false;
};
// 两个固件有各自独立内存；源只在底盘使用，接收端只在云台使用，调用处持短临界区。
inline Source parameter_source;
inline Sink parameter_sink;
struct Diagnostics {uint32_t ready=0,hash=0,applied_count=0;float pitch_position_p=0,pitch_speed_p=0,pitch_torque_nm=0,max_vx_m_s=0,yaw_rate_rad_s=0,pitch_rate_rad_s=0;};
}
extern "C" {inline volatile main_tuning::Diagnostics robot_tuning_diagnostics;}
