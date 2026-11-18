#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>

namespace pitch_feedback {
// 独立只读观测，不参与生命状态、控制许可或电机反馈续命；9页原样传送float。
inline constexpr uint32_t kId=0x214,kOfflineMs=50,kAssemblyMs=30;
inline constexpr uint8_t kPages=9;
struct Sample {
  float angle=0,speed=0,torque=0;bool valid=false;
  float position_target=0,speed_target=0,torque_command=0,feedforward=0,pid_torque=0,position_error=0;
  bool control=false,torque_limited=false,feedforward_endpoint=false;
};
class Link {
 public:
  // 冻结本周期全部目标、反馈和状态；发送分页开始后不再读取变化中的local。
  void publish(const Sample &sample,uint32_t now) {local_=sample;local_ms_=now;}
  // 保留只有实测反馈的调用入口，其余目标默认为零，供协议边界测试使用。
  void publish(float angle,float speed,float torque,bool valid,uint32_t now) {
    Sample s;s.angle=angle;s.speed=speed;s.torque=torque;s.valid=valid;publish(s,now);
  }
  // 开始时生成完整9页快照；每页4字节float及CRC，不额外限定可显示的数值范围。
  void frame(uint8_t out[8],uint32_t now) {
    if(frozen_ && now-tx_start_>=kAssemblyMs) {page_=0;frozen_=false;++sequence_;}
    if(!frozen_) {
      const float values[kPages]={local_.angle,local_.speed,local_.torque,local_.position_target,local_.speed_target,
        local_.torque_command,local_.feedforward,local_.pid_torque,local_.position_error};
      bool valid=local_.valid && now-local_ms_<5;
      for(float v:values)valid=valid && std::isfinite(v);
      const uint8_t flags=0x80u|(valid?0x40u:0)|(local_.control?0x20u:0)|
        (local_.torque_limited?0x10u:0)|(local_.feedforward_endpoint?8u:0);
      for(int p=0;p<kPages;++p) {
        auto *f=tx_[p];f[0]=flags;f[1]=sequence_;f[2]=p;
        uint32_t word=0;const float v=valid?values[p]:0;std::memcpy(&word,&v,4);
        for(int j=0;j<4;++j)f[3+j]=word>>(8*j);f[7]=crc(f);
      }
      tx_start_=now;frozen_=true;
    }
    std::memcpy(out,tx_[page_],8);
  }
  // 仅成功排队后推进页号；整组发完再换序号，邮箱忙不能跳页。
  void sent() {if(++page_==kPages) {page_=0;frozen_=false;++sequence_;}}
  // CRC/页序/整组超时失败立即撤销观测；9页全部来自同组才原子提交，不混用新旧目标和反馈。
  bool receive(const uint8_t in[8],uint32_t now) {
    if((in[0]&0x87u)!=0x80u || in[2]>=kPages || in[7]!=crc(in))return reject();
    const uint8_t page=in[2],seq=in[1];
    if(page==0) {
      const uint8_t delta=seq-last_sequence_;
      if(have_ && now-remote_ms_<kOfflineMs && (delta==0 || delta>128))return false;
      next_=0;rx_sequence_=seq;rx_flags_=in[0];rx_start_=now;
      if(!(rx_flags_&0x40u))remote_.valid=false;
    }
    if(page!=next_ || seq!=rx_sequence_ || in[0]!=rx_flags_ || now-rx_start_>=kAssemblyMs)return reject();
    uint32_t word=0;for(int j=0;j<4;++j)word|=uint32_t(in[3+j])<<(8*j);
    float v;std::memcpy(&v,&word,4);if(!std::isfinite(v))return reject();
    rx_[page]=v;++next_;
    if(next_==kPages) {
      Sample s;s.angle=rx_[0];s.speed=rx_[1];s.torque=rx_[2];s.valid=(rx_flags_&0x40u)!=0;
      s.position_target=rx_[3];s.speed_target=rx_[4];s.torque_command=rx_[5];
      s.feedforward=rx_[6];s.pid_torque=rx_[7];s.position_error=rx_[8];
      s.control=rx_flags_&0x20u;s.torque_limited=rx_flags_&0x10u;s.feedforward_endpoint=rx_flags_&8u;
      remote_=s;remote_ms_=rx_start_;last_sequence_=seq;have_=true;++count_;next_=0;
    }
    return true;
  }
  // 超时按整组第一页的本机接收时刻计算，不能用后续页把旧采样重新变新。
  Sample read(uint32_t now) const {return have_ && now-remote_ms_<kOfflineMs && remote_.valid?remote_:Sample{};}
  // 返回本机观测年龄；未知时为最大无符号数，不使用对端上电时钟。
  uint32_t age(uint32_t now) const {return have_?now-remote_ms_:0xffffffffu;}
  // 返回完整组提交计数，用于判断Ozone是否仍在更新。
  uint32_t count() const {return count_;}
 private:
  // 只撤销观测缓存；不调用停机、使能或电机接口。
  bool reject() {remote_.valid=false;next_=0;return false;}
  // CRC8保护版本/有效位/运行标志、组序号、页号和float数据。
  static uint8_t crc(const uint8_t *data) {
    uint8_t c=0;for(int i=0;i<7;++i) {c^=data[i];for(int j=0;j<8;++j)c=(c&128)?(c<<1)^7:c<<1;}return c;
  }
  static_assert(sizeof(float)==4,"观测协议要求32位float");
  Sample local_{},remote_{};float rx_[kPages]{};uint8_t tx_[kPages][8]{};
  uint32_t local_ms_=0,remote_ms_=0,count_=0,tx_start_=0,rx_start_=0;
  uint8_t sequence_=0,last_sequence_=0,page_=0,next_=0,rx_sequence_=0,rx_flags_=0;
  bool have_=false,frozen_=false;
};
inline Link channel;
}
