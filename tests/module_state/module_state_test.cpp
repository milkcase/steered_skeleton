#include "module_state.hpp"
#include <limits>
#include <type_traits>
#include <cstddef>

// 生产编译时额外包含两板现有接口，检查共享类型与此前各项封装能否共存。
#if defined(TEST_PROJECT_HEADERS)
#include "ins_robot.hpp"
#include "ins_pid.hpp"
#include "ins_imu.hpp"
#include "ins_comm.hpp"
#include "gimbal_chassis_comm.hpp"
#endif

using namespace robot;
static_assert(sizeof(LifeState)==1 && sizeof(ControlSource)==1 && sizeof(ChassisMode)==1 && sizeof(GimbalMode)==1);
static_assert(!std::is_convertible_v<LifeState,int>);
static_assert(!std::is_convertible_v<ChassisMotion,GimbalMotion>);
static_assert(std::is_standard_layout_v<ChassisMotion> && std::is_trivially_copyable_v<ChassisMotion>);
static_assert(std::is_standard_layout_v<GimbalMotion> && std::is_trivially_copyable_v<GimbalMotion>);
static_assert(ModuleState{}.life==LifeState::Dead && ModuleState{}.source==ControlSource::None);
static_assert(ModuleState{}.chassis_mode==ChassisMode::Disabled && ModuleState{}.gimbal_mode==GimbalMode::Disabled);
static_assert(ChassisMotion{}.vx==0 && ChassisMotion{}.vy==0 && ChassisMotion{}.w==0);
static_assert(GimbalMotion{}.yaw==0 && GimbalMotion{}.pitch==0);
constexpr ChassisMotion compile_chassis=(ChassisMotion{1,2,3}+ChassisMotion{3,2,1})*0.5f;
static_assert(compile_chassis.vx==2 && compile_chassis.vy==2 && compile_chassis.w==2);
constexpr GimbalMotion compile_gimbal=2.0f*(GimbalMotion{3,1}-GimbalMotion{1,3});
static_assert(compile_gimbal.yaw==4 && compile_gimbal.pitch==-4);

// 为裸 ARM 测试提供内存填充实现，不访问任何硬件。
extern "C" void *memset(void *p,int value,std::size_t n) {
  auto *out=static_cast<unsigned char *>(p);
  for(std::size_t i=0;i<n;++i)out[i]=value;
  return p;
}
// 为裸 ARM 测试提供非重叠内存复制实现，返回目标地址。
extern "C" void *memcpy(void *p,const void *source,std::size_t n) {
  auto *out=static_cast<unsigned char *>(p);const auto *in=static_cast<const unsigned char *>(source);
  for(std::size_t i=0;i<n;++i)out[i]=in[i];
  return p;
}
#define CHECK(condition) do { if(!(condition))return __LINE__; } while(0)

// 测试默认状态、全部枚举字节、运动量运算与非法数值；返回零或失败行号。
extern "C" int run_tests() {
  ModuleState state;
  CHECK(isValid(state));
  // 穷举所有可能的协议字节，保证预留/未知模式不会被识别为已定义值。
  for(unsigned value=0;value<256;++value) {
    CHECK(isValid(static_cast<LifeState>(value))==(value<=2));
    CHECK(isValid(static_cast<ControlSource>(value))==(value<=1));
    CHECK(isValid(static_cast<ChassisMode>(value))==(value<=3));
    CHECK(isValid(static_cast<GimbalMode>(value))==(value<=1));
  }
  state.life=static_cast<LifeState>(255);CHECK(!isValid(state));state={};
  state.source=static_cast<ControlSource>(2);CHECK(!isValid(state));state={};
  state.chassis_mode=static_cast<ChassisMode>(4);CHECK(!isValid(state));state={};
  state.gimbal_mode=static_cast<GimbalMode>(2);CHECK(!isValid(state));

  const ChassisMotion a{1,-2,3},b{-3,2,1};
  const ChassisMotion sum=a+b,difference=a-b,negative=-a;
  CHECK(sum.vx==-2 && sum.vy==0 && sum.w==4);
  CHECK(difference.vx==4 && difference.vy==-4 && difference.w==2);
  CHECK(negative.vx==-1 && negative.vy==2 && negative.w==-3);
  CHECK(a.vx==1 && a.vy==-2 && a.w==3);  // 非复合运算不得改变输入。
  auto motion=a;
  CHECK(&(motion+=b)==&motion);CHECK(&(motion-=b)==&motion);
  CHECK(motion.vx==a.vx && motion.vy==a.vy && motion.w==a.w);
  CHECK(&(motion*=2)==&motion);
  CHECK(motion.vx==2 && motion.vy==-4 && motion.w==6);
  motion+=motion;CHECK(motion.vx==4 && motion.vy==-8 && motion.w==12);
  CHECK((0.5f*motion).w==6 && (motion*0).vx==0 && (motion*(-1)).vy==8);

  const GimbalMotion target{7,2},feedback{1,4};
  const GimbalMotion error=target-feedback,combined=target+feedback,reverse=-feedback;
  CHECK(error.yaw==6 && error.pitch==-2);  // 不自动把普通角度相减变为最短角差。
  CHECK(combined.yaw==8 && combined.pitch==6 && reverse.yaw==-1 && reverse.pitch==-4);
  auto joint=target;CHECK(&(joint-=feedback)==&joint);CHECK(&(joint+=feedback)==&joint);
  CHECK(joint.yaw==7 && joint.pitch==2 && target.yaw==7);
  CHECK(&(joint*=2)==&joint);CHECK(joint.yaw==14 && joint.pitch==4);
  joint+=joint;CHECK(joint.yaw==28 && joint.pitch==8);
  CHECK((0.5f*joint).yaw==14 && (joint*0).pitch==0 && (joint*(-1)).yaw==-28);

  CHECK(isFinite(ChassisMotion{}) && isFinite(GimbalMotion{}));
  const float bad[]={std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),-std::numeric_limits<float>::infinity()};
  for(float value:bad) {
    CHECK(!isFinite(ChassisMotion{value,0,0}));
    CHECK(!isFinite(ChassisMotion{0,value,0}));
    CHECK(!isFinite(ChassisMotion{0,0,value}));
    CHECK(!isFinite(GimbalMotion{value,0}));
    CHECK(!isFinite(GimbalMotion{0,value}));
  }
  const float large=std::numeric_limits<float>::max();
  CHECK(!isFinite(ChassisMotion{large,0,0}*2));
  CHECK(!isFinite(GimbalMotion{0,large}*2));
  return 0;
}
