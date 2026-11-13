#ifndef VEHICLE_MOTOR_CONFIG_HPP_
#define VEHICLE_MOTOR_CONFIG_HPP_

#include <cstdint>

// 两板共用的实车配置表：这里只描述设备，不引用 HAL 或创建电机对象。
// 换车时先核对铭牌、总线和设备 ID，再修改对应配置，不能仅凭拓扑图推定型号。
namespace vehicle_config {
// 几何参数按用户指定采用考核说明；车体系 x 向前、y 向左，长度单位均为米。
// 此处只保存常量，运动学逆解留到任务清单第六项实现。
inline constexpr float kWheelRadiusM = 0.066f;
struct WheelPosition {
  float x_m;
  float y_m;
};
inline constexpr WheelPosition kWheelPositions[4] = {
    {0.167f, 0.167f},     // 左前。
    {-0.167f, 0.167f},    // 左后。
    {-0.167f, -0.167f},   // 右后。
    {0.167f, -0.167f},    // 右前。
};

enum class Board : uint8_t { Chassis, Gimbal };
enum class CanBus : uint8_t { Can1, Can2 };
// 按用户确认的实车接线，两板 CAN2 并接在同一条物理总线上。
// 板间协议也使用各板本地的 hcan2；其收发实现留到通信协议任务。
inline constexpr CanBus kBoardCommBus = CanBus::Can2;
enum class Model : uint8_t { Unconfigured, M3508, GM6020, DM_J4310 };

struct MotorConfig {
  Board board;        // 唯一控制责任板；共线设备可被两板接收，但只能由责任板发控制命令。
  CanBus bus;         // 责任板本地 CAN 接口，两板 Can2 对应同一条物理总线。
  Model model;
  uint8_t id;          // 设备 ID，不是 CAN 反馈帧 ID；0 表示轮位对应关系尚未确认。
  bool reversed;      // true 表示电机正方向与规定正方向相反。
  float offset_rad;   // 方向确定后标定的角度偏移，单位 rad。
  bool configured;     // 配置是否已录入；不代表允许输出，动作许可由后续安全状态机判断。
};

// 数组下标固定为：0 左前、1 左后、2 右后、3 右前。
// 图中只给出设备 ID 范围 1～4，没有给出轮位对应关系，故不猜测分配。
// 方向和偏移暂录考核说明中的数值；两辆车仍需分别核对、标定。
// 上述未分配说明为早期记录；用户现已确认：驱动、转向均为左前1、左后2、右后3、右前4。
// 按确认结果补齐原待填 ID；configured 仅代表配置录入，不放开动力及其他安全联锁。
// 此对应关系用于本次确认的车辆，另一辆车使用前仍须独立核对。
inline constexpr MotorConfig kWheels[4] = {
    {Board::Chassis, CanBus::Can2, Model::M3508, 1, false, 0.0f, true},
    {Board::Chassis, CanBus::Can2, Model::M3508, 2, false, 0.0f, true},
    {Board::Chassis, CanBus::Can2, Model::M3508, 3, false, 0.0f, true},
    {Board::Chassis, CanBus::Can2, Model::M3508, 4, false, 0.0f, true},
};
inline constexpr MotorConfig kSteers[4] = {
    {Board::Chassis, CanBus::Can1, Model::GM6020, 1, true, 2.61346f, true},
    {Board::Chassis, CanBus::Can1, Model::GM6020, 2, true, -2.06844f, true},
    {Board::Chassis, CanBus::Can1, Model::GM6020, 3, true, 0.00729f, true},
    {Board::Chassis, CanBus::Can1, Model::GM6020, 4, true, 1.58518f, true},
};

// 以用户确认的实车接线为准：yaw、pitch 与驱动轮共用两板相连的 CAN2。
// 软件分工为底盘控制 yaw、云台控制 pitch；board 表示责任归属，不代表总线隔离。
// 按用户要求暂按 DM_J4310 建模；方向、偏移暂用考核说明值，现场仍需复核。
// configured=true 只允许创建零输入实例；不启用电机，不代表实物已验证。
inline constexpr MotorConfig kYaw{
    Board::Chassis, CanBus::Can2, Model::DM_J4310, 1, false, 1.60879f, true};
inline constexpr MotorConfig kPitch{
    Board::Gimbal, CanBus::Can2, Model::DM_J4310, 2, true, 2.91772f, true};
}  // namespace vehicle_config

#endif
