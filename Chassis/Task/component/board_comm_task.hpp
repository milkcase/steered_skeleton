#ifndef STEERED_BOARD_COMM_TASK_HPP_
#define STEERED_BOARD_COMM_TASK_HPP_
#include "gimbal_chassis_comm.hpp"
// 在初始化阶段启动共享 CAN2 接收；失败返回 false，不使能或发送任何电机。
bool StartBoardCommunication();
// 发送一页板间状态/指令，返回 HAL 是否成功；周期调度在第九项接入。
bool SendBoardCommunication();
// 原子复制新鲜完整的对端状态；可读取安全/未 Working 状态，用于启动协商，不能直接执行指令。
bool ReadPeerBoard(robot::GimbalChassisComm::GimbalData &out);
// 同一临界区内检查对端控制许可并复制快照；失败清空输出，仍须本板最终门控。
bool ReadPeerCommand(robot::GimbalChassisComm::GimbalData &out);
// 原子查询新鲜、完整、可用的对端指令；仍不是最终电机输出许可。
bool PeerCommandsUsable();
#endif
