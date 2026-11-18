#pragma once
// 两份Runtime/发送层各持本地电机、状态机和反馈记录；只把线上CAN帧交给对端。
struct DualMainBoard {
  bool chassis;
  MotorFeedback feedback;
  DM_J4310 axis;
  hello_world::pid::Pid pid;
  Gimbal loop;
  RobotRuntime runtime;
  RobotCanService sender;
  GimbalChassisComm comm;
  hello_world::imu::Imu imu;
  bool enabled=false;
  pitch_feedback::Sample frozen_pitch{},delivered_pitch{};
  uint32_t now_ms=0;
  // 显式绑定各板独占关节；模拟器内两板的MotorFeedback记录分开装入，避免共享续命。
  DualMainBoard(bool base) : chassis(base),pid(base?pid_config::kYaw:pid_config::kPitch),
    loop(base?Gimbal::Axis::Yaw:Gimbal::Axis::Pitch,&axis,&pid,base?gimbal_control_config::kYaw:gimbal_control_config::kPitch),
    runtime(base?Board::Chassis:Board::Gimbal,kControl),sender(&runtime,base),
    comm(base?GimbalChassisComm::CodePart::Chassis:GimbalChassisComm::CodePart::Gimbal,base?0x210:0x211,base?0x211:0x210) {
    OptionalParams opt;opt.input_type=InputType::kTorq;opt.angle_range=AngleRange::kNegPiToPosPi;
    axis.init(base?1:2,opt,false,false);loop.configure(base?gimbal_control_config::kYaw:gimbal_control_config::kPitch);
    runtime.registerAxis(&axis,&loop,&pid);runtime.registerCommunication(&comm);runtime.registerImu(&imu);
    hello_world::imu::BMI088HWConfig hw;const float rotation[9]={1,0,0,0,1,0,0,0,1};
    runtime.initializeImu(hw,rotation,true);runtime.setTransportReady(sender.init());
  }
  // 本地达妙只在实际发送FC后报告使能；FD后下一周期反馈禁用，不预先假定握手成功。
  void receiveAxis() {
    uint8_t data[8]={static_cast<uint8_t>((enabled?0x10:0)|axis.motor_info().id),0x80,0,0x80,0,0,30,30};
#if defined(TEST_DUAL_PITCH_OUTSIDE) || defined(TEST_DUAL_PITCH_FREE)
    if(!chassis) {
      const uint16_t code=static_cast<uint16_t>((0.7f+3.141593f)/(2*3.141593f)*65535);
      data[1]=static_cast<uint8_t>(code>>8);data[2]=static_cast<uint8_t>(code);
    }
#endif
    axis.decode(data);MotorFeedbacks().recordAxis(axis.motor_info().id,data,test_now);
  }
};
// 交付本次发送的线帧；0x210/211走真实解码，0x212/213走真实参数校验，电机FC/FD更新独立模型。
void drainDualFrames(DualMainBoard &from,DualMainBoard &to) {
  for(uint32_t i=0;i<frame_count;++i) {
    const auto &frame=frames[i];
    if(frame.can==&hcan2 && frame.id==from.axis.motor_info().id && frame.data[0]==0xff && frame.data[6]==0xff) {
      if(frame.data[7]==0xfc)from.enabled=true;
      if(frame.data[7]==0xfd)from.enabled=false;
    }
    if(frame.can!=(from.chassis?&hcan2:&hcan1))continue;
    const uint32_t saved_now=test_now;test_now=to.now_ms;
    if(frame.id==to.comm.rxId())to.comm.decode(frame.data);
    if(from.chassis && frame.id==main_tuning::kParameterId)main_tuning::parameter_sink.receive(frame.data,test_now);
    if(!from.chassis && frame.id==main_tuning::kParameterAckId)main_tuning::parameter_source.receive(frame.data,test_now);
    if(!from.chassis && frame.id==pitch_feedback::kId) {
      // 记录发出第一页时的真实目标；后续控制目标继续移动，不能拿收齐后的新目标比较旧快照。
      if(frame.data[2]==0) {
        const auto &r=from.loop.result();auto &s=from.frozen_pitch;
        s.position_target=r.target_rad;s.speed_target=r.target_speed_rad_s;s.torque_command=r.torque_nm;
        s.feedforward=r.feedforward_nm;s.pid_torque=r.pid_torque_nm;s.position_error=r.error_rad;
      }
      pitch_feedback::channel.receive(frame.data,test_now);
      if(frame.data[2]+1==pitch_feedback::kPages)to.delivered_pitch=from.frozen_pitch;
    }
    test_now=saved_now;
  }
  frame_count=0;
}
// 真实发送层联动：右下同步→右中实际FC握手→右上，覆盖两板控制/通讯先后顺序的错相。
int runDualMain() {
  DualMainBoard base(true),cloud(false);
  M3508 wheels[4]={M3508(1),M3508(2),M3508(3),M3508(4)};
  GM6020 steers[4]={GM6020(1),GM6020(2),GM6020(3),GM6020(4)};
  hello_world::pid::Pid wheel_pid[4],steer_pid[4];Motor *wp[4],*sp[4];hello_world::pid::Pid *wpp[4],*spp[4];
  for(int i=0;i<4;++i) {
    OptionalParams opt;opt.input_type=InputType::kCurr;steers[i].init(i+1,opt);
    wp[i]=&wheels[i];sp[i]=&steers[i];wpp[i]=&wheel_pid[i];spp[i]=&steer_pid[i];
    wheel_pid[i].configure(pid_config::kWheel[i]);steer_pid[i].configure(pid_config::kSteer[i]);
  }
  Chassis::Solver solver(chassis_ik_config::kLimits);Chassis chassis(&solver);
  DriveController drive(&chassis,wp,sp,wpp,spp,&solver);base.runtime.registerDrive(&drive);base.sender.registerDrive(wp,sp);
  frame_count=0;
  const bool test_recovery=
#if defined(TEST_DUAL_PITCH_OUTSIDE) || defined(TEST_DUAL_PITCH_FREE)
      false;
#else
      true;
#endif
  for(uint32_t elapsed=1;elapsed<(test_recovery?1600u:1200u);++elapsed) {
    base.now_ms=elapsed;cloud.now_ms=elapsed+1373; // 各自上电时间不同，不能把对端时间用于续租。
    uint8_t selector=elapsed<300?2:(elapsed<800?3:1);
    if(test_recovery) {
      // 注入两板采样锁存；始终右上不能恢复。右下重新校准，右中走完整握手，再右上运行。
      if(elapsed==1050) {
        base.imu.ready=cloud.imu.ready=false;
        base.imu.allow_restart=cloud.imu.allow_restart=true;
      }
      if(elapsed>=1110 && elapsed<1200)selector=2;
      if(elapsed>=1200 && elapsed<1480)selector=3;
    }
    setRemote(selector);
    // 交替改变两个定时器的先后次序；每次还原本板反馈区，与独立MCU的存储行为一致。
    for(int phase=0;phase<2;++phase) {
      const bool base_turn=(phase==0)==((elapsed/7)%2==0);
      auto &board=base_turn?base:cloud;auto &other=base_turn?cloud:base;
      test_now=board.now_ms;MotorFeedbacks()=board.feedback;local_comm=&board.comm;
      board.runtime.pollSensors();board.receiveAxis();
      if(base_turn)for(int i=0;i<4;++i) {
        uint8_t data[8]{};data[6]=30;wheels[i].decode(data);steers[i].decode(data);
        MotorFeedbacks().recordDji(i,data,test_now);MotorFeedbacks().recordDji(i+4,data,test_now);
      }
      board.runtime.update();
      if(base_turn && elapsed>=700 && elapsed<800) {
        CHECK(robot_main_diagnostics.pitch_feedback_valid);
        CHECK(std::fabs(robot_main_diagnostics.pitch_feedback_angle_rad-cloud.axis.angle())<0.000001f);
      }
      board.sender.tick();board.feedback=MotorFeedbacks();drainDualFrames(board,other);
    }
    if(elapsed>=700 && elapsed<800)CHECK(base.enabled && cloud.enabled);
    if(elapsed>=700 && elapsed<800) {
      const auto pitch=pitch_feedback::channel.read(base.now_ms);
      CHECK(pitch.valid && std::fabs(pitch.angle-cloud.axis.angle())<0.00011f);
      CHECK(std::fabs(pitch.speed-cloud.axis.vel())<0.0051f && std::fabs(pitch.torque-cloud.axis.torq())<0.00051f);
    }
    if(test_recovery && elapsed>=1050 && elapsed<1480) {
      CHECK(!base.runtime.output().control && !cloud.runtime.output().control);
      if(elapsed<1110)CHECK(!base.imu.ready && !cloud.imu.ready);
      if(elapsed>=1180)CHECK(base.imu.ready && cloud.imu.ready);
    }
    if(elapsed>=900 && (!test_recovery || elapsed<1050 || elapsed>=1520)) {
      CHECK(base.runtime.output().control && cloud.runtime.output().control);
#if defined(TEST_DUAL_PITCH_OUTSIDE)
      CHECK(cloud.loop.result().target_rad<=cloud.loop.params().pitch_max_rad);
      CHECK(cloud.loop.result().torque_nm<0);
#endif
      // 恢复双板控制后，遥测还需等下一组9页收齐，两者各按自己的时限检查。
      if(test_recovery && elapsed>=1520 && elapsed<1560)continue;
      const auto pitch=pitch_feedback::channel.read(base.now_ms);
      CHECK(pitch.valid && pitch.control);
      CHECK(pitch.position_target==base.delivered_pitch.position_target);
      CHECK(pitch.speed_target==base.delivered_pitch.speed_target);
      CHECK(pitch.torque_command==base.delivered_pitch.torque_command);
      CHECK(pitch.feedforward==base.delivered_pitch.feedforward && pitch.pid_torque==base.delivered_pitch.pid_torque);
      CHECK(pitch.position_error==base.delivered_pitch.position_error);
    }
  }
  return 0;
}
