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
  void receiveAxis(float simulated_yaw=0) {
    uint8_t data[8]={static_cast<uint8_t>((enabled?0x10:0)|axis.motor_info().id),0x80,0,0x80,0,0,30,30};
    if(chassis && simulated_yaw!=0) {
      const uint16_t code=static_cast<uint16_t>((simulated_yaw+3.141593f)/(2*3.141593f)*65535);
      data[1]=static_cast<uint8_t>(code>>8);data[2]=static_cast<uint8_t>(code);
    }
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
#if defined(TEST_DUAL_PITCH_OUTSIDE) || defined(TEST_DUAL_PITCH_FREE) || defined(TEST_DUAL_FOLLOW)
      false;
#else
      true;
#endif
#if defined(TEST_DUAL_FOLLOW)
  constexpr uint32_t spin_end_ms=1960,stop_ms=2080,end_ms=2160;
  float held_pitch=0;
  float held_chassis_heading=0;
  bool corrected_stale_reference=false;
#else
  const uint32_t end_ms=test_recovery?1600u:1200u;
#endif
  for(uint32_t elapsed=1;elapsed<end_ms;++elapsed) {
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
#if defined(TEST_DUAL_FOLLOW)
    // 右上不动完成跟随→分离→小陀螺→跟随，最后右下验证急停仍撤销准备许可。
    const uint8_t left=elapsed<1100?3:(elapsed<1210?1:(elapsed<spin_end_ms?2:3));
    const float simulated_yaw=elapsed<980?0.30f:((elapsed>=1000 && elapsed<1060)?-0.30f:0.001f);
    // 底盘实测转速故意与逆解指令不同，确认补偿来自本板IMU而非转速目标。
    base.imu.turn_rate=elapsed>=850 && elapsed<1440?0.4f:0;
    // 右杆水平保持回中，模拟底盘航向在两份完整快照之间继续变化。
    if(elapsed>=880 && elapsed<960)base.imu.heading+=0.0004f;
    // 分离模式下模拟车头左偏、右偏以及回到原朝向，云台目标独立保持。
    if(elapsed==1160)base.imu.heading+=0.10f;
    if(elapsed==1180 || elapsed==1181)base.imu.heading-=0.10f;
    if(elapsed==1200)base.imu.heading+=0.10f;
    if(elapsed>=stop_ms && elapsed<stop_ms+20)selector=2;
    // 上电自检阶段故意偏拨轮；自旋到稳态后拨轮从回中向两侧变化再回来。
    const uint16_t wheel=elapsed<800 || (elapsed>=1760 && elapsed<1810)?1354:
        ((elapsed>=1810 && elapsed<1870)?694:1024);
    const uint16_t pitch_stick=elapsed>=1040 && elapsed<1080?1156:1024;
    // 自转稳定后，依次满杆向前、斜向满杆，再回中。与拨轮变化一同经过真实板间通信。
    const uint16_t forward=elapsed>=1760 && elapsed<1870?1684:1024;
    const uint16_t sideways=elapsed>=1810 && elapsed<1870?1684:1024;
#else
    const uint8_t left=1;
    const float simulated_yaw=0;
    const uint16_t wheel=1024,pitch_stick=1024;
    const uint16_t forward=1024,sideways=1024;
#endif
    setRemote(selector,pitch_stick,sideways,forward,left,wheel);
#if defined(TEST_DUAL_FOLLOW)
    if(elapsed==1100) {
      // 遥控帧刚好在控制算完、CAN发送前到达，真实发送层必须暂停但保留准备许可。
      test_now=cloud.now_ms;local_comm=&cloud.comm;MotorFeedbacks()=cloud.feedback;
      cloud.sender.tick();drainDualFrames(cloud,base);
      CHECK(!cloud.runtime.output().control && cloud.runtime.output().axis_action==AxisAction::Zero);
    }
#endif
    // 交替改变两个定时器的先后次序；每次还原本板反馈区，与独立MCU的存储行为一致。
    for(int phase=0;phase<2;++phase) {
      const bool base_turn=(phase==0)==((elapsed/7)%2==0);
      auto &board=base_turn?base:cloud;auto &other=base_turn?cloud:base;
      test_now=board.now_ms;MotorFeedbacks()=board.feedback;local_comm=&board.comm;
      board.runtime.pollSensors();board.receiveAxis(simulated_yaw);
      if(base_turn)for(int i=0;i<4;++i) {
        uint8_t data[8]{};data[6]=30;wheels[i].decode(data);steers[i].decode(data);
        MotorFeedbacks().recordDji(i,data,test_now);MotorFeedbacks().recordDji(i+4,data,test_now);
      }
      board.runtime.update();
#if defined(TEST_DUAL_FOLLOW)
      if(base_turn && ((elapsed>=850 && elapsed<980) || (elapsed>=1140 && elapsed<1210) || (elapsed>=1250 && elapsed<1380))) {
        const float expected=-base.imu.turn_rate;
        CHECK(std::fabs(base.loop.result().chassis_rate_correction_rad_s-expected)<0.00001f);
        CHECK(std::fabs(robot_main_diagnostics.yaw_chassis_compensation_rpm-expected/main_tuning::rpm)<0.0001f);
        const float world_rpm=(base.axis.vel()+base.imu.turn_rate)/main_tuning::rpm;
        CHECK(std::fabs(robot_main_diagnostics.yaw_world_feedback_rpm-world_rpm)<0.0001f);
        CHECK(robot_main_diagnostics.yaw_world_target_valid==(elapsed<1210));
        if(elapsed<1210) {
          const auto &d=robot_main_diagnostics;
          CHECK(std::fabs(d.yaw_world_target_rad-0.30f)<0.001f);
          CHECK(std::fabs(d.yaw_target_rad+base.imu.heading-d.yaw_world_target_rad)<0.00001f);
          CHECK(std::fabs(d.yaw_world_angle_rad-base.axis.angle()-base.imu.heading)<0.00001f);
          if(std::fabs(base.runtime.output().target.yaw-base.comm.gimbal_data().yaw_target)>0.001f)
            corrected_stale_reference=true;
        }
      }
#endif
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
#if defined(TEST_DUAL_FOLLOW)
    if(elapsed>=850 && elapsed<1100) {
      const auto &o=base.runtime.output();
      CHECK(o.control && cloud.runtime.output().control && o.state.chassis_mode==ChassisMode::Follow);
      CHECK(std::fabs(o.motion.w)<=ROBOT_FOLLOW_MAX_RPM*main_tuning::rpm+0.00001f);
      if(elapsed<980)CHECK(o.motion.w>0 && o.follow_error_rad>0.29f);
      if(elapsed>=1040 && elapsed<1060)CHECK(o.motion.w<0 && o.follow_error_rad< -0.29f);
      if(elapsed>=1060)CHECK(o.motion.w==0 && o.follow_pid_rate_rad_s==0);
    }
    if(elapsed==1099) {held_pitch=cloud.runtime.output().target.pitch;CHECK(held_pitch>0.001f);}
    if(elapsed>=1100 && elapsed<stop_ms) {
      // 换档时关节不得被FD失能，留出分页传输时间后应自动恢复且保留云台目标。
      CHECK(base.enabled && cloud.enabled);
      if((elapsed>=1140 && elapsed<1210) || (elapsed>=1250 && elapsed<spin_end_ms) || elapsed>=spin_end_ms+40) {
        const auto &o=base.runtime.output();
        CHECK(o.control && cloud.runtime.output().control);
        CHECK(std::fabs(cloud.runtime.output().target.pitch-held_pitch)<0.00001f);
        if(elapsed<1210) {
          CHECK(o.state.chassis_mode==ChassisMode::Separate && o.chassis_heading_hold);
          if(elapsed==1140)held_chassis_heading=base.imu.heading;
          CHECK(std::fabs(o.chassis_heading_target_rad-held_chassis_heading)<0.00001f);
          if(elapsed>=1160 && elapsed<1180)CHECK(o.chassis_heading_error_rad<0 && o.motion.w<0);
          else if(elapsed>=1181 && elapsed<1200)CHECK(o.chassis_heading_error_rad>0 && o.motion.w>0);
          else CHECK(o.motion.w==0);
        }
        else if(elapsed<spin_end_ms) {
          // 45 rpm的自转设定应被 yaw 的 30 rpm 余量限制。
          const float spin_rate=30.0f*main_tuning::rpm;
          CHECK(o.state.chassis_mode==ChassisMode::Spin && cloud.runtime.output().motion.w==0);
          CHECK(o.motion.w>0 && o.motion.w<=spin_rate+0.00001f);
          if(elapsed>=1750)CHECK(std::fabs(o.motion.w-spin_rate)<0.00001f);
          if((elapsed>=1785 && elapsed<1805) || (elapsed>=1840 && elapsed<1865)) {
            CHECK(o.motion.vx>0);
            CHECK(std::fabs(std::hypot(o.motion.vx,o.motion.vy)-kControl.max_vx_m_s)<0.0005f);
            const float wheel_limit=spinWheelTargetLimit(o.motion.w);
            CHECK(wheel_limit<robot_debug::drive::kMaxActualSpeed);
            if(elapsed<1805)CHECK(o.motion.vy==0);
            else {
              CHECK(o.motion.vy<0);
              CHECK(std::fabs(o.motion.vx+o.motion.vy)<0.0011f);
            }
            for(const auto &wheel:chassis.targets().wheels)
              CHECK(std::fabs(wheel.speed_rad_s)<=wheel_limit+0.0001f);
            if(elapsed==1790 || elapsed==1850) {
              // 测试配置使用 -8 度。用独立的坐标旋转和轮位计算，验证平移右修正，自转分量不变。
              const float delta=elapsed==1790?0.21f:-0.40f;
              constexpr float right_offset=8.0f*0.017453292519943295f;
              const float c=std::cos(delta-right_offset),s=std::sin(delta-right_offset);
              const float tx=c*o.motion.vx-s*o.motion.vy,ty=s*o.motion.vx+c*o.motion.vy;
              Chassis::Solver aligned_solver(chassis_ik_config::kLimits);Chassis aligned(&aligned_solver);
              float angles[4],speeds[4];
              for(int i=0;i<4;++i) {
                const auto &pos=vehicle_config::kWheelPositions[i];
                const float vx=tx-o.motion.w*pos.y_m,vy=ty+o.motion.w*pos.x_m;
                angles[i]=std::atan2(vy,vx);speeds[i]=std::hypot(vx,vy)/vehicle_config::kWheelRadiusM;
              }
              CHECK(aligned.updateTargets(o.motion,delta,angles,o.state,true));
              for(int i=0;i<4;++i) {
                const auto &wheel=aligned.targets().wheels[i];
                CHECK(std::fabs(wheel.speed_rad_s-speeds[i])<0.0002f);
                CHECK(std::fabs(Chassis::Solver::wrapAngle(wheel.steer_rad-angles[i]))<0.00001f);
              }
              CHECK(aligned.targets().wheels[2].speed_rad_s>robot_debug::drive::kMaxTargetSpeed);
              // 分离和跟随仍按原传感器相对角平移，偏角仅由小陀螺路径使用。
              auto state=o.state;
              for(auto mode:{ChassisMode::Separate,ChassisMode::Follow}) {
                aligned.stop();state.chassis_mode=mode;
                for(float &angle:angles)angle=delta;
                CHECK(aligned.updateTargets({0.2f,0,0},delta,angles,state,true));
                for(const auto &wheel:aligned.targets().wheels)
                  CHECK(std::fabs(wheel.steer_rad-delta)<0.00001f);
              }
            }
          }
          // 通信按页提交整组遥控指令，回中取更新后的稳定读数。
          if(elapsed>=1900)CHECK(o.motion.vx==0 && o.motion.vy==0);
        } else CHECK(o.state.chassis_mode==ChassisMode::Follow);
      }
    }
    if(elapsed>=stop_ms+10)CHECK(!base.runtime.output().control && !cloud.runtime.output().control && base.runtime.output().motion.w==0);
    if(elapsed>=1100)continue;
#endif
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
#if defined(TEST_DUAL_FOLLOW)
  CHECK(corrected_stale_reference);
#endif
  return 0;
}
