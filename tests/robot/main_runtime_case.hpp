#pragma once
// 使用生产参数和真实Runtime：验证全轮速度环、回中主动制动、pitch进Working水平及松杆保持。
int runMainRuntime(robot::RobotRuntime &robot,robot::RobotCanService &service,hello_world::motor::DaMiao &axis,
    robot::GimbalChassisComm &peer,hello_world::motor::Motor *const wheels[4],hello_world::motor::Motor *const steers[4],
    robot::DrivePort *drive,robot::Gimbal &axis_loop) {
  CHECK(robot_control::Supervisor(robot_control::Board::Gimbal,robot_control::kControl).configured());
  CHECK(axis_loop.loopsReady());
  // yaw与pitch各取本轴AIM_RPM，不能把共用的rpm/s加速度当作满杆速度。
  CHECK(std::fabs(robot_control::kControl.yaw_rate_rad_s-ROBOT_YAW_AIM_RPM*main_tuning::rpm)<0.00001f);
  CHECK(std::fabs(robot_control::kControl.pitch_rate_rad_s-ROBOT_PITCH_AIM_RPM*main_tuning::rpm)<0.00001f);
  main_tuning::Source base;
  auto fresh=main_tuning::parameters();
  // 云台测试接收不同于本地编译值的底盘参数，验证只烧底盘能改变pitch及遥控映射。
  fresh.value[main_tuning::PositionP]=6.25f;fresh.value[main_tuning::SpeedP]=0.3f;
  fresh.value[main_tuning::Vx]=fresh.value[main_tuning::Vy]=0.4f;
  fresh.value[main_tuning::PitchLimitEnabled]=0;
  fresh.value[main_tuning::GravityEnabled]=1;fresh.value[main_tuning::GravityCos]=0.1f;
  fresh.value[main_tuning::GravitySin]=-0.02f;fresh.value[main_tuning::GravityMin]=-1;fresh.value[main_tuning::GravityMax]=1;
  base.load(fresh);
  auto step=[&](bool centered) {
    ++test_now;robot.pollSensors();axisFeedback(axis,test_now>=205);
#if !defined(TEST_CHASSIS)
    if(test_now%5==0) {uint8_t data[8];CHECK(base.frame(data));CHECK(main_tuning::parameter_sink.receive(data,test_now));base.sent();}
#endif
#if defined(TEST_CHASSIS)
  CHECK(std::fabs(robot_main_diagnostics.feedforward_nm-ROBOT_YAW_VELOCITY_FF*robot_main_diagnostics.axis_speed_target_rad_s)<0.00001f);
    for(int i=0;i<4;++i) {uint8_t f[8]{};f[3]=100;f[6]=30;wheels[i]->decode(f);MotorFeedbacks().recordDji(i,f,test_now);
      f[3]=0;steers[i]->decode(f);MotorFeedbacks().recordDji(i+4,f,test_now);}
#endif
    // 按实车3ms一页、每1ms读取一次；不能把三页连续塞完而漏测页间peer归零。
    GimbalChassisComm::Flags flags{true,true,true,true,true,true,true,false};
    ModuleState peer_state{test_now>=210?LifeState::Working:LifeState::Resurrection,
      ControlSource::RemoteControl,ChassisMode::Separate,GimbalMode::JointPosition};
#if defined(TEST_CHASSIS)
    auto &peer_data=peer.gimbal_data();peer_data.flags=flags;peer_data.state=peer_state;
    peer_data.yaw_target=0.2f;peer_data.motion=centered?ChassisMotion{}:ChassisMotion{0.2f,0,0};
#else
    auto &peer_data=peer.chassis_data();peer_data.flags=flags;peer_data.state=peer_state;
    peer_data.feedback.yaw=0.2f;peer_data.chassis_yaw=0.1f;
#endif
    if(test_now%3==0) {uint8_t frame[8];peer.encode(frame);local_comm->decode(frame);}
    robot.update();service.tick();
    if(test_now>=20)CHECK(robot_main_diagnostics.peer); // 每1ms检查页间状态，不只检查第三页之后。
    return 0;
  };
  test_now=0;setRemote(2);
  for(int i=0;i<500;++i) {if(test_now==200)setRemote(3);if(test_now==450)setRemote(1);CHECK(step(false)==0);}
  CHECK(robot_tuning_diagnostics.ready);
#if defined(TEST_CHASSIS)
  CHECK(robot_tuning_diagnostics.pitch_position_limits_enabled==ROBOT_PITCH_POSITION_LIMIT_ENABLED);
#else
  CHECK(robot_tuning_diagnostics.pitch_position_limits_enabled==0);  // 模拟底盘下发与本地默认不同的开关。
#endif
  CHECK(robot_main_diagnostics.imu && robot_main_diagnostics.peer);
  if(drive!=nullptr)CHECK(drive->ready(test_now));
  CHECK(robot.output().ready);
  CHECK(robot.output().control && robot.output().state.life==robot::LifeState::Working);
#if defined(TEST_CHASSIS)
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()>0);
#if defined(TEST_MAIN_PEER_REJECTION)
  // 收包先CRC拒收：第一时间锁存撤销事件，后续回右下再收到安全页也不能覆盖首因。
  uint8_t bad[8];peer.encode(bad);bad[7]^=1;local_comm->decode(bad);robot.update();service.tick();
  CHECK(!robot.output().control && axis.getInput()==0);
  CHECK(robot_main_stop.count==1 && robot_main_stop.source==static_cast<uint32_t>(MainStopSource::SupervisorGate));
  CHECK(robot_main_stop.peer_valid==0 && robot_main_stop.axis_valid==1 && robot_main_stop.axis_enabled==1);
  CHECK(robot_main_stop.rx_event.reason==GimbalChassisComm::RxFailure::Crc && robot_main_stop.rx_completed>0);
  peer.restartTx();peerFrame(peer,*local_comm,LifeState::Dead,false);robot.update();service.tick();
  ++test_now;peer.gimbal_data().flags.fault=true;peer.encode(bad);local_comm->decode(bad);
  CHECK(local_comm->rxDiagnostics().last_rejection.reason==GimbalChassisComm::RxFailure::SafetyPage);
  CHECK(robot_main_stop.rx_event.reason==GimbalChassisComm::RxFailure::Crc && robot_main_stop.count==1);
  return 0;
#endif
#if defined(TEST_MAIN_AXIS_FEEDBACK_LOSS)
  // 电机先返回禁用状态：必须在软件发送FD前锁存为反馈丢失，而非误记为软件先停机。
  axisFeedback(axis,false);robot.update();service.tick();
  CHECK(!robot.output().control && axis.getInput()==0);
  CHECK(robot_main_stop.count==1 && robot_main_stop.source==static_cast<uint32_t>(MainStopSource::FeedbackLost));
  CHECK(robot_main_stop.axis_enabled==0 && robot_main_stop.axis_valid==0 && robot_main_stop.prepared==1);
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && steers[i]->getInput()==0);
  peerFrame(peer,*local_comm,LifeState::Dead,false);robot.update();service.tick();CHECK(robot_main_stop.count==1);
  return 0;
#endif
  // 当前快照可能在回中前已冻结，等其余页和下一整组提交后检查制动；不改变真实通信截止。
  for(int i=0;i<18;++i)CHECK(step(true)==0);CHECK(robot.output().control);
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()<0); // 正转轮回中后主动反向制动。
  // 普通运动快照、遥控和IMU仍新鲜，仅参数确认失联，发送层也必须停止所有轴/轮。
  simulated_main_ack=false;for(int i=0;i<220;++i)CHECK(step(true)==0);
  CHECK(!robot_tuning_diagnostics.ready && !robot.output().control && axis.getInput()==0);
  CHECK(robot_main_stop.count==1 && robot_main_stop.source==static_cast<uint32_t>(MainStopSource::SupervisorGate));
  CHECK(robot_main_stop.parameters==0 && robot_main_stop.axis_enabled==1); // 软件先失能，不能误记为电机先掉反馈。
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && steers[i]->getInput()==0);
#else
  CHECK(robot_tuning_diagnostics.pitch_gravity_ff_enabled==1 && robot_tuning_diagnostics.gravity_cos_nm==0.1f);
  CHECK(robot_main_diagnostics.feedforward_nm>0.09f && robot_main_diagnostics.axis_feedback_torque_valid==1);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-6.25f)<0.0001f);
  CHECK(std::fabs(robot_tuning_diagnostics.max_vx_m_s-0.4f)<0.0001f);
  CHECK(std::fabs(robot.output().target.pitch)<0.00001f);
  // 检查右杆前后的目标角度变化与回中保持，实际机械方向由实车确认。
  setRemote(1,1156);for(int i=0;i<80;++i)CHECK(step(false)==0);
  const float target_after_forward=robot.output().target.pitch;
  CHECK(target_after_forward>0);
  setRemote(1);CHECK(step(true)==0);
  setRemote(1,892);for(int i=0;i<80;++i)CHECK(step(false)==0);
  CHECK(robot.output().target.pitch<target_after_forward);
  setRemote(1);const float held=robot.output().target.pitch;
  for(int i=0;i<30;++i)CHECK(step(true)==0);
  CHECK(robot.output().control && std::fabs(robot.output().target.pitch-held)<0.00001f);
  // Working收到另一组只缓存且撤销许可，绝不热换参数或自动重新运行。
  fresh.value[main_tuning::PositionP]=7.375f;base.load(fresh);
  // 27字段连同开始/提交帧以5ms间隔发送，共需145ms；等完整组到齐后再验证安全应用。
  for(int i=0;i<160;++i)CHECK(step(true)==0);
  CHECK(!robot.output().control && axis.getInput()==0);
  CHECK(robot_main_stop.count==1 && robot_main_stop.source==static_cast<uint32_t>(MainStopSource::SupervisorGate));
  CHECK(robot_main_stop.parameters==0 && robot_main_stop.axis_enabled==1);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-6.25f)<0.0001f);
  setRemote(2);remote_valid=false;CHECK(step(true)==0);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-6.25f)<0.0001f);
  remote_valid=true;CHECK(step(true)==0);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-7.375f)<0.0001f && robot_tuning_diagnostics.applied_count==2);
  CHECK(!robot.output().control && axis.getInput()==0);
  CHECK(robot_main_stop.count==1); // 右下和后续禁用反馈保留首个现场。
#endif
  setRemote(2);peerFrame(peer,*local_comm,robot::LifeState::Dead,false);robot.update();service.tick();
  CHECK(!robot.output().control && axis.getInput()==0);
#if defined(TEST_CHASSIS)
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && steers[i]->getInput()==0);
#endif
  return 0;
}
