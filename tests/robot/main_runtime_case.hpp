#pragma once
// 使用生产参数和真实Runtime：验证全轮速度环、回中主动制动、pitch进Working水平及松杆保持。
int runMainRuntime(robot::RobotRuntime &robot,robot::RobotCanService &service,hello_world::motor::DaMiao &axis,
    robot::GimbalChassisComm &peer,hello_world::motor::Motor *const wheels[4],hello_world::motor::Motor *const steers[4],
    robot::DrivePort *drive,robot::Gimbal &axis_loop) {
  CHECK(robot_control::Supervisor(robot_control::Board::Gimbal,robot_control::kControl).configured());
  CHECK(axis_loop.loopsReady());
  main_tuning::Source base;
  auto fresh=main_tuning::parameters();
  // 云台测试接收不同于本地编译值的底盘参数，验证只烧底盘能改变pitch及遥控映射。
  fresh.value[main_tuning::PositionP]=6.25f;fresh.value[main_tuning::SpeedP]=0.3f;
  fresh.value[main_tuning::Vx]=fresh.value[main_tuning::Vy]=0.4f;
  base.load(fresh);
  auto step=[&](bool centered) {
    ++test_now;robot.pollSensors();axisFeedback(axis,test_now>=205);
#if !defined(TEST_CHASSIS)
    if(test_now%5==0) {uint8_t data[8];CHECK(base.frame(data));CHECK(main_tuning::parameter_sink.receive(data,test_now));base.sent();}
#endif
#if defined(TEST_CHASSIS)
    for(int i=0;i<4;++i) {uint8_t f[8]{};f[3]=100;f[6]=30;wheels[i]->decode(f);MotorFeedbacks().recordDji(i,f,test_now);
      f[3]=0;steers[i]->decode(f);MotorFeedbacks().recordDji(i+4,f,test_now);}
#endif
    peerFrame(peer,*local_comm,test_now>=210?robot::LifeState::Working:robot::LifeState::Resurrection);
#if defined(TEST_CHASSIS)
    if(centered) {peer.gimbal_data().motion={};for(int k=0;k<3;++k) {uint8_t f[8];peer.encode(f);local_comm->decode(f);}}
#endif
    robot.update();service.tick();
    return 0;
  };
  test_now=0;setRemote(2);
  for(int i=0;i<400;++i) {if(test_now==130)setRemote(3);if(test_now==365)setRemote(1);CHECK(step(false)==0);}
  CHECK(robot_tuning_diagnostics.ready);
  CHECK(robot_main_diagnostics.imu && robot_main_diagnostics.peer);
  if(drive!=nullptr)CHECK(drive->ready(test_now));
  CHECK(robot.output().ready);
  CHECK(robot.output().control && robot.output().state.life==robot::LifeState::Working);
#if defined(TEST_CHASSIS)
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()>0);
  CHECK(step(true)==0);CHECK(robot.output().control);
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()<0); // 正转轮回中后主动反向制动。
  // 普通运动快照、遥控和IMU仍新鲜，仅参数确认失联，发送层也必须停止所有轴/轮。
  simulated_main_ack=false;for(int i=0;i<220;++i)CHECK(step(true)==0);
  CHECK(!robot_tuning_diagnostics.ready && !robot.output().control && axis.getInput()==0);
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && steers[i]->getInput()==0);
#else
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-6.25f)<0.0001f);
  CHECK(std::fabs(robot_tuning_diagnostics.max_vx_m_s-0.4f)<0.0001f);
  CHECK(std::fabs(robot.output().target.pitch)<0.00001f);
  setRemote(1,1156);for(int i=0;i<80;++i)CHECK(step(false)==0);
  CHECK(robot.output().target.pitch<0);
  setRemote(1);const float held=robot.output().target.pitch;
  for(int i=0;i<30;++i)CHECK(step(true)==0);
  CHECK(robot.output().control && std::fabs(robot.output().target.pitch-held)<0.00001f);
  // Working收到另一组只缓存且撤销许可，绝不热换参数或自动重新运行。
  fresh.value[main_tuning::PositionP]=7.375f;base.load(fresh);
  for(int i=0;i<120;++i)CHECK(step(true)==0);
  CHECK(!robot.output().control && axis.getInput()==0);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-6.25f)<0.0001f);
  setRemote(2);remote_valid=false;CHECK(step(true)==0);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-6.25f)<0.0001f);
  remote_valid=true;CHECK(step(true)==0);
  CHECK(std::fabs(robot_tuning_diagnostics.pitch_position_p-7.375f)<0.0001f && robot_tuning_diagnostics.applied_count==2);
  CHECK(!robot.output().control && axis.getInput()==0);
#endif
  setRemote(2);peerFrame(peer,*local_comm,robot::LifeState::Dead,false);robot.update();service.tick();
  CHECK(!robot.output().control && axis.getInput()==0);
#if defined(TEST_CHASSIS)
  for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && steers[i]->getInput()==0);
#endif
  return 0;
}
