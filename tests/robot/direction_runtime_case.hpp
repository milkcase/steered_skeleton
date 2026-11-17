#pragma once
#if ROBOT_STEER_DIRECTION_ENABLED
// 真实Runtime/DT7/电机驱动/CAN发送链验证：仅左前双环，取消500ms后仍遵守全部异步联锁。
int runDirectionRuntime(RobotRuntime &robot,RobotCanService &service,DaMiao &axis,GimbalChassisComm &peer,
                        Motor *const wheels[4],Motor *const steers[4]) {
#if defined(TEST_CHASSIS)
  robot_debug::Power other(true,false);
  OptionalParams reversed;reversed.dir=kDirRev;reversed.input_type=InputType::kCurr;
  static_cast<GM6020 *>(steers[0])->init(1,reversed);
  if(robot_debug::direction::kFour)for(int i=1;i<4;++i)static_cast<GM6020 *>(steers[i])->init(i+1,reversed);
#else
  robot_debug::Power other(true,true);
#endif
  uint32_t delivered=0;bool saw_current=false;bool radio_ok=true;
  // 一周期内先收反馈、再控制、最后发送；对端必须实际编解码，不能直接写会话许可。
  auto advance=[&](uint8_t right,bool active) {
    robot.pollSensors();axisFeedback(axis,false);axis.setInput(1);
    remote_valid=radio_ok;setRemote(right,1024,active?980:1024,active?1600:1024);
    robot_debug::Input in;in.now_ms=test_now;in.transport=in.imu=in.feedback=true;in.remote=radio_ok;
    in.right=right;in.steer=active?0.02f:0;in.neutral=!active;in.other_neutral=true;
    robot_debug::Session session;
#if defined(TEST_CHASSIS)
    uint8_t zero[8]{};zero[6]=30;
    for(int i=0;i<4;++i) {
      wheels[i]->decode(zero);
      uint8_t feedback[8];memcpy(feedback,zero,8);if(robot_debug::direction::kFour)feedback[1]=i*20;
      steers[i]->decode(feedback);wheels[i]->setInput(100);steers[i]->setInput(100);
      MotorFeedbacks().recordDji(i,zero,test_now);MotorFeedbacks().recordDji(i+4,zero,test_now);
    }
    GimbalChassisComm::ChassisData received;
    in.peer=peer.readPeer(received) && received.flags.imu_ready && received.flags.yaw_feedback_valid;
    session=received.session;
#else
    GimbalChassisComm::GimbalData received;
    in.peer=peer.readPeer(received) && received.flags.imu_ready && received.flags.pitch_feedback_valid;
    session=received.session;in.remote=in.peer && received.preview.valid;
    in.right=received.preview.right;in.left=received.preview.left;in.steer=received.preview.steer;
    in.neutral=received.preview.neutral;in.other_neutral=received.preview.other_neutral;
#endif
    // 对端替身也读取真实接收撤销记录，模拟实车Runtime，不能丢掉正常回中提示。
    const auto rx=peer.rxDiagnostics();in.link.rx_event=rx.last_rejection;
    in.link.crc_errors=rx.crc_errors;in.link.format_errors=rx.format_errors;in.link.order_errors=rx.order_errors;
    in.link.assembly_timeouts=rx.assembly_timeouts;in.link.replays=rx.replays;
    other.step(in,session);
#if defined(TEST_CHASSIS)
    auto &d=peer.gimbal_data();d={};d.preview={in.steer,right,1,!active,true,radio_ok};
#else
    auto &d=peer.chassis_data();d={};
#endif
    d.state=other.output().state;d.session=other.session();
    d.flags.debug_preview=d.flags.debug_power=d.flags.imu_ready=true;
    d.flags.ready=other.output().ready;d.flags.command_valid=other.output().command_valid;
    d.flags.remote_valid=in.remote;d.flags.emergency_released=in.remote && in.right!=2;
#if defined(TEST_CHASSIS)
    d.flags.pitch_feedback_valid=true;
#else
    d.flags.yaw_feedback_valid=true;
#endif
    if(test_now%3==0) {uint8_t f[8];peer.encode(f);local_comm->decode(f);}
    robot.update();service.tick();
    for(;delivered<frame_count;++delivered)if(frames[delivered].id==local_comm->txId())peer.decode(frames[delivered].data);
    CHECK(axis.getInput()==0 && !axis.is_enabled());
#if defined(TEST_CHASSIS)
    for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && (robot_debug::direction::kFour || i==0 || steers[i]->getInput()==0));
    if(steers[0]->getInput()!=0) {saw_current=true;CHECK(robot.output().control && robot.output().state.life==LifeState::Working);}
    CHECK(std::fabs(steers[0]->getInput())<=robot_debug::direction::kCurrentA);
#endif
    ++test_now;return 0;
  };
  for(int i=0;i<1850;++i)CHECK(advance(i<80?2:(i<430?3:1),i>=500)==0);
  CHECK(robot_motor_debug_diagnostics.direction_mode==1 && robot_motor_debug_diagnostics.time_limit_enabled==0);
#if defined(TEST_CHASSIS)
  CHECK(saw_current && robot.output().control && robot_motor_debug_diagnostics.run_count==1);
  CHECK(robot_motor_debug_diagnostics.loops.active && robot_motor_debug_diagnostics.loops.speed_target_rad_s>0);
  CHECK(robot_motor_debug_diagnostics.loops.position_target_rad>0 && robot_motor_debug_diagnostics.loops.position_feedback_rad==0);
  if(robot_debug::direction::kFour) {
    for(int i=0;i<4;++i)CHECK(robot_four_steer_diagnostics[i].active && steers[i]->getInput()!=0);
    CHECK(robot_four_steer_diagnostics[0].current_command_a!=robot_four_steer_diagnostics[3].current_command_a);
  }
#else
  CHECK(other.output().control && robot_motor_debug_diagnostics.session_ready);
#endif
  // 核对实际报文字节：GM仅ID1槽；C620全零，DM失能，电流单位和硬上限正确。
  for(uint32_t i=0;i<frame_count;++i) {
    const auto &f=frames[i];
    if(f.id==0x200)for(uint8_t x:f.data)CHECK(x==0);
    if(f.id==0x1FE) {
      if(!robot_debug::direction::kFour)for(int k=2;k<8;++k)CHECK(f.data[k]==0);
      else for(int slot=0;slot<4;++slot) {
        const int bits=(f.data[2*slot]<<8)|f.data[2*slot+1],raw=bits>=32768?bits-65536:bits;
        CHECK(std::abs(raw)<=robot_debug::mode::kRawLimit);
        if(raw!=0)CHECK(f.life==LifeState::Working && raw<0);
      }
      const int bits=(f.data[0]<<8)|f.data[1];const int raw=bits>=32768?bits-65536:bits;
      CHECK(std::abs(raw)<=robot_debug::direction::kRawCurrentLimit);
      if(raw!=0)CHECK(f.life==LifeState::Working && raw<0);
    }
    if(f.id<16)CHECK(f.data[7]==0xFD);
  }
  // 正常回中停机，再偏杆可直接继续；只适用于正常回中，后续真正超速仍锁止。
  for(int i=0;i<30;++i)CHECK(advance(1,false)==0);
  CHECK(!robot.output().control);
#if defined(TEST_CHASSIS)
  CHECK(steers[0]->getInput()==0 && robot_motor_debug_diagnostics.recorded_loops.active==1);
#endif
  for(int i=0;i<30;++i)CHECK(advance(1,true)==0);
#if defined(TEST_CHASSIS)
  CHECK(robot.output().control && robot_motor_debug_diagnostics.run_count==2);
#else
  CHECK(other.output().control);
#endif
#if defined(TEST_CHASSIS)
  if(robot_debug::direction::kFour) {
    // 公共update入口跨毫秒收到新反馈后仍应有效；不访问私有方法或改动实际动作许可。
    can_tick_injection=4;robot.update();
    CHECK(can_tick_injection==0 && robot.output().control && robot_motor_debug_diagnostics.feedback==1);
  }
  // 最终发送前新到达超速反馈时，旧版错误地记18；现在必须保留MotionLimit和现场速度。
  frame_count=delivered=0;
  for(int i=0;i<80;++i)CHECK(advance(2,false)==0);
  for(int i=0;i<350;++i)CHECK(advance(3,false)==0);
  for(int i=0;i<100;++i)CHECK(advance(1,true)==0);
  CHECK(robot.output().control);
  const auto rpm=static_cast<uint16_t>((robot_debug::steer_safety::kActualSpeedLimit+0.2f)*60/(2*robot_debug::direction::kPi))+1;
  uint8_t fast[8]{};fast[2]=rpm>>8;fast[3]=rpm;fast[6]=30;
  const int fault_index=robot_debug::direction::kFour?2:0;
  const uint32_t previous_fault_count=robot_four_steer_fault.count;
  steers[fault_index]->decode(fast);MotorFeedbacks().recordDji(4+fault_index,fast,test_now);service.tick();
  CHECK(!robot.output().control && steers[0]->getInput()==0);
  CHECK(robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::MotionLimit);
  CHECK(robot_motor_debug_diagnostics.stop_check.source==2 && robot_motor_debug_diagnostics.stop_check.mask==robot_debug::CheckMotion);
  if(!robot_debug::direction::kFour)CHECK(std::fabs(robot_motor_debug_diagnostics.stop_check.actual_velocity)>robot_motor_debug_diagnostics.stop_check.speed_limit);
  else {
    for(int i=0;i<4;++i)CHECK(steers[i]->getInput()==0 && wheels[i]->getInput()==0);
    CHECK(robot_four_steer_fault.count==previous_fault_count+1 && robot_four_steer_fault.wheel_id==3);
    CHECK(robot_four_steer_fault.source==2 && robot_four_steer_fault.failure_mask==robot_debug::CheckMotion);
    CHECK(robot_four_steer_fault.reason==robot_debug::Reason::MotionLimit);
    CHECK(std::fabs(robot_four_steer_fault.velocity_rad_s)>robot_debug::steer_safety::kActualSpeedLimit);
    const uint32_t failed_time=robot_four_steer_fault.time_ms;
    for(int i=0;i<30;++i)CHECK(advance(2,false)==0);
    CHECK(robot_four_steer_fault.count==previous_fault_count+1 && robot_four_steer_fault.time_ms==failed_time);
  }
#endif
  if(robot_debug::direction::kFour) {
    // 真实Runtime仍在5ms采样断档时撤销IMU就绪，读数恢复前不能放行；镜像须保留掉零年龄。
    test_now+=5;robot.update();
    CHECK(robot_motor_debug_diagnostics.imu==0 && robot_runtime_health.imu_drops>0);
    CHECK(robot_runtime_health.last_imu_drop_cache_ready==1 && robot_runtime_health.last_imu_drop_age_ms>=5);
    CHECK(!robot.output().control);
    robot.pollSensors();robot.update();
    CHECK(robot_motor_debug_diagnostics.imu==1 && robot_runtime_health.imu_age_ms==0);
  }
  return 0;
}
#endif
