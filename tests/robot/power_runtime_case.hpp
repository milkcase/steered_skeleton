#pragma once
#if ROBOT_MOTOR_DEBUG_POWER_ENABLED
// 将真实Runtime、提供的电机编码器、最终发送层与另一板的纯会话控制器连成端到端模拟。
int runPowerRuntime(RobotRuntime &robot,RobotCanService &service,DaMiao &axis,GimbalChassisComm &peer,
                    Motor *const wheels[4],Motor *const steers[4]) {
#if defined(TEST_CHASSIS)
  robot_debug::Power counterpart(true,false);
  OptionalParams reversed;reversed.dir=kDirRev;reversed.input_type=InputType::kCurr;
  static_cast<GM6020 *>(steers[0])->init(1,reversed);
#else
  robot_debug::Power counterpart(true,true);
#endif
  uint32_t delivered=0;bool saw_nonzero=false,extra_channel_active=false;
  // 真实本板1ms控制/发送，对端3ms一页；计数与快照均经实际CRC/分页代码推进。
  auto advance=[&](uint8_t right,float stick) {
    robot.pollSensors();axisFeedback(axis,false);axis.setInput(1);
    setRemote(right,extra_channel_active?1100:1024,stick==0?1024:694);
    robot_debug::Input other;other.now_ms=test_now;other.transport=other.imu=other.feedback=other.remote=true;
    other.right=right;other.steer=stick;other.neutral=stick==0;other.other_neutral=true;
    robot_debug::Session session;
#if defined(TEST_CHASSIS)
    uint8_t raw[8]{};raw[6]=30;
    for(int i=0;i<4;++i) {
      uint8_t steering_feedback[8];memcpy(steering_feedback,raw,8);
      // 左前为反向配置，原始-164经真实驱动解码为约+30mA；与50mA指令独立。
      if(i==0) {steering_feedback[4]=0xFF;steering_feedback[5]=0x5C;}
      wheels[i]->decode(raw);steers[i]->decode(steering_feedback);wheels[i]->setInput(1000);steers[i]->setInput(1000);
      MotorFeedbacks().recordDji(i,raw,test_now);MotorFeedbacks().recordDji(i+4,steering_feedback,test_now);
    }
    GimbalChassisComm::ChassisData received;
    other.peer=peer.readPeer(received) && received.flags.debug_power && received.flags.imu_ready && received.flags.yaw_feedback_valid;
    session=received.session;
#else
    GimbalChassisComm::GimbalData received;
    other.peer=peer.readPeer(received) && received.flags.debug_power && received.flags.imu_ready && received.flags.pitch_feedback_valid;
    session=received.session;other.remote=other.peer && received.preview.valid;
    other.right=received.preview.right;other.left=received.preview.left;other.steer=received.preview.steer;
    other.neutral=received.preview.neutral;other.other_neutral=received.preview.other_neutral;
#endif
    counterpart.step(other,session);
#if defined(TEST_CHASSIS)
    auto &d=peer.gimbal_data();d={};d.preview={stick,right,1,stick==0,true,true};
#else
    auto &d=peer.chassis_data();d={};
#endif
    const auto &o=counterpart.output();d.state=o.state;d.session=counterpart.session();
    d.flags.debug_preview=d.flags.debug_power=d.flags.imu_ready=true;
    d.flags.ready=o.ready;d.flags.command_valid=o.command_valid;d.flags.remote_valid=other.remote;
    d.flags.emergency_released=other.remote && other.right!=2;
#if defined(TEST_CHASSIS)
    d.flags.pitch_feedback_valid=true;
#else
    d.flags.yaw_feedback_valid=true;
#endif
    if(test_now%3==0) {uint8_t data[8];peer.encode(data);local_comm->decode(data);}
    robot.update();service.tick();
    for(;delivered<frame_count;++delivered) {
      const auto &f=frames[delivered];if(f.id==local_comm->txId())peer.decode(f.data);
    }
    ++test_now;
  };
  for(int step=0;step<1200;++step) {
    advance(step<70?2:(step<400?3:1),step<450?0:-0.5f);
    CHECK(axis.getInput()==0 && !axis.is_enabled());
#if defined(TEST_CHASSIS)
    for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && (i==0 || steers[i]->getInput()==0));
    CHECK(std::fabs(steers[0]->getInput())<=robot_debug::tuning::kCurrentLimitA);
    if(steers[0]->getInput()!=0) {
      saw_nonzero=true;CHECK(robot.output().state.life==LifeState::Working && robot.output().control);
      // 模拟的毫秒已在advance末尾推进；恰好到500ms时，看门狗复查应立即停止而非延长一轮。
      if(!robot.controlStillSafe()) {
        CHECK(robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::TimeLimit);
        CHECK(!robot.output().control && steers[0]->getInput()==0);
      }
      CHECK(robot_motor_debug_diagnostics.power_enabled==1 && robot_motor_debug_diagnostics.zero_output_only==0);
      CHECK(robot_motor_debug_diagnostics.life_state==static_cast<uint32_t>(robot.output().state.life));
    }
#endif
  }
#if defined(TEST_CHASSIS)
  CHECK(saw_nonzero && robot_motor_debug_diagnostics.run_count==1);
  CHECK(robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::TimeLimit);
  CHECK(robot_motor_debug_diagnostics.recorded_peak_output>0 && robot_motor_debug_diagnostics.recorded_peak_output<=robot_debug::tuning::kCurrentLimitA);
  if(robot_debug::tuning::kPositionKp>=5.0f && robot_debug::tuning::kCurrentLimitA<=0.100f)
    CHECK(std::fabs(robot_motor_debug_diagnostics.recorded_peak_output-robot_debug::tuning::kCurrentLimitA)<0.000001f);
  CHECK(robot_motor_debug_diagnostics.configured_kp==robot_debug::tuning::kPositionKp);
  CHECK(robot_motor_debug_diagnostics.configured_current_limit_a==robot_debug::tuning::kCurrentLimitA);
  CHECK(robot_motor_debug_diagnostics.configured_profile==robot_debug::tuning::kProfile);
  CHECK(std::fabs(robot_motor_debug_diagnostics.recorded_peak_feedback_current_a-164.0f*3.0f/16384.0f)<0.000001f);
  CHECK(robot_motor_debug_diagnostics.recorded_feedback_samples>400);
#endif
  for(uint32_t i=0;i<frame_count;++i) {
    const auto &f=frames[i];
    CHECK(f.id!=GM6020::GetTxId(1,InputType::kRaw)); // 四台均为电流模式，不能继续夹带0x1FF电压零帧。
    if(f.id<16) {CHECK(f.can==&hcan2 && f.data[7]==0xFD);}
    if(f.id==M3508::GetTxId(1))for(uint8_t byte:f.data)CHECK(byte==0);
    if(f.id==GM6020::GetTxId(1,InputType::kCurr)) {
      CHECK(f.can==&hcan1);for(int slot=2;slot<8;++slot)CHECK(f.data[slot]==0);
      const int raw=(f.data[0]<<8)|f.data[1];const int signed_raw=raw>=32768?raw-65536:raw;
      CHECK(signed_raw>=-robot_debug::tuning::kRawCurrentLimit && signed_raw<=robot_debug::tuning::kRawCurrentLimit);
      if(signed_raw!=0)CHECK(f.life==LifeState::Working && signed_raw<0);
    }
  }
  // 重新准备后，破坏全局显示值不能改变私有命令；急停在最终发送前到达必须撤销并发零帧。
  for(int i=0;i<80;++i)advance(2,0);
  for(int i=0;i<350;++i)advance(3,0);
  for(int i=0;i<40;++i)advance(1,0);
  for(int i=0;i<100;++i)advance(1,-0.5f);
#if defined(TEST_CHASSIS)
  CHECK(robot.output().control);
  robot_motor_debug_diagnostics.final_input=25000;robot_motor_debug_diagnostics.session_token=0;
  steers[0]->setInput(1.0f);steers[1]->setInput(1.0f);service.tick();
  CHECK(std::fabs(steers[0]->getInput())<=robot_debug::tuning::kCurrentLimitA && steers[1]->getInput()==0);
  auto &urgent=peer.gimbal_data();urgent.preview.right=2;urgent.preview.steer=0;urgent.preview.neutral=true;
  urgent.flags.emergency_released=false;urgent.flags.command_valid=false;urgent.state={};urgent.session.phase=0;
  // 只推进到新急停快照的第一页，刻意不补齐后两页，验证撤销不等待完整提交。
  for(int i=0;i<3;++i) {
    uint8_t bytes[8];peer.encode(bytes);local_comm->decode(bytes);
    if((bytes[0]&15)==0)break;
  }
  const auto before=frame_count;service.tick();CHECK(!robot.output().control && steers[0]->getInput()==0);
  bool zero=false;
  for(uint32_t i=before;i<frame_count;++i)if(frames[i].id==GM6020::GetTxId(1,InputType::kCurr)) {
    for(uint8_t byte:frames[i].data)CHECK(byte==0);zero=true;
  }
  CHECK(zero);
  // 急停的第一页在两次控制之间到达：既要立刻发零，也必须锁存快照撤销而不是事后恢复值。
  const auto &stopped=robot_motor_debug_diagnostics.stop_check;
  CHECK(stopped.count>=1 && stopped.source==2 && (stopped.mask&robot_debug::CheckPeer)!=0);
  CHECK(stopped.snapshot_valid==0 && stopped.peer==0 && stopped.run_count==2);
  CHECK(stopped.rx_event.reason==GimbalChassisComm::RxFailure::SafetyPage && stopped.rx_event.page==0);
  CHECK((stopped.rx_event.flags_byte&32)==0);
  const uint32_t stop_time=stopped.time_ms;
  robot.stop();
  CHECK(robot_motor_debug_diagnostics.stop_check.time_ms==stop_time);
  // 邮箱持续忙时最多20ms即停止，不能只因旧遥控/会话仍有效便持续保留动力许可。
  for(int i=0;i<80;++i)advance(2,0);
  CHECK(robot_motor_debug_diagnostics.stop_check.time_ms==stop_time);
  for(int i=0;i<350;++i)advance(3,0);
  for(int i=0;i<40;++i)advance(1,0);
  for(int i=0;i<100;++i)advance(1,-0.5f);
  CHECK(robot.output().control);free_mailboxes=0;
  for(int i=0;i<25;++i)advance(1,-0.5f);
  CHECK(!robot.output().control && steers[0]->getInput()==0);
  CHECK(robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::Transmit);free_mailboxes=3;
  // 控制入口停止推进超过3ms时，发送层独立撤销本轮；不等待下一次控制回调补做停止。
  for(int i=0;i<80;++i)advance(2,0);
  for(int i=0;i<350;++i)advance(3,0);
  for(int i=0;i<40;++i)advance(1,0);
  for(int i=0;i<100;++i)advance(1,-0.5f);
  CHECK(robot.output().control);test_now+=4;service.tick();
  CHECK(!robot.output().control && steers[0]->getInput()==0);
  CHECK(robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::Clock);
  const auto &stale=robot_motor_debug_diagnostics.stop_check;
  // advance末尾已推进1ms，再跳4ms，故相对最后控制采样的真实年龄为5ms。
  CHECK(stale.source==2 && (stale.mask&robot_debug::CheckControlAge)!=0 && stale.control_age_ms==5);
  CHECK(stale.count>=3 && stale.run_count==4);
  const uint32_t stale_time=stale.time_ms;
  robot.stop();for(int i=0;i<10;++i)advance(2,0);
  CHECK(robot_motor_debug_diagnostics.stop_check.time_ms==stale_time);
#else
  CHECK(axis.getInput()==0 && !robot.output().control);
  // 云台真实DT7某个非目标通道短暂偏离死区：授权立即撤销，原始通道和首次原因要留到右下之后。
  for(int i=0;i<80;++i)advance(2,0);
  for(int i=0;i<350;++i)advance(3,0);
  for(int i=0;i<40;++i)advance(1,0);
  for(int i=0;i<50;++i)advance(1,-0.5f);
  CHECK(local_comm->gimbal_data().session.phase==2);
  const uint32_t previous_veto_count=robot_motor_debug_diagnostics.stop_check.count;
  extra_channel_active=true;advance(1,-0.5f);
  const auto &veto=robot_motor_debug_diagnostics.stop_check;
  CHECK(veto.source==4 && veto.mask==robot_debug::CheckOtherChannel);
  CHECK(veto.count==previous_veto_count+1);
  CHECK(veto.channels_valid==1 && veto.other_neutral==0 && veto.local_phase==2);
  bool other_changed=false;
  for(int i=0;i<5;++i)if(i!=1 && std::fabs(veto.channels[i])>0.05f)other_changed=true;
  CHECK(other_changed && axis.getInput()==0 && local_comm->gimbal_data().session.phase==0);
  const uint32_t veto_time=veto.time_ms;
  extra_channel_active=false;for(int i=0;i<20;++i)advance(2,0);
  CHECK(robot_motor_debug_diagnostics.stop_check.time_ms==veto_time && robot_motor_debug_diagnostics.stop_check.mask==robot_debug::CheckOtherChannel);
#endif
  return 0;
}
#endif
