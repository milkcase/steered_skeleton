#pragma once
#if ROBOT_DRIVE_SPEED_ENABLED
// 真实M3508/DT7/双板CAN验收路径；不连接实车，验证电流单位和唯一左前槽位。
int runDriveRuntime(RobotRuntime &robot,RobotCanService &service,DaMiao &axis,GimbalChassisComm &peer,
                    Motor *const wheels[4],Motor *const steers[4]) {
#if defined(TEST_CHASSIS)
  robot_debug::Power other(true,false);
#else
  robot_debug::Power other(true,true);
#endif
  uint32_t delivered=0;uint16_t encoder=0;bool saw_current=false;
  // 先注入被动反馈，再由实际Runtime控制并发帧；对端握手必须经真实协议收齐。
  auto advance=[&](uint8_t right,float stick) {
    robot.pollSensors();axisFeedback(axis,false);axis.setInput(1);
    setRemote(right,1024,1024,stick>0?1354:(stick<0?694:1024));
    if(robot_debug::drive::kVector)setRemote(right,1024,stick!=0?791:1024,stick!=0?1257:1024);
    robot_debug::Input in;in.now_ms=test_now;in.transport=in.imu=in.feedback=in.remote=true;
    in.right=right;in.steer=stick;in.neutral=stick==0;in.other_neutral=true;
    if(robot_debug::drive::kVector) {in.steer=stick!=0?0.25f:0;in.magnitude=std::fabs(stick);
      in.vector_wheel_target=in.magnitude*robot_debug::drive::kMaxTargetSpeed*0.70710678f;}
    robot_debug::Session session;
#if defined(TEST_CHASSIS)
    const uint16_t rpm=robot.output().control && test_now>650?20:0;
    if(rpm)encoder=(encoder+3)%8192;
    for(int i=0;i<4;++i) {
      uint8_t wheel[8]{};wheel[6]=30;
      if(i==0) {wheel[0]=encoder>>8;wheel[1]=encoder;wheel[2]=rpm>>8;wheel[3]=rpm;wheel[5]=123;}
      uint8_t steer[8]{};steer[6]=30;
      wheels[i]->decode(wheel);steers[i]->decode(steer);
      wheels[i]->setInput(1000);steers[i]->setInput(1000);
      MotorFeedbacks().recordDji(i,wheel,test_now);MotorFeedbacks().recordDji(i+4,steer,test_now);
    }
    GimbalChassisComm::ChassisData received;
    in.peer=peer.readPeer(received) && received.flags.imu_ready && received.flags.yaw_feedback_valid;session=received.session;
#else
    GimbalChassisComm::GimbalData received;
    in.peer=peer.readPeer(received) && received.flags.imu_ready && received.flags.pitch_feedback_valid;session=received.session;
    in.remote=in.peer && received.preview.valid;in.right=received.preview.right;in.left=received.preview.left;
    in.steer=received.preview.steer;in.neutral=received.preview.neutral;in.other_neutral=received.preview.other_neutral;
    if(robot_debug::drive::kVector) {in.magnitude=received.preview.magnitude;
      in.vector_wheel_target=in.magnitude*robot_debug::drive::kMaxTargetSpeed*std::cos(in.steer*3.141592653589793f);}
#endif
    // 对端替身也必须读取安全第一页撤销记录，才能与实车一致识别正常回中，不能省略此输入。
    const auto rx=peer.rxDiagnostics();in.link.rx_event=rx.last_rejection;
    in.link.crc_errors=rx.crc_errors;in.link.format_errors=rx.format_errors;in.link.order_errors=rx.order_errors;
    in.link.assembly_timeouts=rx.assembly_timeouts;in.link.replays=rx.replays;
    other.step(in,session);
#if defined(TEST_CHASSIS)
    auto &d=peer.gimbal_data();d={};d.preview={stick,right,1,stick==0,true,true};
    if(robot_debug::drive::kVector) {d.preview.steer=in.steer;d.preview.magnitude=in.magnitude;}
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
    if(test_now%3==0) {uint8_t bytes[8];peer.encode(bytes);local_comm->decode(bytes);}
    robot.update();service.tick();
    for(;delivered<frame_count;++delivered)if(frames[delivered].id==local_comm->txId())peer.decode(frames[delivered].data);
    CHECK(axis.getInput()==0 && !axis.is_enabled());
#if defined(TEST_CHASSIS)
    for(int i=0;i<4;++i)CHECK((robot_debug::drive::kVector || steers[i]->getInput()==0) && (i==0 || wheels[i]->getInput()==0));
    CHECK(std::fabs(wheels[0]->getInput())<=robot_debug::drive::kCurrentA*robot_debug::drive::kRawPerAmp+0.0001f);
    if(wheels[0]->getInput()!=0) {
      saw_current=true;CHECK(robot.output().control && robot.output().state.life==LifeState::Working);
      CHECK(std::fabs(wheels[0]->getInput()-robot_motor_debug_diagnostics.loops.current_command_a*robot_debug::drive::kRawPerAmp)<0.001f);
    }
#endif
    ++test_now;return 0;
  };
  for(int i=0;i<1200;++i)CHECK(advance(i<80?2:(i<430?3:1),i<500?0:0.5f)==0);
  CHECK(robot_motor_debug_diagnostics.drive_mode==1 && robot_motor_debug_diagnostics.position_loop_enabled==0);
#if defined(TEST_CHASSIS)
  CHECK(saw_current);
  if(robot_debug::drive::kTuning)CHECK(robot.output().control && wheels[0]->getInput()!=0);
  else {CHECK(!robot.output().control && wheels[0]->getInput()==0);
    CHECK(robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::TimeLimit);}
  if(!robot_debug::drive::kTuning) {
    CHECK(robot_motor_debug_diagnostics.recorded_peak_speed_rad_s>0.10f && robot_motor_debug_diagnostics.recorded_peak_speed_rad_s<0.12f);
    CHECK(std::fabs(robot_motor_debug_diagnostics.recorded_peak_feedback_current_a-123.0f*20/16384)<0.0001f);
  }
  if(robot_debug::drive::kVector) {
    for(int i=0;i<4;++i)CHECK(steers[i]->getInput()!=0 && robot_four_steer_diagnostics[i].active);
    CHECK(robot_vector_wheel_targets[0]>0 && robot_four_steer_diagnostics[0].position_target_rad>0);
  }
#endif
  // 检查线上真实帧：只有CAN2/0x200的第一槽可非零，GM电流帧全零，DM始终失能。
  for(uint32_t i=0;i<frame_count;++i) {
    const auto &f=frames[i];
    if(f.id==0x1FE) {
      if(!robot_debug::drive::kVector)for(uint8_t x:f.data)CHECK(x==0);
      else for(int k=0;k<4;++k) {
        const int bits=(f.data[k*2]<<8)|f.data[k*2+1],raw=bits>=32768?bits-65536:bits;
        CHECK(std::abs(raw)<=robot_debug::direction::kRawCurrentLimit);
        if(raw!=0)CHECK(f.life==LifeState::Working);
      }
    }
    if(f.id==0x200) {
      CHECK(f.can==&hcan2);for(int k=2;k<8;++k)CHECK(f.data[k]==0);
      const int bits=(f.data[0]<<8)|f.data[1],raw=bits>=32768?bits-65536:bits;
      CHECK(raw>=-robot_debug::drive::kRawLimit && raw<=robot_debug::drive::kRawLimit);
      if(raw!=0)CHECK(f.life==LifeState::Working);
    }
    if(f.id<16)CHECK(f.data[7]==0xFD);
  }
  // 不松杆不能续期；重新准备后模拟目标总线邮箱持续拥塞，20ms必须停，不靠500ms兜底。
  for(int i=0;i<50;++i)CHECK(advance(1,0.5f)==0);
#if defined(TEST_CHASSIS)
  CHECK(robot_motor_debug_diagnostics.run_count==1);
#endif
  if(robot_debug::drive::kTuning) {
    for(int i=0;i<35;++i)CHECK(advance(1,0)==0);
    CHECK(!robot.output().control);
#if defined(TEST_CHASSIS)
    CHECK(robot_motor_debug_diagnostics.recorded_peak_speed_rad_s>0.10f && robot_motor_debug_diagnostics.recorded_peak_speed_rad_s<0.12f);
    CHECK(std::fabs(robot_motor_debug_diagnostics.recorded_peak_feedback_current_a-123.0f*20/16384)<0.0001f);
#endif
    for(int i=0;i<60;++i)CHECK(advance(1,0.5f)==0);
#if defined(TEST_CHASSIS)
    CHECK(robot.output().control && robot_motor_debug_diagnostics.run_count==2);
#else
    CHECK(other.output().control);
#endif
  }
  for(int i=0;i<80;++i)CHECK(advance(2,0)==0);
  for(int i=0;i<350;++i)CHECK(advance(3,0)==0);
  for(int i=0;i<100;++i)CHECK(advance(1,0.5f)==0);
#if defined(TEST_CHASSIS)
  CHECK(robot.output().control);free_mailboxes=0;
  for(int i=0;i<23;++i)CHECK(advance(1,0.5f)==0);
  CHECK(wheels[0]->getInput()==0 && robot_motor_debug_diagnostics.last_stop==robot_debug::Reason::Transmit);
  free_mailboxes=3;
  if(robot_debug::drive::kVector) {
    // 右后转向新到超速反馈时，组合模式必须同时清四舵和左前驱动，不能只检查驱动速度。
    frame_count=delivered=0;
    for(int i=0;i<80;++i)CHECK(advance(2,0)==0);
    for(int i=0;i<350;++i)CHECK(advance(3,0)==0);
    for(int i=0;i<100;++i)CHECK(advance(1,0.5f)==0);
    CHECK(robot.output().control);
    uint8_t fast[8]{};fast[2]=3;fast[3]=232;fast[6]=30;
    steers[2]->decode(fast);MotorFeedbacks().recordDji(6,fast,test_now);service.tick();
    CHECK(!robot.output().control);
    for(int i=0;i<4;++i)CHECK(wheels[i]->getInput()==0 && steers[i]->getInput()==0);
  }
#endif
  return 0;
}
#endif
