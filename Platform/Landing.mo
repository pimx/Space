within Platform;

  // ========================================================================
  // Simple Test Model
  // ========================================================================

  model Landing

    Rocket  rocket(startAltitude=100);
    Control control;
    Engine  engine;
    Imu     imu;
  
  initial algorithm

      rocket.pos     := {0, 0, 100};
      rocket.euler   := {0, 0.2, 0};
      imu.meas_pos   := rocket.pos;
      imu.meas_euler := rocket.euler;

  equation

    imu.acc   = rocket.acc;
    imu.vel   = rocket.vel;
    imu.pos   = rocket.pos;
    imu.omega = rocket.omega;
    imu.euler = rocket.euler;

    control.reqPos[1] = 0;
    control.reqPos[2] = 0;
    control.reqPos[3] = 100 - time;
    
    control.reqEuler[1] = 0;
    control.reqEuler[2] = 0;
    control.reqEuler[3] = 0;

    control.imu_pos   = rocket.pos;
    control.imu_vel   = rocket.vel;
    control.imu_euler = rocket.euler;
    control.imu_omega = rocket.omega;
    control.fuelMass  = rocket.fuelMass;
    control.dryMass   = rocket.dryMass;

    engine.thrustCmd       = control.thrustCmd;
    engine.gimbalPitchCmd  = control.gimbalPitchCmd;
    engine.gimbalYawCmd    = control.gimbalYawCmd;

    rocket.thrustMagnitude = engine.thrustMagnitude;
    rocket.gimbalPitch     = engine.gimbalPitch;
    rocket.gimbalYaw       = engine.gimbalYaw;

    // Send data at each step
    when sample(0, 0.01) then // Send every 0.01 seconds
      sendUDP(
        {
          time,
          rocket.pos[1],
          rocket.pos[2],
          rocket.pos[3],
          rocket.euler[1],
          rocket.euler[2],
          rocket.euler[3],
          rocket.thrustMagnitude,
          rocket.thrustVector[1],
          rocket.thrustVector[2],
          rocket.thrustVector[3],
          time
        },
        "127.0.0.1",
        5002);
    end when;

    annotation(
      experiment(StartTime=0, StopTime=100, Interval=0.01),
      __OpenModelica_simulationFlags(
      rt=1.0,  // Real-time factor (1.0 = real-time, 0.5 = half speed, 2.0 = double speed)
      s="euler",  // Fixed-step solver
      lv="LOG_STATS"
      ),
      Documentation(info="<html>
        <p>Test case: Rocket starts at 10m and maintains hover.</p>
        </html>"));

  end Landing;