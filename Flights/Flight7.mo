within Flights;

model Flight7

  Dynamics      dynamics;
  Masses        masses;
  ControlLaunch control;
  Aerodynamics  aerodynamics;
  MemsImu.IMU   imu;
  Gravity       gravity;
  Engine        engine;
  
  parameter Real pos0_ECEF[3] = Transformations.pos_WGS_to_ECEF(0,0,0) "Start position at WGS 0,0,0";
  parameter Real t0           = 0 "Epoch time [s]";
  //parameter Real q_NB[4]      = Constants.q_NB_V "Vertical start";
  
  //Real q_NED_BDY[4];
  Real A_alt;
  //Real A_eas;
  //Real A_nor;
  //Real A_pos;
  Real A_vel;
  //Real A_acc;
  Real thv[3];
  Real th[3];
  Real thm;
  
equation

  imu.omega_DYN = dynamics.omega_IMU;
  imu.acc_DYN   = dynamics.acc_IMU;
  
  aerodynamics.v_body     =  dynamics.vel_BDY "Velocity in body frame [m/s]";
  aerodynamics.omega_body =  dynamics.omega_BDY "Angular velocity in body frame [rad/s]";
  aerodynamics.altitude   =  dynamics.pos_ECEF[1] - Constants.R_earth "Altitude [m]";

  gravity.pos_ECEF = dynamics.pos_ECEF;

  masses.rateFuel = engine.actualFuelRate;
  //q_NED_BDY = dynamics.q_NED_BDY;
  //dynamics.q_NED_BDY = q_NB;
  //control.q_NED_BDY = dynamics.q_NED_BDY;

  control.mass       = masses.mass;
  control.inertia    = masses.inertia;
  control.q_BDY_ECEF = dynamics.q_BDY_ECEF;
  control.omega_ECEF = dynamics.omega_ECEF;
  control.pos_ECEF   = dynamics.pos_ECEF;
  control.vel_ECEF   = dynamics.vel_ECEF;
  control.acc_ECEF   = dynamics.acc_ECEF;
  //control.angle_NED = dynamics.angle_ECEF;
  //control.omega_NED = dynamics.omega_ECEF;

  th = control.for_thrust_BDY;
  thm = sqrt(th[1]*th[1] + th[2]*th[2] + th[3]*th[3]);
  if thm > 0.1 then
    thv = th / thm;
  else
    thv = {1,0,0};
  end if;

  engine.requiredDirection = thv;
  engine.requiredMagnitude = thm;
  engine.centerPosition    = masses.centerPosition;
  
  dynamics.gravity_ECEF    = gravity.grav_ECEF;
  dynamics.mass            = masses.mass;
  dynamics.inertia         = masses.inertia;
  dynamics.for_thrust_BDY  = engine.actualVector;
  dynamics.mom_thrust_BDY  = engine.actualMoments;

  dynamics.for_aero_BDY    = aerodynamics.F_aero; // control.for_aero_BDY;
  dynamics.mom_aero_BDY    = aerodynamics.M_aero; //control.mom_aero_BDY;

  //A_nor =  dynamics.pos_ECEF[3] - pos0_ECEF[3];
  //A_eas =  dynamics.pos_ECEF[2] - pos0_ECEF[2];
  A_alt =  dynamics.pos_ECEF[1] - pos0_ECEF[1];

  //A_pos = sqrt((dynamics.pos_ECI[1]-laserImu.pos_ECI[1])^2+(dynamics.pos_ECI[2]-laserImu.pos_ECI[2])^2+(dynamics.pos_ECI[3]-laserImu.pos_ECI[3])^2);
  A_vel = sqrt((dynamics.vel_ECI[1]-imu.vel_ECI[1])^2+(dynamics.vel_ECI[2]-imu.vel_ECI[2])^2+(dynamics.vel_ECI[3]-imu.vel_ECI[3])^2);
  //A_acc = sqrt((dynamics.acc_ECI[1]-laserImu.acc_ECI[1])^2+(dynamics.acc_ECI[2]-laserImu.acc_ECI[2])^2+(dynamics.acc_ECI[3]-laserImu.acc_ECI[3])^2);

  //A_nor_IMU =  laserImu.pos_imu_ECEF[3];
  //A_eas_IMU =  laserImu.pos_imu_ECEF[2];
  //A_alt_IMU =  laserImu.pos_imu_ECEF[1];

  //when dynamics.pos_ECEF[1] - pos0_ECEF[1] < 0 then
    
  //end when;

  annotation(experiment(StartTime=0, StopTime=450, Interval=0.01));

end Flight7;
