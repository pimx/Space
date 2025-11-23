within Flights;

model Flight7

  Dynamics      dynamics;
  ControlLaunch control;
  Aerodynamics  aerodynamics;
  MemsImu.IMU laserImu;
  
  parameter Real pos0_ECEF[3] = Transformations.pos_WGS_to_ECEF(0,0,0) "Start position at WGS 0,0,0";
  parameter Real t0           = 0 "Epoch time [s]";
  //parameter Real q_NB[4]      = Constants.q_NB_V "Vertical start";
  
  //Real q_NED_BDY[4];
  Real A_alt;
  Real A_eas;
  Real A_nor;
  //Real A_alt_IMU;
  //Real A_eas_IMU;
  //Real A_nor_IMU;
  //Real omega[3];
  
equation

  laserImu.omega_BDY = dynamics.omega_BDY;
  laserImu.acc_BDY = dynamics.acc_BDY;
  
  //omega = laserImu.omega_NED;
  //acc   = laserImu.acc_NED;
  //der(alpha) = omega;
  //der(vel)   = acc;

  aerodynamics.v_body     = dynamics.vel_BDY "Velocity in body frame [m/s]";
  aerodynamics.omega_body = dynamics.omega_BDY "Angular velocity in body frame [rad/s]";
  aerodynamics.altitude   = -dynamics.pos_ECEF[3] "Altitude [m]";

  //q_NED_BDY = dynamics.q_NED_BDY;
  //dynamics.q_NED_BDY = q_NB;
  //control.q_NED_BDY = dynamics.q_NED_BDY;

  //control.pos_NED   = dynamics.pos_ECEF;
  //control.vel_NED   = dynamics.vel_ECEF;
  //control.angle_NED = dynamics.angle_ECEF;
  //control.omega_NED = dynamics.omega_ECEF;
  
  dynamics.mass            = 1;
  dynamics.for_thrust_BDY  = control.for_thrust_BDY;
  dynamics.mom_thrust_BDY  = control.mom_thrust_BDY;

  dynamics.for_aero_BDY    = {0,0,0}; // aerodynamics.F_aero / dynamics.mass; // control.for_aero_BDY;
  dynamics.mom_aero_BDY    = {0,0,0}; // aerodynamics.M_aero / dynamics.mass; //control.mom_aero_BDY;

  A_nor =  dynamics.pos_ECEF[3] - pos0_ECEF[3];
  A_eas =  dynamics.pos_ECEF[2] - pos0_ECEF[2];
  A_alt =  dynamics.pos_ECEF[1] - pos0_ECEF[1];

  //A_nor_IMU =  laserImu.pos_imu_ECEF[3];
  //A_eas_IMU =  laserImu.pos_imu_ECEF[2];
  //A_alt_IMU =  laserImu.pos_imu_ECEF[1];

  annotation(experiment(StartTime=0, StopTime=10, Interval=0.01));

end Flight7;
