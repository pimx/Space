within Flights;

model Dynamics
  
  parameter Real t0 = 0 "Epoch time [s]";
  parameter Real pos0_ECEF[3] = Constants.pos0_ECEF; // Transformations.pos_WGS_to_ECEF(0,0,0);
  parameter Real pos1_ECEF[3] = Constants.pos0_ECEF; //Transformations.pos_WGS_to_ECEF(0,0,0);
  // Mass properties
  //parameter Real mass = 1000 "Mass [kg]";
  parameter Real D = Constants.D_ref;                           // диаметр корпуса
  parameter Real L = Constants.L_ref;                           // длина корпуса
  parameter Real I_xx = 1.0 / 2.0 * (D/2)^2;                    // 100 "Moment of inertia [kg⋅m²]";
  parameter Real I_yy = 1.0 / 4.0 * (D/2)^2 + 1.0 / 12.0 * L^2; //1000 "Moment of inertia [kg⋅m²]";
  parameter Real I_zz = 1.0 / 4.0 * (D/2)^2 + 1.0 / 12.0 * L^2; //1000 "Moment of inertia [kg⋅m²]";
  
  //Real I[3,3] = diagonal({I_xx, I_yy, I_zz}) "Inertia tensor without mass";
  
  // state
  //Real q_NED_BDY[4](start=Constants.q_NB_V) "Vertical start";
  Real q_BDY_ECEF[4](start={1,0,0,0});

  Real angle_ECI[3](start={0,0,0}) "Angles [p,q,r] [rad]";
  Real omega_ECI[3](start={0,0,0}) "Angular velocity [p,q,r] [rad/s]";
  Real alpha_ECI[3](start={0,0,0}) "Angular accelerations [p,q,r] [rad/s^2]";
  Real pos_ECI[3](start=Frames.PositionTransforms.ECEF_to_ECI(pos0_ECEF, 0));
  Real vel_ECI[3](start=Frames.VelocityTransforms.ECEF_to_ECI({0,0,0}, pos0_ECEF, 0));
  Real acc_ECI[3](start=Frames.AccelerationTransforms.ECEF_to_ECI({0,0,0}, {0,0,0}, pos0_ECEF, 0));
  
  // input
  input Real mass;
  input Real for_thrust_BDY[3] "Thrust in body frame [N]";
  input Real for_aero_BDY[3] "Aerodynamic forces in body frame [N]";
  input Real mom_thrust_BDY[3] "Thrust vectoring moment [N⋅m]";
  input Real mom_aero_BDY[3] "Aerodynamic moment [N⋅m]";

  // output
  output Real pos_BDY[3](start={0,0,0});
  output Real vel_BDY[3](start={0,0,0});
  output Real acc_BDY[3](start={0,0,0});
  output Real angle_BDY[3](start={0,0,0});
  output Real omega_BDY[3](start={0,0,0});
  output Real alpha_BDY[3](start={0,0,0});
  

  // temporary
   Real pos_ECEF[3];
   Real vel_ECEF[3];
   Real acc_ECEF[3];
   Real angle_ECEF[3];
   Real omega_ECEF[3];
   Real alpha_ECEF[3];


  Real theta;
  Real inertia_tenzor[3,3];

  Real gravity_ECI[3] "Gravity in inertial frame";
  Real gravity_ECEF[3] "Gravity in inertial frame";
  Real q_dot[4];

  Real for_BDY[3] "Aerodynamic and thrust forces in body frame [N]";
  Real mom_BDY[3] "Total moment [N⋅m]";
  Real for_ECI[3] "Aerodynamic and thrust forces in body frame [N]";
  //Real mom_ECI[3] "Total moment [N⋅m]";

  Real acc_ECI_imu[3];
  Real vel_ECI_imu[3];
  Real pos_ECI_imu[3];
  Real angle_ECI_imu[3];
  Real omega_ECI_imu[3];
  Real alpha_ECI_imu[3];

equation

  // Параметры
  //    Точка старта        в земной      СК  pos0_ECEF
  //    Точка посадки       в земной      СК  pos1_ECEF
  // Состояния:
  //    Положение           в местной     СК  pos_NED
  //    Скорость            в местной     СК  vel_NED
  //    Углы                в местной     СК  angle_NED
  //    Угловые скорости    в местной     СК  omega_NED
  // Вход:
  //    Действующие силы    в собственной СК  thrust_BOD + aero_BOD
  //    Действующие моменты в собственной СК  thrust_BOD + aero_BOD
  // Выход
  //    Угловые ускоренния  в собственной СК  alpha_BOD
  //    Ускорения           в собственной СК  acc_BOD
  //    Смещение ЦТ         в собственной СК  dis_BOD

  gravity_ECI  = -3.986004418e14 * pos_ECI  / (sqrt(pos_ECI*pos_ECI))^3;      // Gravity (computed in ECI)
  gravity_ECEF = -3.986004418e14 * pos_ECEF / (sqrt(pos_ECEF*pos_ECEF))^3;    // Gravity (computed in ECEF)

  theta = Frames.Constants.omega_earth * (time+t0);

  for_BDY        = for_thrust_BDY + for_aero_BDY;                        // Total force
  mom_BDY        = mom_thrust_BDY + mom_aero_BDY;                        // Total moment

  for_ECI = Frames.PositionTransforms.BODY_to_ECI(for_BDY, q_BDY_ECEF, theta);
  acc_ECI = for_ECI / mass + gravity_ECI;
  der(pos_ECI) = vel_ECI;
  der(vel_ECI) = acc_ECI;

  q_dot = Frames.QuaternionOps.derivative(q_BDY_ECEF, {0,0,0}); //omega_BDY);
  der(q_BDY_ECEF) = q_dot;

  inertia_tenzor = diagonal({I_xx, I_yy, I_zz});
  //inertia_tenzor * der(omega_BDY) + cross(omega_BDY, inertia_tenzor*omega_BDY) = mom_BDY;     // Euler's equation for rotation

  der(omega_BDY) = solve(inertia_tenzor, mom_BDY - cross(omega_BDY, inertia_tenzor * omega_BDY));
  der(omega_BDY) = alpha_BDY;

  omega_ECEF = Frames.AngularVelocityTransforms.BODY_to_ECEF(omega_BDY, q_BDY_ECEF);
  omega_ECI  = Frames.AngularVelocityTransforms.ECEF_to_ECI(omega_ECEF, theta);
  alpha_ECEF = Frames.AngularAccelerationTransforms.BODY_to_ECEF(alpha_BDY, q_BDY_ECEF);
  alpha_ECI  = Frames.AngularAccelerationTransforms.ECEF_to_ECI(alpha_ECEF, theta);

  //der(omega_ECI) = solve(inertia_tenzor, mom_BDY - cross(omega_ECI, inertia_tenzor * omega_ECI));
  //alpha_ECI = der(omega_ECI);
  //der(omega_ECI) = alpha_ECI;
  der(angle_ECI) = omega_ECI;

  //output Real pos_BDY[3];
  //output Real vel_BDY[3];
  //output Real acc_BDY[3];
  //output Real angle_BDY[3];
  //output Real omega_BDY[3];
  //output Real alpha_BDY[3];

  pos_ECEF = Frames.PositionTransforms.ECI_to_ECEF(pos_ECI, theta);
  vel_ECEF = Frames.VelocityTransforms.ECI_to_ECEF(vel_ECI, pos_ECI, theta);
  acc_ECEF = Frames.AccelerationTransforms.ECI_to_ECEF(acc_ECI, vel_ECI, pos_ECI, theta);
  angle_ECEF = Frames.AngularVelocityTransforms.ECI_to_ECEF(angle_ECI, theta);
  //omega_ECEF = Frames.AngularVelocityTransforms.relativeBodyECEF(omega_ECI, q_BDY_ECEF);
  //omega_ECEF = Frames.AngularVelocityTransforms.ECI_to_ECEF(omega_ECI, theta);
  //alpha_ECEF = {0,0,0}; //Frames.AngularAccelerationTransforms.ECI_to_ECEF(alpha_ECI, theta);



  pos_BDY  = Frames.PositionTransforms.ECEF_to_BODY(pos_ECEF-pos0_ECEF, q_BDY_ECEF);
  vel_BDY  = Frames.VelocityTransforms.ECEF_to_BODY(vel_ECEF, pos_ECEF-pos0_ECEF, q_BDY_ECEF, omega_ECEF);
  acc_BDY = Frames.AccelerationTransforms.ECEF_to_BODY(acc_ECEF, vel_ECEF, pos_ECEF-pos0_ECEF, q_BDY_ECEF, omega_ECEF, alpha_ECEF);
  angle_BDY = Frames.AngularVelocityTransforms.ECEF_to_BODY(angle_ECEF, q_BDY_ECEF);
  //omega_BDY = Frames.AngularVelocityTransforms.ECEF_to_BODY(omega_ECEF, q_BDY_ECEF);
  //alpha_BDY = Frames.AngularAccelerationTransforms.ECEF_to_BODY(alpha_ECEF, q_BDY_ECEF, {0,0,0}, {0,0,0});

//#####################################################################################################################

  pos_ECI_imu    = Frames.PositionTransforms.ECEF_to_ECI(pos_ECEF, theta);
  vel_ECI_imu    = Frames.VelocityTransforms.ECEF_to_ECI(vel_ECEF, pos_ECEF, theta);
  acc_ECI_imu    = Frames.AccelerationTransforms.ECEF_to_ECI(acc_ECEF, vel_ECEF, pos_ECEF, theta);
  angle_ECI_imu  = Frames.AngularVelocityTransforms.ECEF_to_ECI(angle_ECEF, theta);
  omega_ECI_imu  = Frames.AngularVelocityTransforms.ECEF_to_ECI(omega_ECEF, theta);
  alpha_ECI_imu  = Frames.AngularAccelerationTransforms.ECEF_to_ECI(alpha_ECEF, theta);

/*
  
  omega_imu_BDY  = omega_BDY;
  omega_imu_ECEF = Frames.AngularVelocityTransforms.BODY_to_ECEF(omega_imu_BDY, q_imu_BDY_ECEF);
  omega_imu_ECI  = Frames.AngularVelocityTransforms.ECEF_to_ECI(omega_imu_ECEF, theta);

  //alpha_imu_BDY  = Frames.AngularAccelerationTransforms.ECEF_to_BODY(alpha_imu_ECEF, q_imu_BDY_ECEF, {0,0,0}, {0,0,0});
  //alpha_imu_ECEF = Frames.AngularAccelerationTransforms.ECI_to_ECEF(alpha_imu_ECI, theta);
  der(omega_imu_ECI) = alpha_imu_ECI;

  q_imu_dot = Frames.QuaternionOps.derivative(q_imu_BDY_ECEF, omega_imu_ECI);
  der(q_imu_BDY_ECEF) = q_imu_dot;

  acc_imu_BDY    = acc_BDY;
  acc_imu_ECEF   = Frames.AccelerationTransforms.BODY_to_ECEF(acc_imu_BDY, vel_imu_BDY, pos_imu_BDY, q_imu_BDY_ECEF, omega_imu_ECEF, alpha_imu_ECEF);
  acc_imu_ECI    = Frames.AccelerationTransforms.ECEF_to_ECI(acc_imu_ECEF, vel_imu_ECEF, pos_imu_ECEF, theta);

  vel_imu_BDY  = Frames.VelocityTransforms.ECEF_to_BODY(vel_imu_ECEF, pos_imu_ECEF, q_imu_BDY_ECEF, omega_imu_ECEF);
  vel_imu_ECEF = Frames.VelocityTransforms.ECI_to_ECEF(vel_imu_ECI, pos_imu_ECI, theta);
  der(vel_imu_ECI) = acc_imu_ECI;

  pos_imu_BDY  = Frames.PositionTransforms.ECEF_to_BODY(pos_imu_ECEF, q_imu_BDY_ECEF);
  pos_imu_ECEF = Frames.PositionTransforms.ECI_to_ECEF(pos_imu_ECI, theta);
  der(pos_imu_ECI) = vel_imu_ECI;

  pos_ECEF = Frames.PositionTransforms.ECI_to_ECEF(pos_ECI, theta);
  vel_ECEF = Frames.VelocityTransforms.ECI_to_ECEF(vel_ECI, pos_ECI, theta);
  acc_ECEF = Frames.AccelerationTransforms.ECI_to_ECEF(acc_ECI, vel_ECI, pos_ECI, theta);
  angle_ECEF = {0,0,0};
  omega_ECEF = Frames.AngularVelocityTransforms.relativeBodyECEF(omega_ECI, q_BDY_ECEF);
  alpha_ECEF = Frames.AngularAccelerationTransforms.ECI_to_ECEF(alpha_ECI, theta);

//Real r_eci[3], v_eci[3], a_eci[3];
//Real q_body_ecef[4], omega_body_eci[3];

// Transform to ECEF
//Real pos_ecef[3] = Frames.PositionTransforms.ECI_to_ECEF(r_eci, theta);
////Real v_ecef[3] = Frames.VelocityTransforms.ECI_to_ECEF(v_eci, r_eci, theta);

// Transform to BODY
//Real omega_body_ecef[3] = Frames.AngularVelocityTransforms.relativeBodyECEF(omega_body_eci, q_body_ecef);
//Real r_body[3] = Frames.PositionTransforms.ECEF_to_BODY(r_ecef, q_body_ecef);
//Real v_body[3] = Frames.VelocityTransforms.ECEF_to_BODY(v_ecef, r_ecef, q_body_ecef, omega_body_ecef);


  for_thrust_NED  = Transformations.force_BODY_to_NED_quat(for_thrust_BDY, q_NED_BDY);
  for_aero_NED    = Transformations.force_BODY_to_NED_quat(for_aero_BDY, q_NED_BDY);

  for_thrust_ECI  = Transformations.force_NED_to_ECI(for_thrust_NED, t0+time, pos0_ECEF);
  for_aero_ECI    = Transformations.force_NED_to_ECI(for_aero_NED, t0+time, pos0_ECEF);

  acc_ECI = (for_thrust_ECI + for_aero_ECI + for_gravity_ECI);         // Equations of motion in ECI (inertial frame)
  der(vel_ECI) = acc_ECI;
  der(pos_ECI) = vel_ECI;

  pos_NED = Transformations.position_ECI_to_NED(pos_ECI, t0+time, pos0_ECEF);
  vel_NED = Transformations.velocity_ECI_to_NED(pos_ECI, vel_ECI, t0+time, pos0_ECEF);
  acc_NED = Transformations.force_ECI_to_NED(acc_ECI, t0+time, pos0_ECEF);

  inertia_tenzor = diagonal({I_xx, I_yy, I_zz});
  inertia_tenzor * der(omega_BDY) + cross(omega_BDY, I*omega_BDY) = mom_BDY;     // Euler's equation for rotation
  der(q_NED_BDY) = Transformations.quaternion_derivative(q_NED_BDY, omega_BDY);                      // Quaternion kinematics (no gimbal lock!)

  pos_BDY = Transformations.force_NED_to_BODY_quat(pos_NED, q_NED_BDY);
  vel_BDY = Transformations.velocity_NED_to_BODY_quat(vel_NED, q_NED_BDY);
  acc_BDY = Transformations.force_NED_to_BODY_quat(acc_NED, q_NED_BDY);
  omega_BDY = der(angle_BDY);
  alpha_BDY = der(omega_BDY);

  //q_NED_BDY = q;
  */

end Dynamics;
