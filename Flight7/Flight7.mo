within Flight7;

model Flight7
  //import CT = CoordinateTransformations;
  
  parameter Real r_origin_ECEF[3] = Transformations.geodetic_to_ECEF(0,0,0);
  parameter Real t0 = 0 "Epoch time [s]";
  
  Real q[4](start={1,0,0,0}) "Orientation quaternion";
  Real omega_body[3](start={0,0,0}) "Angular velocity [p,q,r] [rad/s]";
  Real alpha_body[3](start={0,0,0}) "Angular acceleration [rad/s²]";
  
  Real r_ECI[3](start=Transformations.position_Local_to_ECI({0,0,0}, t0, r_origin_ECEF));
  Real v_ECI[3](start=Transformations.velocity_Local_to_ECI({0,0,0}, {0,0,0}, t0, r_origin_ECEF));
  Real a_ECI[3](start={0,0,0});

  //Real r_ECI[3](start=position_ECEF_to_ECI(r_origin_ECEF, t0));
  //Real v_ECI[3](start={0,0,0});
  //Real a_ECI[3](start={0,0,0});
  Real r_NED[3](start={0,0,0});
  Real v_NED[3](start={0,0,0}) "Velocity in NED [m/s]";

  Real v_body[3](start={0,0,0}) "Velocity in body frame [m/s]";
  
  Real F_thrust_ECI[3] "Thrust in inertial frame";
  Real F_aero_ECI[3] "Aero in inertial frame";
  Real F_gravity_ECI[3] "Gravity in inertial frame";

  // Forces
  Real F_thrust_body[3] "Thrust in body frame [N]";
  Real F_aero_body[3] "Aerodynamic forces in body frame [N]";
  Real F_body[3] "Aerodynamic and thrust forces in body frame [N]";

  Real F_thrust_NED[3] "Thrust in NED frame [N]";
  Real F_aero_NED[3] "Aerodynamic forces in NED frame [N]";
  //Real F_gravity_NED[3] = {0, 0, 9.81*1000} "Gravity [N]";
  //Real F_gravity_body[3] "Gravity in body frame [N]";
  
  // Moments in body frame
  Real M_body[3] "Total moment [N⋅m]";
  Real M_thrust_body[3] "Thrust vectoring moment [N⋅m]";
  Real M_aero_body[3] "Aerodynamic moment [N⋅m]";
  
  // Mass properties
  parameter Real mass = 1000 "Mass [kg]";
  parameter Real I_xx = 100 "Moment of inertia [kg⋅m²]";
  parameter Real I_yy = 1000 "Moment of inertia [kg⋅m²]";
  parameter Real I_zz = 1000 "Moment of inertia [kg⋅m²]";
  Real I[3,3] = diagonal({I_xx, I_yy, I_zz}) "Inertia tensor";

  Real A_alt;
  
equation

  F_thrust_body = {0,0,-12000};
  F_aero_body   = {0,0,2000};
  F_body        = F_thrust_body + F_aero_body;                        // Total moment

  M_thrust_body = {0,0,0};
  M_aero_body   = {0,0,0};
  M_body        = M_thrust_body + M_aero_body;                        // Total moment

  F_thrust_NED  = Transformations.force_Body_to_NED_quat(F_thrust_body, q);
  F_aero_NED    = Transformations.force_Body_to_NED_quat(F_aero_body, q);

  F_thrust_ECI  = Transformations.force_Local_to_ECI(F_thrust_NED, time, r_origin_ECEF);
  F_aero_ECI    = Transformations.force_Local_to_ECI(F_aero_NED, time, r_origin_ECEF);
  F_gravity_ECI = -3.986004418e14 * r_ECI / (sqrt(r_ECI*r_ECI))^3;    // Gravity (computed in ECI)


  a_ECI = (F_thrust_ECI + F_aero_ECI + F_gravity_ECI) / mass;         // Equations of motion in ECI (inertial frame)
  der(v_ECI) = a_ECI;
  der(r_ECI) = v_ECI;

  I * der(omega_body) + cross(omega_body, I*omega_body) = M_body;     // Euler's equation for rotation
  der(q) = Transformations.quaternion_derivative(q, omega_body);                      // Quaternion kinematics (no gimbal lock!)


  // Transform to local frame for monitoring
  r_NED = Transformations.position_ECI_to_Local(r_ECI, time, r_origin_ECEF);
  v_NED = Transformations.velocity_ECI_to_Local(r_ECI, v_ECI, time, r_origin_ECEF);
//  a_NED = acceleration_Body_to_NED_quat()

  A_alt = -r_NED[3];

  // Transform velocities
  v_body = Transformations.velocity_NED_to_Body_quat(v_NED, q);
//  F_gravity_body = force_NED_to_Body_quat(F_gravity_NED, q);
  

  

  
  // Extract alpha_body for use in other equations
  alpha_body = {der(omega_body[1]), der(omega_body[2]), der(omega_body[3])};
  
  // Translational dynamics in NED
  //mass * der(v_NED) = F_thrust_NED + F_gravity_NED + force_Body_to_NED_quat(F_aero_body, q);
  
  annotation(experiment(StartTime=0, StopTime=10, Interval=0.001));

end Flight7;
