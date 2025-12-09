within Flights;

model ControlLaunch  "Control system for launch"

  Aerodynamics aerodynamics;
  Gravity      gravity;
  
  //constant Real g0               = Constants.g0          "Standard gravity [m/s^2]";

  parameter Real pos0_ECEF[3] = Transformations.pos_WGS_to_ECEF(0,0,0)    "положение начала системы координат NED (старт)";
  
  //input Real pos_NED[3](start={0,0,0})       "положение в местной системе координат";
  //input Real vel_NED[3](start={0,0,0})       "скорость в местной системе координат";
  //input Real angle_NED[3](start={0,0,0})     "углы (ориентация аппарата) в местной системе координат";
  //input Real omega_NED[3](start={0,0,0})     "угловые скорости в собственной системе координат";

  input  Real mass;
  input  Real q_BDY_ECEF[4];
  input  Real inertia[3,3];
  input  Real omega_ECEF[3];
  input  Real pos_ECEF[3];
  input  Real vel_ECEF[3];
  input  Real acc_ECEF[3];

  output Real for_thrust_BDY[3](start={0,0,0})    "требуемая тяга и направление";
  output Real mom_thrust_BDY[3](start={0,0,0})    "требуемые моменты";

  Real omega_BDY[3];
  Real vel_BDY[3];
  //Real for_thrust_NED[3];
  //Real for_aero_NED[3];
  Real alt,vel,acc,tim,mg0;
  Real for_aero_BDY[3];
  Real mom_aero_BDY[3];
  Real for_grav_BDY[3];
  Real mom_grav_BDY[3];

equation

  gravity.pos_ECEF = pos_ECEF;

  omega_BDY = Frames.AngularVelocityTransforms.ECEF_to_BODY(omega_ECEF, q_BDY_ECEF);
  vel_BDY   = Frames.VelocityTransforms.ECEF_to_BODY(vel_ECEF, {0,0,0}, q_BDY_ECEF, {0,0,0});
  aerodynamics.v_body     =  vel_BDY                              "Velocity in body frame [m/s]";
  aerodynamics.omega_body =  omega_BDY                            "Angular velocity in body frame [rad/s]";
  aerodynamics.altitude   =  pos_ECEF[1] - Constants.pos0_ECEF[1] "Altitude [m]";
  for_aero_BDY    = aerodynamics.F_aero; // control.for_aero_BDY;
  mom_aero_BDY    = aerodynamics.M_aero; //control.mom_aero_BDY;



  mg0 = mass*gravity.grav_mag;
  alt = pos_ECEF[1] - Constants.pos0_ECEF[1];
  vel = vel_ECEF[1];

  if time < 50 then                 // набор высоты
    acc = 0;
    tim = 0;
    for_grav_BDY   = {mg0*2.5, 0, 0};
    mom_grav_BDY   = {0,       0, 0};
    for_thrust_BDY = { for_grav_BDY[1] - for_aero_BDY[1], 0, 0};
    mom_thrust_BDY = { 0,0,0 }; //mom_grav_BDY - mom_aero_BDY;
  elseif time < 150 then            // набор высоты по инерции
    acc = 0;
    tim = 0;
    for_grav_BDY   = {0,     0, 0};
    mom_grav_BDY   = {0,     0, 0};
    for_thrust_BDY = {0,     0, 0};
    mom_thrust_BDY = {0,     0, 0};
  elseif alt > 0.01 and vel < 0 and alt < 40000 then  // посадка
    acc = 0.5*vel^2/alt;
    tim = 2*alt/vel;
    for_grav_BDY   = {mg0+acc*mass,  0, 0};
    mom_grav_BDY   = {0,        0, 0};
    for_thrust_BDY = { for_grav_BDY[1] - for_aero_BDY[1], 0, 0};
    mom_thrust_BDY = { 0,0,0 }; //mom_grav_BDY - mom_aero_BDY;
  else                                // снижение по инерции
    acc = 0;
    tim = 0;
    for_grav_BDY   = {0,     0, 0};
    mom_grav_BDY   = {0,     0, 0};
    for_thrust_BDY = {0,     0, 0};
    mom_thrust_BDY = {0,     0, 0};
  end if;

end ControlLaunch;
