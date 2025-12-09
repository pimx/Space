within Flights;

model ControlLanding
  "Control system for Landing"
  
  constant Real g0               = Constants.g0          "Standard gravity [m/s^2]";

  parameter Real pos0_ECEF[3] = Transformations.pos_WGS_to_ECEF(0,0,0)    "положение начала системы координат NED (старт)";
  
  input Real pos_NED[3](start={0,0,0})       "положение в местной системе координат";
  input Real vel_NED[3](start={0,0,0})       "скорость в местной системе координат";
  input Real angle_NED[3](start={0,0,0})     "углы (ориентация аппарата) в местной системе координат";
  input Real omega_NED[3](start={0,0,0})     "угловые скорости в собственной системе координат";
  input Real q_NED_BDY[4];

  output Real for_thrust_BDY[3](start={0,0,0})    "требуемая тяга и направление";
  output Real for_aero_BDY[3](start={0,0,0})      "требуемая тяга и направление";
  output Real mom_thrust_BDY[3](start={0,0,0})    "требуемые моменты";
  output Real mom_aero_BDY[3](start={0,0,0})      "требуемые моменты";

  //Real for_thrust_NED[3];
  //Real for_aero_NED[3];

equation

  if time < 100 then
    for_thrust_BDY = {0.00,  0.00,  g0+1};
    for_aero_BDY   = {0.00,  0.00,   0.00};
    mom_thrust_BDY = {0.00,  0.00,   0.00};
    mom_aero_BDY   = {0.00,  0.00,   0.00};
  else
    for_thrust_BDY = {0.00,  0.00,  g0-1};
    for_aero_BDY   = {0.00,  0.00,   0.00};
    mom_thrust_BDY = {0.00,  0.00,   0.00};
    mom_aero_BDY   = {0.00,  0.00,   0.00};
  end if;

end ControlLanding;
