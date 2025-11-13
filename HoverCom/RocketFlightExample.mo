model RocketFlightExample "Example simulation of rocket flight"
  extends Rocket6DOF.Rocket;
  
  // Override initial conditions for this example
  // Launch from equator at 100 km altitude, moving eastward
  parameter Real r0_ECI[3] = {Rocket6DOF.Constants.R_earth + 100e3, 0, 0};
  parameter Real v0_ECI[3] = {0, 7500, 100}; // Orbital velocity + some vertical
  
  // Simple thrust profile for demonstration
  Real thrust_magnitude;
  Real thrust_direction[3];
  
equation
  // Example thrust profile: burn for 30 seconds, then coast
  if time < 30 then
    thrust_magnitude = 80000.0; // 80 kN
    // Thrust mostly in X direction (forward), small pitch up
    thrust_direction = {1.0, 0.0, -0.1};
  else
    thrust_magnitude = 0.0;
    thrust_direction = {1.0, 0.0, 0.0};
  end if;
  
  // Connect to rocket inputs
  T_cmd = thrust_magnitude;
  u_thrust_cmd = thrust_direction;
  
  annotation(experiment(StartTime=0, StopTime=100, Tolerance=1e-6));
  
end RocketFlightExample;
