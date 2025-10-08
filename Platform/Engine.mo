within Platform;

model Engine "Rocket engine with thrust vectoring"
  import SI = Modelica.Units.SI;
  
  // Parameters
  parameter Real maxThrust = 196200 "Maximum thrust [N] (20000 kgf)";
  parameter Real fuelMassInit = 10000 "Initial fuel mass [kg]";
    parameter Real specificImpulse = 300 "Specific impulse [s]";
    parameter Real g0 = 9.80665 "Standard gravity [m/s^2]";
  
  // State variables
  Real fuelMass(start=fuelMassInit, fixed=true) "Remaining fuel";
  Real thrustCmdState(start=0, fixed=true) "Throttle state";
  Real gimbalPitchState(start=0, fixed=true) "Gimbal pitch angle";
  Real gimbalYawState(start=0, fixed=true) "Gimbal yaw angle";

  // Input from Control
  Real gimbalPitchCmd, gimbalYawCmd "Gimbal angles [rad]";
  Real thrustCmd "Commanded thrust magnitude [N]";

  // Output
  Real thrustMagnitude;
  Real gimbalPitch;
  Real gimbalYaw;

  
  // Internal variables
  Real fuelFlowRate;
  
equation
  // Throttle dynamics
  der(thrustCmdState)      = 5  * (thrustCmd - thrustCmdState);
  
  // Gimbal dynamics
  der(gimbalPitchState) = 10 * max(-1, min(1, gimbalPitchCmd - gimbalPitchState));
  der(gimbalYawState)   = 10 * max(-1, min(1, gimbalYawCmd   - gimbalYawState));
  
  // Thrust calculation
  thrustMagnitude = if fuelMass > 0 then thrustCmdState * maxThrust else 0;
  gimbalPitch = gimbalPitchState;
  gimbalYaw = gimbalYawState;
  
  // Thrust direction
  //thrustDirection = {
  //  sin(gimbalYaw),
  //  sin(gimbalPitch),
  //  cos(gimbalPitch) * cos(gimbalYaw)
  //};
  
  // Fuel consumption
  //fuelFlowRate = if fuelMass > 0 then thrustMagnitude / exhaustVelocity else 0;
  //der(fuelMass) = -fuelFlowRate;
  
  // Send to bus
  //dataBus.thrustDirection = thrustDirection;
  //dataBus.fuelMass = fuelMass;
  //dataBus.fuelFlowRate = fuelFlowRate;
  
    // Fuel consumption
    fuelFlowRate  = if thrustMagnitude > 0 then -thrustMagnitude / (specificImpulse * g0) else 0;
    der(fuelMass) = -fuelFlowRate;

    
end Engine;
