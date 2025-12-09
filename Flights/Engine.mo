within Flights;

model Engine "Rocket engine with thrust vectoring"
	
	Chamber chamber[9] (
		angle  = { 0, pi/4*0, pi/4*1, pi/4*2, pi/4*3, pi/4*4, pi/4*5, pi/4*6, pi/4*7 },
		offset = { 0, 1,      1,      1,      1,      1,      1,      1,      1      }
	);


	constant Real maxThrust     =  2.00 * (Constants.M_dry+Constants.M_fuel) * Constants.g0  "максимальная тяга 169 / SCALE^3 * M_G0 * M_IMPULSE [N]";
	constant Real minThrust     =  1.15 * Constants.M_dry * Constants.g0  "минимальная  тяга  91 / SCALE^3 * M_G0 * M_IMPULSE [N]";

	// Input from Control
	input Real requiredDirection[3] "Gimbal angles [rad]";
	input Real requiredMagnitude "Commanded thrust magnitude [N]";
	input Real centerPosition[3];

	// Output
	output Real actualVector[3];
	output Real actualDirection[3];
	output Real actualMagnitude;
	output Real actualFuelRate;
	output Real actualMoments[3];

  
	// Internal variables
	//Real fuelFlowRate;
	//Real maxTotalThrust;
  
equation

	chamber[1].requiredThrottle  = min(requiredMagnitude / maxThrust, 1.0);
	chamber[1].requiredDirection = requiredDirection;
	chamber[1].centerPosition    = centerPosition;
	for i in 2:9 loop
		chamber[i].requiredThrottle  = 0;
		chamber[i].requiredDirection = requiredDirection;
		chamber[i].centerPosition    = centerPosition;
	end for;
	
	actualVector    = sum(chamber[i].chamberVector for i in 1:size(chamber, 1));
	actualMagnitude = sum(chamber.chamberMagnitude);
	actualFuelRate  = sum(chamber.chamberFuelRate);
	actualMoments   = sum(chamber[i].chamberMoments for i in 1:size(chamber, 1));
	
	actualDirection = if abs(actualMagnitude) > 0 then Vectors.normalize(actualVector) else {1,0,0};

  // Throttle dynamics
  //der(thrustCmdState)      = 5  * (thrustCmd - thrustCmdState);
  
  // Gimbal dynamics
  //der(gimbalPitchState) = 10 * max(-1, min(1, gimbalPitchCmd - gimbalPitchState));
  //der(gimbalYawState)   = 10 * max(-1, min(1, gimbalYawCmd   - gimbalYawState));
  
  // Thrust calculation
  //thrustMagnitude = if fuelMass > 0 then thrustCmdState * maxThrust else 0;
  //gimbalPitch = gimbalPitchState;
  //gimbalYaw = gimbalYawState;
  
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
    //fuelFlowRate  = if thrustMagnitude > 0 then -thrustMagnitude / (specificImpulse * g0) else 0;
    //der(fuelMass) = -fuelFlowRate;

    
end Engine;
