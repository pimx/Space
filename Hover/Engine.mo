within Hover;

model Engine "Rocket engine with thrust vectoring"
	
	//import SI = Modelica.Units.SI;
	import Modelica.Math.*;
	import Modelica.Constants.pi;


	Chamber	chamber01(angle=pi/2*0, offset=1.0);
	Chamber chamber02(angle=pi/2*1, offset=1.0);
	Chamber chamber03(angle=pi/2*2, offset=1.0);
	Chamber chamber04(angle=pi/2*3, offset=1.0);

  
	// Parameters
	//parameter Real maxThrust = 196200 "Maximum thrust [N] (20000 kgf)";
	parameter Real fuelMassInit = 10000 "Initial fuel mass [kg]";
	//parameter Real maxTotalThrust = 200000;
    //parameter Real specificImpulse = 300 "Specific impulse [s]";
    //parameter Real g0 = 9.80665 "Standard gravity [m/s^2]";
  
	// State variables
	//Real fuelMass(start=fuelMassInit, fixed=true) "Remaining fuel";
	//Real thrustMagnitudeState(start=0, fixed=true) "Throttle state";
	//Real thrustDirectionState(start=0, fixed=true) "Gimbal pitch angle";
	//Real gimbalYawState(start=0, fixed=true) "Gimbal yaw angle";

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
	//maxTotalThrust = chamber01.maxThrust + chamber02.maxThrust + chamber03.maxThrust + chamber04.maxThrust;
	//requiredMagnitude = maxTotalThrust / 2;

	chamber01.requiredThrottle  = requiredMagnitude / 4 / chamber01.maxThrust;
	chamber01.requiredDirection = requiredDirection;
	chamber01.centerPosition    = centerPosition;

	chamber02.requiredThrottle  = requiredMagnitude / 4 / chamber02.maxThrust;
	chamber02.requiredDirection = requiredDirection;
	chamber02.centerPosition    = centerPosition;

	chamber03.requiredThrottle  = requiredMagnitude / 4 / chamber03.maxThrust;
	chamber03.requiredDirection = requiredDirection;
	chamber03.centerPosition    = centerPosition;

	chamber04.requiredThrottle  = requiredMagnitude / 4 / chamber04.maxThrust;
	chamber04.requiredDirection = requiredDirection;
	chamber04.centerPosition    = centerPosition;

	actualVector    = chamber01.chamberVector + chamber02.chamberVector + chamber03.chamberVector + chamber04.chamberVector;
	actualMagnitude = chamber01.chamberMagnitude + chamber02.chamberMagnitude + chamber03.chamberMagnitude + chamber04.chamberMagnitude;
	actualFuelRate  = chamber01.chamberFuelRate + chamber02.chamberFuelRate + chamber03.chamberFuelRate + chamber04.chamberFuelRate;
	actualMoments   = chamber01.chamberMoments + chamber02.chamberMoments + chamber03.chamberMoments + chamber04.chamberMoments;
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
