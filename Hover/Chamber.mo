within Hover;

model Chamber "Rocket engine chamber with thrust vectoring"
	//import SI = Modelica.Units.SI;
	import Modelica.Math.*;
	import Modelica.Constants.pi;
	import Modelica.Mechanics.MultiBody.Frames.TransformationMatrices.*;
  
	// Parameters
    parameter Real specificImpulse = 300     "Specific impulse [s]";
    parameter Real g0              = 9.80665 "Standard gravity [m/s^2]";
	parameter Real maxThrust       = 196200  "Maximum thrust [N] (20000 kgf)";
	parameter Real angle           = pi * 2 / 4     "Orientation angle [rad]";
	parameter Real offset = 0.5 "Radial distance of chambers from rocket centerline [m]";
	parameter Real gimbalLimit = 0.2 "20*pi/180 Gimbal angle limit [rad]";

	//Real fuelMass         (start=fuelMassInit, fixed=true) "Remaining fuel";
	//Real thrustCmdState   (start=0, fixed=true) "Throttle state";
	//Real gimbalPitchState (start=0, fixed=true) "Gimbal pitch angle";
	//Real gimbalYawState   (start=0, fixed=true) "Gimbal yaw angle";

	// Input from EngineControl
	input Real requiredThrottle      "Значение тяги 0..1 [N]";
	input Real requiredDirection[3]  "Направление вектора тяги [rad]";
    input Real centerPosition[3]     "Положение центра масс";
	//Real gimbalYawCmd   "Отклонение от плоскости оси ракеты со стороны оси Gimbal angle [rad]";

	// Output to EngineControl
	output Real chamberMagnitude "Thrust magnitude for chamber";
	output Real chamberFuelRate "Fuel rate for chamber";
	output Real chamberDirection[3] "Thrust direction in rocket body frame";
	output Real chamberVector[3] "Thrust vector in rocket body frame";
	output Real chamberMoments[3] "Thrust moments in rocket body frame";
    output Real chamberDeviation;
	//output Real gimbalAngles[2] "Gimbal angles [rad] - [beta_y (pitch), beta_z (yaw)] for each chamber";

	output Real gimbalPitch;
	output Real gimbalYaw;
  
	// Internal variables
    Real chamberPosition[3];
    Real T_rocket_to_chamber[3, 3] "Transformation matrix from rocket to chamber frame";
    Real T_chamber_to_rocket[3, 3] "Transformation matrix from chamber to rocket frame";
    Real momentsPosition[3] "Положение центра масс относительно центра размещения камер";

	Real gimbalPitch_mm;
	Real gimbalYaw_mm;

	Real requiredChamberDirection[3];
	Real computedChamberDirection[3];

	Real c_ang;
    Real s_ang;
    Real c_pitch;
    Real s_pitch;
    Real c_yaw;
    Real s_yaw;

equation

    // Base orientation of chamber i (before gimbal)
	c_ang = if abs(cos(angle)) > 1.0e-15 then cos(angle) else 0;
	s_ang = if abs(sin(angle)) > 1.0e-15 then sin(angle) else 0;

    // Chamber X is along rocket X, Chamber Y points to center (-radial)
    // Chamber position in rocket frame (radial offset from X-axis)
	chamberPosition[1] = 0;  // Axial position (all at same X location)
	chamberPosition[2] = offset * c_ang;
	chamberPosition[3] = offset * s_ang;

    
    // Transformation from rocket to chamber (base orientation)
    T_rocket_to_chamber[1] = {1, 0, 0};            // Chamber X along rocket X
    T_rocket_to_chamber[2] = {0, -c_ang, -s_ang};  // Chamber Y toward center
    T_rocket_to_chamber[3] = {0,  s_ang, -c_ang};  // Chamber Z completes RH system

    // Transformation from chamber to rocket (base orientation)
    T_chamber_to_rocket = transpose(T_rocket_to_chamber);

    momentsPosition = centerPosition - chamberPosition;

//equation

	// requiredThrottle
	// requiredDirection

    // Transform desired thrust direction to chamber frame
    //for j in 1:3 loop
    //  for k in 1:3 loop
    //    desiredDirChamber[j] := desiredDirChamber[j] + 
    //      T_rocket_to_chamber[i, j, k] * desiredThrustDirNorm[k];
    //  end for;
    //end for;
	// requiredChamberDir is requiredDirection in chamber frame
    requiredChamberDirection = T_rocket_to_chamber * requiredDirection;
    //requiredChamberDirection[1] = T_rocket_to_chamber[1, 1] * requiredDirection[1] + T_rocket_to_chamber[1, 2] * requiredDirection[2] + T_rocket_to_chamber[1,3] * requiredDirection[3];
    //requiredChamberDirection[2] = T_rocket_to_chamber[2, 1] * requiredDirection[1] + T_rocket_to_chamber[2, 2] * requiredDirection[2] + T_rocket_to_chamber[2,3] * requiredDirection[3];
    //requiredChamberDirection[3] = T_rocket_to_chamber[3, 1] * requiredDirection[1] + T_rocket_to_chamber[3, 2] * requiredDirection[2] + T_rocket_to_chamber[3,3] * requiredDirection[3];


    // For small angles: tan(beta_z) ≈ -dy/dx and tan(beta_y) ≈ dz/dx
    //if abs(requiredChamberDirection[1]) > 0 then
		gimbalPitch_mm = atan2( requiredChamberDirection[3], requiredChamberDirection[1]); // Pitch
		gimbalYaw_mm   = atan2(-requiredChamberDirection[2], requiredChamberDirection[1]); // Yaw
    //else
	//	gimbalPitch_mm = 0;
	//	gimbalYaw_mm   = 0;
    //end if;
	
    // Apply gimbal limits
	gimbalPitch = max(-gimbalLimit, min(gimbalLimit, gimbalPitch_mm));
	gimbalYaw   = max(-gimbalLimit, min(gimbalLimit, gimbalYaw_mm));
    
    // Calculate actual thrust direction in chamber frame with gimbal angles
    // Thrust vector in chamber frame (along positive X when not gimballed)
    // After gimbal rotations (first yaw about Z, then pitch about Y)
    c_pitch = cos(gimbalPitch);
    s_pitch = sin(gimbalPitch);
    c_yaw   = cos(gimbalYaw);
    s_yaw   = sin(gimbalYaw);
    
    computedChamberDirection[1] =  c_pitch * c_yaw;
    computedChamberDirection[2] = -c_pitch * s_yaw;
    computedChamberDirection[3] =  s_pitch;

	chamberMagnitude = requiredThrottle * maxThrust;
	chamberFuelRate = -chamberMagnitude / (specificImpulse * g0);

    chamberDirection = Vectors.normalize(T_chamber_to_rocket * computedChamberDirection);
    chamberVector = chamberDirection * chamberMagnitude;
    //thrustDirRocketFromChamber[j] = T_chamber_to_rocket[j, k] * chamberThrustDir[k];

    //thrustVecRocket[i, :] := thrustDirRocketFromChamber[i, :] * chamberThrust;

    // Calculate moment contribution from this chamber
    // M = r × F (cross product)
    chamberMoments[1] = momentsPosition[2] * chamberVector[3] - momentsPosition[3] * chamberVector[2];
    chamberMoments[2] = momentsPosition[3] * chamberVector[1] - momentsPosition[1] * chamberVector[3];
    chamberMoments[3] = momentsPosition[1] * chamberVector[2] - momentsPosition[2] * chamberVector[1];

    chamberDeviation = 1.0 - Vectors.normalize(chamberDirection) * Vectors.normalize(requiredDirection);

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

    
end Chamber;
