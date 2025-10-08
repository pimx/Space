within Platform;

  // ========================================================================
  // Main Rocket Model
  // ========================================================================

  model Rocket
    "Complete rocket model with position control and UDP communication"
    
    // Parameters
    parameter Real rocketHeight = 20 "Rocket height [m]";
    parameter Real rocketDiameter = 2 "Rocket diameter [m]";
    parameter Real dryMass = 1000 "Dry mass [kg]";
    parameter Real fuelMassInit = 10000 "Initial fuel mass [kg]";
    parameter Real startAltitude = 20 "Starting altitude [m]";
    parameter Real specificImpulse = 300 "Specific impulse [s]";
    parameter Real g0 = 9.80665 "Standard gravity [m/s^2]";
    
    Real thrustMagnitude "Commanded thrust magnitude [N]";
    Real gimbalPitch, gimbalYaw "Gimbal angles [rad]";

    // Aerodynamic parameters
    parameter Real dragCoeff = 0.5 "Drag coefficient";
    parameter Real airDensity = 1.225 "Air density [kg/m^3]";
    parameter Real thrustMomentArm = 0.3 * rocketHeight "Thrust moment arm [m]";
    
    // State variables (these will be integrated)
    Real pos[3](start={0, 0, startAltitude}) "Position [x, y, z] [m]";
    Real vel[3](start={0, 0, 0}) "Velocity [vx, vy, vz] [m/s]";
    Real acc[3](start={0, 0, 0}) "Acceleration [ax, ay, az] [m/s2]";
    Real euler[3](start={0, 0.2, 0}) "Euler angles [roll, pitch, yaw] [rad]";
    Real omega[3](start={0, 0, 0}) "Angular velocity [rad/s]";
    Real fuelMass(start=fuelMassInit) "Current fuel mass [kg]";
    
    // Output variables (computed from states)
    Real totalMass "Total mass [kg]";
    Real thrustVector[3] "Thrust vector in world frame [N]";
    Real thrustBody[3] "Thrust in body frame [N]";
    
  //protected
    Real crossSection = Modelica.Constants.pi * (rocketDiameter/2)^2;
    Real Ixx "Moment of inertia around x [kg*m^2]";
    Real Iyy "Moment of inertia around y [kg*m^2]";
    Real Izz "Moment of inertia around z [kg*m^2]";
    
    Real R11, R12, R13, R21, R22, R23, R31, R32, R33 "Rotation matrix elements";
    Real c1, c2, c3, s1, s2, s3 "Cosines and sines of Euler angles";
    
    
  equation
    // Mass calculation
    totalMass = dryMass + max(0, fuelMass);
    
    // Moments of inertia (solid cylinder)
    Ixx = totalMass * (3*(rocketDiameter/2)^2 + rocketHeight^2) / 12;
    Iyy = Ixx;
    Izz = totalMass * (rocketDiameter/2)^2 / 2;
    
    // Rotation matrix from body to world frame (Z-Y-X Euler)
    c1 = cos(euler[1]); s1 = sin(euler[1]);
    c2 = cos(euler[2]); s2 = sin(euler[2]);
    c3 = cos(euler[3]); s3 = sin(euler[3]);
    
    R11 = c2*c3;
    R12 = c2*s3;
    R13 = -s2;
    R21 = s1*s2*c3 - c1*s3;
    R22 = s1*s2*s3 + c1*c3;
    R23 = s1*c2;
    R31 = c1*s2*c3 + s1*s3;
    R32 = c1*s2*s3 - s1*c3;
    R33 = c1*c2;
    
    // Thrust vector in body frame
    thrustBody[1] = thrustMagnitude * sin(gimbalPitch);
    thrustBody[2] = thrustMagnitude * sin(gimbalYaw);
    thrustBody[3] = thrustMagnitude * cos(gimbalPitch) * cos(gimbalYaw);
    
    // Transform thrust to world frame
    thrustVector[1] = R11*thrustBody[1] + R12*thrustBody[2] + R13*thrustBody[3];
    thrustVector[2] = R21*thrustBody[1] + R22*thrustBody[2] + R23*thrustBody[3];
    thrustVector[3] = R31*thrustBody[1] + R32*thrustBody[2] + R33*thrustBody[3];
    
    // Translational dynamics: F = ma
    der(pos[1]) = vel[1];
    der(pos[2]) = vel[2];
    der(pos[3]) = vel[3];
    
    acc[1] = (thrustVector[1] - 0.5*airDensity*crossSection*dragCoeff*abs(vel[1])*vel[1]) / totalMass;
    acc[2] = (thrustVector[2] - 0.5*airDensity*crossSection*dragCoeff*abs(vel[2])*vel[2]) / totalMass;
    acc[3] = (thrustVector[3] - totalMass*g0 - 0.5*airDensity*crossSection*dragCoeff*abs(vel[3])*vel[3]) / totalMass;
    
    der(vel[1]) = acc[1];
    der(vel[2]) = acc[2];
    der(vel[3]) = acc[3];

    // Rotational kinematics (Euler angle rates)
    der(euler[1]) = omega[1] + omega[2]*s1*s2/c2 + omega[3]*c1*s2/c2;
    der(euler[2]) = omega[2]*c1 - omega[3]*s1;
    der(euler[3]) = omega[2]*s1/c2 + omega[3]*c1/c2;
    
    // Rotational dynamics (moments)
    der(omega[1]) = (-thrustBody[2]*thrustMomentArm - omega[2]*omega[3]*(Izz - Iyy)) / Ixx;
    der(omega[2]) = (-thrustBody[1]*thrustMomentArm - omega[3]*omega[1]*(Ixx - Izz)) / Iyy;
    der(omega[3]) = 0 / Izz;  // No yaw moment from thrust
    
    // Fuel consumption
    der(fuelMass) = if thrustMagnitude > 0 then -thrustMagnitude / (specificImpulse * g0) else 0;
    
    // Ground contact
    when pos[3] < 0 then
      reinit(pos[3], 0);
      reinit(vel[3], max(0, vel[3]));  // Only stop downward velocity
    end when;
    
    // Fuel depletion
    when fuelMass < 0 then
      reinit(fuelMass, 0);
    end when;
    
    annotation(
      experiment(StartTime=0, StopTime=100, Tolerance=1e-6, Interval=0.01),
      Documentation(info="<html>
        <p>Simplified rocket vertical landing model.</p>
        <p>State variables (12):</p>
        <ul>
          <li>pos[3] - Position</li>
          <li>vel[3] - Velocity</li>
          <li>euler[3] - Euler angles</li>
          <li>omega[3] - Angular velocity</li>
          <li>fuelMass - Fuel mass</li>
        </ul>
        </html>"));
  end Rocket;
