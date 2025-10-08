within Platform;

model Control "Guidance, Navigation and Control"
  import SI = Modelica.Units.SI;
  import Modelica.Math.*;
  
    // Control gains
    parameter Real kpZ = 2000 "Vertical position gain";
    parameter Real kdZ = 1000 "Vertical velocity gain";
    parameter Real kpAngle = 1.0 "Attitude angle gain";
    parameter Real kdAngle = 1.3 "Attitude rate gain";
    parameter Real kpHoriz = -0.005 "Horizontal position gain";  // 0.2
    parameter Real kdHoriz = -0.05 "Horizontal position gain";  // 0.2
    parameter Real maxGimbal = 0.262 "Maximum gimbal angle [rad] (15 deg)";
    parameter Real g0 = 9.80665 "Standard gravity [m/s^2]";
    parameter Real maxThrust = 196200 "Maximum thrust [N] (20000 kgf)";

    // Command inputs (from UDP or default)
    Real reqPos[3] "Required position [x, y, z] [m]";
    Real reqEuler[3] "Required orientation [roll, pitch, yaw] [rad]";
    
    Real imu_pos[3] "Position [x, y, z] [m]";
    Real imu_vel[3] "Velocity [vx, vy, vz] [m/s]";
    Real imu_euler[3] "Euler angles [roll, pitch, yaw] [rad]";
    Real imu_omega[3] "Angular velocity [rad/s]";
    
    Real fuelMass "Current fuel mass [kg]";
    Real dryMass "Dry mass [kg]";

    Real gimbalPitchCmd, gimbalYawCmd "Gimbal angles [rad]";
    Real thrustCmd "Commanded thrust before limits [N]";

    Real posErr[3], velErr[3], eulerErr[3], omegaErr[3];
    Real totalMass "Total mass [kg]";

    Real gp1, gp2, gp3, gp4, gps;

equation

    totalMass = dryMass + max(0, fuelMass);

    // Control errors
    posErr = reqPos - imu_pos;
    velErr = -imu_vel;  // Target zero velocity
    eulerErr = reqEuler - imu_euler;
    omegaErr = -imu_omega;  // Target zero angular velocity
    
    
    // Thrust vectoring (attitude and horizontal position)
    gp1 = kpAngle * eulerErr[2];
    gp2 = kdAngle * omegaErr[2];
    gp3 = kpHoriz * posErr[1];
    gp4 = kdHoriz * velErr[1];
    gps = gp1 + gp2 + gp3 + gp4;
    
    gimbalPitchCmd = max(-maxGimbal, min(maxGimbal, -gps));
    //gimbalPitch = max(-maxGimbal, min(maxGimbal, -kpAngle * eulerErr[2] - kdAngle * omegaErr[2] - kpHoriz * posErr[1] - kdHoriz * velErr[1]));

    gimbalYawCmd = max(-maxGimbal, min(maxGimbal,
                    kpAngle * eulerErr[1] + kdAngle * omegaErr[1] - kpHoriz * posErr[2]));
    
    // Thrust magnitude control (vertical)
    thrustCmd = (totalMass * g0 + kpZ * posErr[3] + kdZ * velErr[3]) / maxThrust / cos(gimbalPitchCmd) / cos(gimbalYawCmd);	// 0..1
    //thrustMagnitude = max(0, min(maxThrust, thrustCmd) / cos(gimbalPitch) / cos(gimbalYaw);

    // Thrust vector in body frame
    //thrustBody[1] = thrustMagnitude * sin(gimbalPitch);
    //thrustBody[2] = thrustMagnitude * sin(gimbalYaw);
    //thrustBody[3] = thrustMagnitude * cos(gimbalPitch) * cos(gimbalYaw);

end Control;
