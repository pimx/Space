within Flights;

package MemsImu "MEMS IMU Model with Earth Effects and Coriolis"
  extends Modelica.Icons.Package;

  model IMU "IMU with Complete Navigation Equations"
    
    parameter Real t0 = 0 "Epoch time [s]";
    parameter Real pos0_ECEF[3] = Constants.pos0_ECEF; // Transformations.pos_WGS_to_ECEF(0,0,0);

    // inputs
    input  Real acc_BDY[3];
    input  Real omega_BDY[3];

    Real theta;

    Real q_imu_BDY_ECEF[4](start={1,0,0,0});
    Real q_imu_dot[4];

    Real angle_imu_BDY[3](start={0,0,0});
    Real angle_imu_ECEF[3];
    Real angle_imu_ECI[3];

    Real omega_imu_BDY[3];
    Real omega_imu_ECEF[3];
    Real omega_imu_ECI[3];

    Real alpha_imu_ECI[3];
    Real alpha_imu_BDY[3];
    Real alpha_imu_ECEF[3];

    Real acc_imu_BDY[3];
    Real acc_imu_ECEF[3];
    Real acc_imu_ECI[3];

    Real vel_imu_BDY[3](start={0,0,0});
    Real vel_imu_ECEF[3];
    Real vel_imu_ECI[3](start={0,0,0});

    Real pos_imu_BDY[3](start={0,0,0});
    Real pos_imu_ECEF[3];
    Real pos_imu_ECI[3](start={0,0,0});

  equation
  
    theta = Frames.Constants.omega_earth * (time+t0);

    der(angle_imu_BDY) = omega_imu_BDY;
    angle_imu_ECEF = Frames.AngularVelocityTransforms.BODY_to_ECEF(angle_imu_BDY, q_imu_BDY_ECEF);
    angle_imu_ECI  = Frames.AngularVelocityTransforms.ECEF_to_ECI(angle_imu_ECEF, theta);

    omega_imu_BDY  = omega_BDY;
    omega_imu_ECEF = Frames.AngularVelocityTransforms.BODY_to_ECEF(omega_imu_BDY, q_imu_BDY_ECEF);
    omega_imu_ECI  = Frames.AngularVelocityTransforms.ECEF_to_ECI(omega_imu_ECEF, theta);

    q_imu_dot = Frames.QuaternionOps.derivative(q_imu_BDY_ECEF, omega_imu_ECI);
    der(q_imu_BDY_ECEF) = q_imu_dot;

    der(omega_imu_BDY) = alpha_imu_BDY;
    alpha_imu_ECEF = Frames.AngularAccelerationTransforms.BODY_to_ECEF(alpha_imu_BDY, q_imu_BDY_ECEF);
    alpha_imu_ECI  = Frames.AngularAccelerationTransforms.ECEF_to_ECI(alpha_imu_ECEF, theta);
    //alpha_imu_ECEF = Frames.AngularAccelerationTransforms.ECI_to_ECEF(alpha_imu_ECI, theta);
    //alpha_imu_BDY  = Frames.AngularAccelerationTransforms.ECEF_to_BODY(alpha_imu_ECEF, q_imu_BDY_ECEF, {0,0,0}, {0,0,0});

    acc_imu_BDY    = acc_BDY;
    der(vel_imu_BDY) = acc_imu_BDY;
    der(pos_imu_BDY) = vel_imu_BDY;

    vel_imu_ECEF = Frames.VelocityTransforms.BODY_to_ECEF(vel_imu_BDY, pos_imu_BDY, q_imu_BDY_ECEF, omega_imu_ECEF);
    pos_imu_ECEF = Frames.PositionTransforms.BODY_to_ECEF(pos_imu_BDY, q_imu_BDY_ECEF);

    acc_imu_ECEF   = Frames.AccelerationTransforms.BODY_to_ECEF(acc_imu_BDY, vel_imu_BDY, pos_imu_BDY, q_imu_BDY_ECEF, omega_imu_ECEF, alpha_imu_ECEF);
    acc_imu_ECI    = Frames.AccelerationTransforms.ECEF_to_ECI(acc_imu_ECEF, vel_imu_ECEF, pos_imu_ECEF, theta);

    der(vel_imu_ECI) = acc_imu_ECI;
    der(pos_imu_ECI) = vel_imu_ECI;


//    vel_imu_ECEF = Frames.VelocityTransforms.ECI_to_ECEF(vel_imu_ECI, pos_imu_ECI, theta);
//    vel_imu_BDY  = Frames.VelocityTransforms.ECEF_to_BODY(vel_imu_ECEF, pos_imu_ECEF, q_imu_BDY_ECEF, omega_imu_ECEF);

//    pos_imu_ECEF = Frames.PositionTransforms.ECI_to_ECEF(pos_imu_ECI, theta);
//    pos_imu_BDY  = Frames.PositionTransforms.ECEF_to_BODY(pos_imu_ECEF, q_imu_BDY_ECEF);

    //pos_ECI_imu    = Frames.PositionTransforms.ECEF_to_ECI(pos_ECEF, theta);
    //vel_ECI_imu    = Frames.VelocityTransforms.ECEF_to_ECI(vel_ECEF, pos_ECEF, theta);
    //acc_ECI_imu    = Frames.AccelerationTransforms.ECEF_to_ECI(acc_ECEF, vel_ECEF, pos_ECEF, theta);
    //angle_ECI_imu  = Frames.AngularVelocityTransforms.ECEF_to_ECI(angle_ECEF, theta);
    //omega_ECI_imu  = Frames.AngularVelocityTransforms.ECEF_to_ECI(omega_ECEF, theta);
    //alpha_ECI_imu  = Frames.AngularAccelerationTransforms.ECEF_to_ECI(alpha_ECEF, theta);

  end IMU;

end MemsImu;
