within Flights;

package MemsImu "MEMS IMU Model with Earth Effects and Coriolis"
  extends Modelica.Icons.Package;

  model IMU "IMU with Complete Navigation Equations"
    
    parameter Real t0 = 0 "Epoch time [s]";
    parameter Real pos0_ECEF[3] = Constants.pos0_ECEF; // Transformations.pos_WGS_to_ECEF(0,0,0);
    parameter Real tau = 0.005 "Filter time constant [s]";
    parameter Real wn = 10 "Natural frequency [rad/s]";
    parameter Real zeta = 0.7 "Damping ratio";


    // inputs
    input  Real acc_DYN[3];     // ускорения в инерциальной СК
    input  Real omega_DYN[3];   // угловые скорости в инерциальной СК

    Real theta;

    Real q_BDY_ECEF[4](start={1,0,0,0});
    Real q_dot[4];

    Real pos_ECI[3](start=Frames.PositionTransforms.ECEF_to_ECI(pos0_ECEF, 0));
    Real vel_ECI[3](start=Frames.VelocityTransforms.ECEF_to_ECI({0,0,0}, pos0_ECEF, 0));

    //Real angle_BDY[3](start={0,0,0});
    //Real angle_ECEF[3];
    //Real angle_ECI[3];

    Real omega_BDY[3];
    Real omega_ECEF[3];
    Real omega_ECI[3];
    Real omega_Filtered[3](start={0,0,0});
    Real x1[3](start={0,0,0});
    Real x2[3](start={0,0,0});
    Real omega_Dot[3];
    Real omega_Dot2[3];

    //Real alpha_ECI[3];
    //Real alpha_BDY[3];
    Real alpha_ECEF[3];
    Real alpha_ECEF2[3];

    Real acc_BDY[3];
    Real acc_ECEF[3];
    Real acc_ECI[3];

    Real vel_BDY[3];
    Real vel_ECEF[3];
    //Real vel_ECI[3](start={0,0,0});

    //Real pos_BDY[3];
    Real pos_ECEF[3];
    //Real pos_ECI[3](start={0,0,0});

  equation
  
    theta = Frames.Constants.omega_earth * (time+t0);

    acc_ECI = acc_DYN;
    // интегрирование ускорений и сил в инерциальной СК
    der(pos_ECI) = vel_ECI;
    der(vel_ECI) = acc_ECI;

    omega_ECEF = Frames.AngularVelocityTransforms.ECI_to_ECEF(omega_DYN, theta);
    omega_ECI  = Frames.AngularVelocityTransforms.ECEF_to_ECI(omega_ECEF, theta);

    for i in 1:3 loop
      tau * der(omega_Filtered[i]) + omega_Filtered[i] = omega_ECEF[i];
      omega_Dot[i] = (omega_ECEF[i] - omega_Filtered[i]) / tau;
    end for;
    alpha_ECEF  = omega_Dot;

    for i in 1:3 loop
      der(x1[i]) = x2[i];
      der(x2[i]) = wn^2 * (omega_ECEF[i] - x1[i]) - 2*zeta*wn*x2[i];
      omega_Dot2[i] = x2[i];
    end for;
    alpha_ECEF2  = omega_Dot2;
  
    // вычисление угловых ускорений и скоростей в инерциальной и земной СК
    //der(omega_ECEF) = alpha_ECEF;
    //der(omega_ECI) = alpha_ECI;
    //alpha_ECEF  = omega_Dot; //Frames.AngularAccelerationTransforms.ECI_to_ECEF(alpha_ECI, theta);
    //omega_ECEF = Frames.AngularVelocityTransforms.ECI_to_ECEF(omega_ECI, theta);
    // вычисление угловых скоростей в связанной СК
    omega_BDY  = Frames.AngularVelocityTransforms.ECEF_to_BODY(omega_ECEF, q_BDY_ECEF);
    // интегрирование кватерниона и соответственно углов
    q_dot = Frames.QuaternionOps.derivative(q_BDY_ECEF, omega_BDY);
    der(q_BDY_ECEF) = q_dot;

    // вычисление ускорений, скоростей и координат в связанной СК
    acc_ECEF = Frames.AccelerationTransforms.ECI_to_ECEF(acc_ECI, vel_ECI, pos_ECI, theta);
    acc_BDY  = Frames.AccelerationTransforms.ECEF_to_BODY(acc_ECEF, vel_ECEF, pos_ECEF, q_BDY_ECEF, omega_ECEF, alpha_ECEF);

    vel_ECEF = Frames.VelocityTransforms.ECI_to_ECEF(vel_ECI, pos_ECI, theta);
    vel_BDY  = Frames.VelocityTransforms.ECEF_to_BODY(vel_ECEF, pos_ECEF, q_BDY_ECEF, omega_ECEF);

    pos_ECEF = Frames.PositionTransforms.ECI_to_ECEF(pos_ECI, theta);
    //pos_BDY  = Frames.PositionTransforms.ECEF_to_BODY(pos_ECEF, q_BDY_ECEF);

  end IMU;

end MemsImu;
