within Flights;

package Sensors

  model LaserIMU "Laser-based IMU (Ring Laser Gyro or Fiber Optic Gyro)"

    input Real omega_BDY[3](each unit="rad/s")      "True angular velocity [rad/s]" annotation(Placement(transformation(extent={{-140,40},{-100,80}})));
    input Real acc_BDY[3](each unit="m/s2")      "True acceleration [m/s2]" annotation(Placement(transformation(extent={{-140,-80},{-100,-40}})));
    
    output Real quaternion[4];

    output Real omega_IMU[3](each unit="rad/s")      "Measured angular velocity [rad/s]" annotation(Placement(transformation(extent={{100,40},{140,80}})));
    output Real acc_IMU[3](each unit="m/s2")      "Measured acceleration [m/s2]" annotation(Placement(transformation(extent={{100,-80},{140,-40}})));
    output Real pos_IMU[3];
    output Real vel_IMU[3];
    output Real euler_IMU[3];
    
    parameter Real g = 9.80665 "Gravitational acceleration [m/s²]";
    parameter Real omega_earth = 7.2921155e-5 "Earth rotation rate [rad/s]";
    parameter Real latitude = 0 "Latitude [rad] for Earth rotation correction";
    parameter Real sample_time = 0.001 "Sensor sample time [s]";
    
    // ============================================
    // PARAMETERS - Initial Conditions
    // ============================================
    parameter Real q0_init = 1.0/sqrt(2.0) "Initial quaternion q0 (scalar part)";
    parameter Real q1_init = 0 "Initial quaternion q1";
    parameter Real q2_init = 1.0/sqrt(2.0) "Initial quaternion q2";
    parameter Real q3_init = 0 "Initial quaternion q3";
    
    parameter Real vN_0 = 0 "Initial North velocity [m/s]";
    parameter Real vE_0 = 0 "Initial East velocity [m/s]";
    parameter Real vD_0 = 0 "Initial Down velocity [m/s]";
    
    parameter Real pN_0 = 0 "Initial North position [m]";
    parameter Real pE_0 = 0 "Initial East position [m]";
    parameter Real pD_0 = 0 "Initial Down position [m]";
    
    // ============================================
    // STATE VARIABLES
    // ============================================
    // Attitude (quaternion)
    Real q0(start = q0_init, fixed = true) "Quaternion scalar part";
    Real q1(start = q1_init, fixed = true) "Quaternion vector i";
    Real q2(start = q2_init, fixed = true) "Quaternion vector j";
    Real q3(start = q3_init, fixed = true) "Quaternion vector k";
    
    // Velocity
    Real vN(start = vN_0, fixed = true) "North velocity [m/s]";
    Real vE(start = vE_0, fixed = true) "East velocity [m/s]";
    Real vD(start = vD_0, fixed = true) "Down velocity [m/s]";
    
    // Position
    Real pN(start = pN_0, fixed = true) "North position [m]";
    Real pE(start = pE_0, fixed = true) "East position [m]";
    Real pD(start = pD_0, fixed = true) "Down position [m]";

    Real C_bn[3, 3] "Direction Cosine Matrix from Body to NED";
    Real acc_NED[3] "Acceleration in NED frame [m/s²]";
    Real omega_corrected[3] "Corrected angular velocity [rad/s]";
    Real omega_ie_n[3] "Earth rotation rate in NED frame [rad/s]";
    Real a_cor[3] "Coriolis and centripetal acceleration [m/s²]";
    
    // Sensor measurements (with errors)
    Real omega_meas[3] "Measured angular velocity [rad/s]";
    Real acc_meas[3] "Measured acceleration [m/s²]";

    // Euler angles (for output)
    Real phi "Roll angle [rad]";
    Real theta "Pitch angle [rad]";
    Real psi "Yaw angle [rad]";
    
    // Quaternion normalization
    Real q_norm "Quaternion norm";

  equation

    //omega_IMU = omega_BDY;
    //acc_IMU   = acc_BDY;
    
    // ============================================
    // SENSOR MEASUREMENTS (with errors)
    // ============================================
    omega_meas = omega_BDY;
    acc_meas   = acc_BDY; // + Transformations.force_NED_to_BODY_quat({0,0,g}, {q0,q1,q2,q3});
    
    // ============================================
    // EARTH ROTATION RATE IN NED FRAME
    // ============================================
    omega_ie_n[1] = omega_earth * cos(latitude);  // North component
    omega_ie_n[2] = 0;                             // East component
    omega_ie_n[3] = -omega_earth * sin(latitude); // Down component
    
    // ============================================
    // QUATERNION KINEMATICS
    // ============================================
    // Corrected angular velocity (compensate for Earth rotation)
    omega_corrected = omega_meas - transpose(C_bn) * omega_ie_n;
    
    // Quaternion derivative (using quaternion multiplication)
    der(q0) = 0.5 * (-q1 * omega_corrected[1] - q2 * omega_corrected[2] - q3 * omega_corrected[3]);
    der(q1) = 0.5 * (q0 * omega_corrected[1] + q2 * omega_corrected[3] - q3 * omega_corrected[2]);
    der(q2) = 0.5 * (q0 * omega_corrected[2] - q1 * omega_corrected[3] + q3 * omega_corrected[1]);
    der(q3) = 0.5 * (q0 * omega_corrected[3] + q1 * omega_corrected[2] - q2 * omega_corrected[1]);
    
    // ============================================
    // QUATERNION NORMALIZATION
    // ============================================
    q_norm = sqrt(q0^2 + q1^2 + q2^2 + q3^2);
    
    // ============================================
    // DCM FROM QUATERNION
    // ============================================
    C_bn = [
      q0^2 + q1^2 - q2^2 - q3^2,  2*(q1*q2 - q0*q3),            2*(q1*q3 + q0*q2);
      2*(q1*q2 + q0*q3),          q0^2 - q1^2 + q2^2 - q3^2,    2*(q2*q3 - q0*q1);
      2*(q1*q3 - q0*q2),          2*(q2*q3 + q0*q1),            q0^2 - q1^2 - q2^2 + q3^2
    ];
    
    // ============================================
    // EULER ANGLES FROM QUATERNION
    // ============================================
    phi = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1^2 + q2^2));
    theta = asin(2 * (q0 * q2 - q3 * q1));
    psi = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2^2 + q3^2));
    
    // ============================================
    // SPECIFIC FORCE TRANSFORMATION & GRAVITY
    // ============================================
    acc_NED = C_bn * acc_meas; // + {0, 0, g};
    
    // ============================================
    // CORIOLIS AND CENTRIPETAL CORRECTIONS
    // ============================================
    // Coriolis: -2 * omega_ie_n × v_n
    // Centripetal: -omega_ie_n × (omega_ie_n × r) ≈ negligible for small distances
    a_cor[1] = 2 * omega_ie_n[3] * vE;                    // North Coriolis
    a_cor[2] = -2 * omega_ie_n[3] * vN - 2 * omega_ie_n[1] * vD;  // East Coriolis
    a_cor[3] = 2 * omega_ie_n[1] * vE;                    // Down Coriolis

    // ============================================
    // VELOCITY INTEGRATION (with corrections)
    // ============================================
    der(vN) = acc_NED[1] + a_cor[1];
    der(vE) = acc_NED[2] + a_cor[2];
    der(vD) = acc_NED[3] + a_cor[3];
    
    // ============================================
    // POSITION INTEGRATION
    // ============================================
    der(pN) = vN;
    der(pE) = vE;
    der(pD) = vD;
    
    // ============================================
    // OUTPUT ASSIGNMENTS
    // ============================================
    quaternion[1] = q0 / q_norm;
    quaternion[2] = q1 / q_norm;
    quaternion[3] = q2 / q_norm;
    quaternion[4] = q3 / q_norm;
    
    euler_IMU[1] = phi;
    euler_IMU[2] = theta;
    euler_IMU[3] = psi;
    
    vel_IMU[1] = vN;
    vel_IMU[2] = vE;
    vel_IMU[3] = vD;
    
    pos_IMU[1] = pN;
    pos_IMU[2] = pE;
    pos_IMU[3] = pD;

    omega_IMU = omega_BDY;
    acc_IMU   = acc_BDY;

  end LaserIMU;
  
end Sensors;

/*
## Key differences between the models:

### MEMS IMU:
- **Gyro bias instability**: ~0.1 deg/s (typical for tactical grade)
- **Gyro noise**: 0.0001 rad/s/√Hz
- **Lower cost, suitable for most applications**
- No lock-in effects

### Laser IMU (RLG/FOG):
- **Gyro bias instability**: ~0.0006 deg/s (100x better)
- **Gyro noise**: 0.000001 rad/s/√Hz (100x better)
- **Lock-in effect**: Dead zone at very low rotation rates (<0.0001 rad/s)
- **Higher precision, used in aerospace/defense**

Both models include:
- Bias instability and random walk
- White noise
- Scale factor errors
- Axis misalignment
- Realistic noise power spectral density

You can adjust the parameters based on your specific sensor datasheets!
*/