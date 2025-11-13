package Rocket6DOF "6-DOF Rocket Model with ECI integration and ECEF outputs"

  // ============================================================================
  // CONSTANTS AND PARAMETERS
  // ============================================================================
  
  package Constants

    // Earth hframeters
    constant Real mu               = 3.986004418e14   "Earth gravitational parameter [m^3/s^2]";
    constant Real omega_earth      = 7.2921159e-5     "Earth rotation rate [rad/s]";
    constant Real R_earth          = 6378137.0        "WGS84 equatorial radius [m]";
    constant Real f_earth          = 1/298.257223563  "WGS84 flattening";
    constant Real e_earth          = sqrt(2*f_earth - f_earth^2) "WGS84 eccentricity";
    constant Real g0               = 9.80665          "Standard gravity [m/s^2]";
    constant Real gravity          = g0               "Gravity [m/s²]";

    // mass and inertial parameters
    constant Real S_ref            = 7.0              "Reference area [m^2]";
    constant Real L_ref            = 30.0             "Reference length [m]";
    constant Real D_ref            = 3.0              "Reference diameter [m]";
    constant Real m_dry            = 24000.0          "Dry mass [kg]";
    constant Real m_fuel_initial   = 40000.0          "Initial fuel mass [kg]";
    constant Real x_cm_dry         = 20.0             "Dry CoM position from nose [m]";
    constant Real x_fuel_empty     = 20               "Empty fuel tank CoM from nose [m]";
    constant Real x_fuel_full      = 20               "Full fuel tank CoM from nose [m]";
    constant Real I_dry[3,3]       = [30000, 0, 0; 0, 150000, 0; 0, 0, 150000] "Dry inertia tensor [kg·m^2]";
    constant Real I_fuel_full[3,3] = [50000, 0, 0; 0, 250000, 0; 0, 0, 250000] "Full fuel inertia tensor [kg·m^2]";
    constant Real vehicle_mass     = 30000            "Rocket mass [kg]";

    // Vehicle parameters
    constant Real max_gimbal_angle = 0.15 "Maximum gimbal angle [rad]";
    
    // Altitude control (outer loop - slow)
    constant Real Kp_alt = 0.8 "Altitude proportional gain";
    constant Real Ki_alt = 0.05 "Altitude integral gain";
    constant Real Kd_alt = 2.0 "Altitude derivative gain";
    constant Real alt_deadband = 0.1 "Altitude deadband [m]";
    
    // Vertical velocity control (middle loop)
    constant Real Kp_vel_z = 0.15 "Vertical velocity proportional gain";
    constant Real Ki_vel_z = 0.02 "Vertical velocity integral gain";
    constant Real max_vel_z_setpoint = 2.0 "Max vertical velocity setpoint [m/s]";
    
    // Horizontal position control (outer loop - slow)
    constant Real Kp_pos_xy = 0.05 "Horizontal position proportional gain";
    constant Real Ki_pos_xy = 0.002 "Horizontal position integral gain";
    constant Real Kd_pos_xy = 0.3 "Horizontal position derivative gain";
    constant Real pos_xy_deadband = 0.5 "Position deadband [m]";
    
    // Horizontal velocity control (middle loop)
    constant Real Kp_vel_xy = 0.8 "Horizontal velocity proportional gain";
    constant Real Ki_vel_xy = 0.05 "Horizontal velocity integral gain";
    constant Real max_vel_xy_setpoint = 1.5 "Max horizontal velocity setpoint [m/s]";
    constant Real max_accel_xy = 1.5 "Max horizontal acceleration [m/s²]";
    
    // Attitude control (inner loop - fast)
    constant Real Kp_att = 3.0 "Attitude proportional gain";
    constant Real Ki_att = 0.1 "Attitude integral gain";
    constant Real Kd_att = 0.8 "Attitude derivative gain";
    constant Real max_attitude = 0.19 "Maximum attitude angle [rad] - slightly less than limit";
    
    // Attitude rate control (innermost loop - very fast)
    constant Real Kp_rate = 1.5 "Attitude rate proportional gain";
    constant Real Ki_rate = 0.3 "Attitude rate integral gain";
    constant Real max_rate = 0.2 "Maximum attitude rate [rad/s]";
    
    // Anti-windup limits
    constant Real max_alt_integral = 50.0 "Max altitude integral term";
    constant Real max_vel_integral = 10.0 "Max velocity integral term";
    constant Real max_pos_integral = 20.0 "Max position integral term";
    constant Real max_att_integral = 0.5 "Max attitude integral term";

  end Constants;

  // ============================================================================
  // COORDINATE TRANSFORMATION FUNCTIONS
  // ============================================================================
  
  package Transformations
    
    function quaternionMultiply "Multiply two quaternions: q_result = q1 * q2"
      input  Real q1[4] "First quaternion [w,x,y,z]";
      input  Real q2[4] "Second quaternion [w,x,y,z]";
      output Real q_result[4] "Result quaternion";
    algorithm
      q_result[1] := q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] - q1[4]*q2[4];
      q_result[2] := q1[1]*q2[2] + q1[2]*q2[1] + q1[3]*q2[4] - q1[4]*q2[3];
      q_result[3] := q1[1]*q2[3] - q1[2]*q2[4] + q1[3]*q2[1] + q1[4]*q2[2];
      q_result[4] := q1[1]*q2[4] + q1[2]*q2[3] - q1[3]*q2[2] + q1[4]*q2[1];
    end quaternionMultiply;
    
    function quaternionConjugate "Conjugate of quaternion"
      input  Real q[4] "Input quaternion [w,x,y,z]";
      output Real q_conj[4] "Conjugate quaternion";
    algorithm
      q_conj := {q[1], -q[2], -q[3], -q[4]};
    end quaternionConjugate;
    
    function quaternionNormalize "Normalize quaternion"
      input  Real q[4] "Input quaternion";
      output Real q_norm[4] "Normalized quaternion";
    protected
      Real norm;
    algorithm
      norm := sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2);
      q_norm := q / norm;
    end quaternionNormalize;
    
    function quaternionToRotationMatrix "Convert quaternion to rotation matrix"
      input  Real q[4] "Quaternion [w,x,y,z]";
      output Real R[3,3] "Rotation matrix";
    protected
      Real w, x, y, z;
    algorithm
      w := q[1]; x := q[2]; y := q[3]; z := q[4];
      
      R[1,1] := 1 - 2*(y^2 + z^2);
      R[1,2] := 2*(x*y - w*z);
      R[1,3] := 2*(x*z + w*y);
      
      R[2,1] := 2*(x*y + w*z);
      R[2,2] := 1 - 2*(x^2 + z^2);
      R[2,3] := 2*(y*z - w*x);
      
      R[3,1] := 2*(x*z - w*y);
      R[3,2] := 2*(y*z + w*x);
      R[3,3] := 1 - 2*(x^2 + y^2);
    end quaternionToRotationMatrix;
    
    function quaternionToEuler "Convert quaternion to Euler angles (ZYX sequence)"
      input  Real q[4] "Quaternion [w,x,y,z]";
      output Real euler[3] "Euler angles [yaw, pitch, roll] in radians";
    protected
      Real w, x, y, z;
      Real sinp;
    algorithm
      w := q[1]; x := q[2]; y := q[3]; z := q[4];
      
      // Roll (x-axis rotation)
      euler[3] := atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
      
      // Pitch (y-axis rotation)
      sinp := 2*(w*y - z*x);
      if abs(sinp) >= 1 then
        euler[2] := sign(sinp) * Modelica.Constants.pi/2; // Use 90 degrees if out of range
      else
        euler[2] := asin(sinp);
      end if;
      
      // Yaw (z-axis rotation)
      euler[1] := atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
    end quaternionToEuler;
    
    function ECItoECEF "Transform position from ECI to ECEF"
      input  Real r_ECI[3] "Position in ECI [m]";
      input  Real t "Time since reference epoch [s]";
      output Real r_ECEF[3] "Position in ECEF [m]";
    protected
      Real theta "Earth rotation angle";
    algorithm
      theta := Constants.omega_earth * t;
      r_ECEF[1] := cos(theta)*r_ECI[1] + sin(theta)*r_ECI[2];
      r_ECEF[2] := -sin(theta)*r_ECI[1] + cos(theta)*r_ECI[2];
      r_ECEF[3] := r_ECI[3];
    end ECItoECEF;
    
    function ECEFtoECI "Transform position from ECEF to ECI"
      input  Real r_ECEF[3] "Position in ECEF [m]";
      input  Real t "Time since reference epoch [s]";
      output Real r_ECI[3] "Position in ECI [m]";
    protected
      Real theta "Earth rotation angle";
    algorithm
      theta := Constants.omega_earth * t;
      r_ECI[1] := cos(theta)*r_ECEF[1] - sin(theta)*r_ECEF[2];
      r_ECI[2] := sin(theta)*r_ECEF[1] + cos(theta)*r_ECEF[2];
      r_ECI[3] := r_ECEF[3];
    end ECEFtoECI;
    
    function velocityECItoECEF "Transform velocity from ECI to ECEF"
      input  Real r_ECI[3] "Position in ECI [m]";
      input  Real v_ECI[3] "Velocity in ECI [m/s]";
      input  Real t "Time since reference epoch [s]";
      output Real v_ECEF[3] "Velocity in ECEF [m/s]";
    protected
      Real theta "Earth rotation angle";
      Real r_ECEF[3];
    algorithm
      theta := Constants.omega_earth * t;
      
      // Transform velocity
      v_ECEF[1] := cos(theta)*v_ECI[1] + sin(theta)*v_ECI[2];
      v_ECEF[2] := -sin(theta)*v_ECI[1] + cos(theta)*v_ECI[2];
      v_ECEF[3] := v_ECI[3];
      
      // Subtract Earth rotation component
      r_ECEF := ECItoECEF(r_ECI, t);
      v_ECEF[1] := v_ECEF[1] + Constants.omega_earth * r_ECEF[2];
      v_ECEF[2] := v_ECEF[2] - Constants.omega_earth * r_ECEF[1];
    end velocityECItoECEF;
    
    function velocityECItoBodyWoEarthRotation
      "Transform velocity from ECI to body frame, removing Earth rotation component"
      input  Real v_ECI[3] "Velocity vector in ECI frame [vx, vy, vz] (m/s)";
      input  Real r_ECI[3] "Position vector in ECI frame [x, y, z] (m)";
      input  Real q[4] "Quaternion representing body orientation [q0, q1, q2, q3] (scalar-first)";
      output Real v_body[3] "Velocity vector in body frame without Earth rotation (m/s)";

    protected
      Real R[3,3] "Rotation matrix from ECI to body";
      Real q0, q1, q2, q3;
      Real q_norm;
      //Real omega_earth = 7.2921159e-5 "Earth's angular velocity (rad/s)";
      Real v_earth_rotation[3] "Velocity due to Earth's rotation";
      Real v_ECI_corrected[3] "ECI velocity with Earth rotation removed";
      
    algorithm
      // Normalize quaternion
      q_norm := sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2);
      q0 := q[1]/q_norm;
      q1 := q[2]/q_norm;
      q2 := q[3]/q_norm;
      q3 := q[4]/q_norm;
      
      // Construct rotation matrix from ECI to body (R_EB)
      R[1,1] := q0^2 + q1^2 - q2^2 - q3^2;
      R[1,2] := 2*(q1*q2 + q0*q3);
      R[1,3] := 2*(q1*q3 - q0*q2);
      
      R[2,1] := 2*(q1*q2 - q0*q3);
      R[2,2] := q0^2 - q1^2 + q2^2 - q3^2;
      R[2,3] := 2*(q2*q3 + q0*q1);
      
      R[3,1] := 2*(q1*q3 + q0*q2);
      R[3,2] := 2*(q2*q3 - q0*q1);
      R[3,3] := q0^2 - q1^2 - q2^2 + q3^2;
      
      // Calculate velocity due to Earth's rotation: v_rot = omega_Earth × r_ECI
      // omega_Earth vector is [0, 0, omega_earth] in ECI frame (rotation about Z-axis)
      // Cross product: [0, 0, omega] × [x, y, z] = [-omega*y, omega*x, 0]
      v_earth_rotation[1] := -Constants.omega_earth * r_ECI[2];
      v_earth_rotation[2] :=  Constants.omega_earth * r_ECI[1];
      v_earth_rotation[3] :=  0.0;
      
      // Remove Earth rotation component from ECI velocity
      v_ECI_corrected[1] := v_ECI[1] - v_earth_rotation[1];
      v_ECI_corrected[2] := v_ECI[2] - v_earth_rotation[2];
      v_ECI_corrected[3] := v_ECI[3] - v_earth_rotation[3];
      
      // Transform corrected velocity to body frame
      v_body := R * v_ECI_corrected;
      
      annotation(Documentation(info="<html>
    <p>Transforms velocity vector from Earth-Centered Inertial (ECI) frame to body frame,
    removing the velocity component due to Earth's rotation.</p>
    <p><b>Inputs:</b></p>
    <ul>
    <li>v_ECI: Velocity in ECI frame (includes Earth rotation)</li>
    <li>r_ECI: Position in ECI frame (needed to calculate rotation component)</li>
    <li>q: Quaternion [q0, q1, q2, q3] representing rotation from ECI to body frame</li>
    </ul>
    <p><b>Output:</b></p>
    <ul>
    <li>v_body: Velocity in body frame (relative to Earth surface, no rotation component)</li>
    </ul>
    <p>Earth's angular velocity: ω = 7.2921159×10⁻⁵ rad/s</p>
    <p>The function computes: v_body = R_EB * (v_ECI - ω_Earth × r_ECI)</p>
    </html>"));
    end velocityECItoBodyWoEarthRotation;

    function forceECItoECEF "Transform position from ECI to ECEF"
      input  Real r_ECI[3] "Force in ECI [m]";
      input  Real t "Time since reference epoch [s]";
      output Real r_ECEF[3] "Force in ECEF [m]";
    protected
      Real theta "Earth rotation angle";
    algorithm
      theta := Constants.omega_earth * t;
      r_ECEF[1] := cos(theta)*r_ECI[1] + sin(theta)*r_ECI[2];
      r_ECEF[2] := -sin(theta)*r_ECI[1] + cos(theta)*r_ECI[2];
      r_ECEF[3] := r_ECI[3];
    end forceECItoECEF;
    
    function ECEFtoGeodetic "Convert ECEF to geodetic coordinates (WGS84)"
      input  Real r_ECEF[3] "Position in ECEF [m]";
      output Real lat "Latitude [rad]";
      output Real lon "Longitude [rad]";
      output Real alt "Altitude above ellipsoid [m]";
    protected
      Real x, y, z;
      Real p, theta, N;
      constant Real a = Constants.R_earth;
      constant Real e2 = Constants.e_earth^2;
      constant Real b = a*(1 - Constants.f_earth);
    algorithm
      x := r_ECEF[1];
      y := r_ECEF[2];
      z := r_ECEF[3];
      
      p := sqrt(x^2 + y^2);
      theta := atan2(z*a, p*b);
      
      lon := atan2(y, x);
      lat := atan2(z + (e2/(1-e2))*b*sin(theta)^3, p - e2*a*cos(theta)^3);
      
      N := a / sqrt(1 - e2*sin(lat)^2);
      alt := p/cos(lat) - N;
    end ECEFtoGeodetic;
    
  end Transformations;

  // ============================================================================
  // ATMOSPHERE MODEL
  // ============================================================================
  
  package Atmosphere
    
    function density "Simple exponential atmosphere model"
      input  Real altitude "Altitude above sea level [m]";
      output Real rho "Air density [kg/m^3]";
    protected
      constant Real rho0 = 1.225 "Sea level density";
      constant Real H = 8500.0 "Scale height [m]";
    algorithm
      if altitude < 0 then
        rho := rho0;
      else
        rho := rho0 * exp(-altitude/H);
      end if;
    end density;
    
    function speedOfSound "Speed of sound"
      input  Real altitude "Altitude [m]";
      output Real a "Speed of sound [m/s]";
    protected
      constant Real a0 = 340.3 "Sea level speed of sound";
      constant Real T0 = 288.15 "Sea level temperature [K]";
      Real T "Temperature [K]";
    algorithm
      T := max(216.65, T0 - 0.0065*altitude); // Simple linear model
      a := sqrt(1.4 * 287.05 * T);
    end speedOfSound;
    
  end Atmosphere;

  // ============================================================================
  // AERODYNAMICS MODEL
  // ============================================================================
  
  model AerodynamicsModel "6-DOF aerodynamic forces and moments"
    
    // Geometric parameters
    parameter Real S_ref = Constants.S_ref "Reference area [m^2]";
    parameter Real L_ref = Constants.L_ref "Reference length [m]";
    parameter Real D_ref = Constants.D_ref "Reference diameter [m]";
    
    // Inputs
    input Real v_body[3] "Velocity in body frame [m/s]";
    input Real omega_body[3] "Angular velocity in body frame [rad/s]";
    input Real altitude "Altitude [m]";
    input Real q_dyn "Dynamic pressure [Pa]";
    
    // Outputs
    output Real F_aero[3] "Aerodynamic force in body frame [N]";
    output Real M_aero[3] "Aerodynamic moment in body frame [N·m]";
    
    Real F_aerox[3] "Aerodynamic force in body frame [N]";
    Real M_aerox[3] "Aerodynamic moment in body frame [N·m]";
  // Aerodynamic angles
    Real alpha "Angle of attack [rad]";
    Real beta "Sideslip angle [rad]";
    Real V_mag "Velocity magnitude [m/s]";
    Real Mach "Mach number";
    
    // Aerodynamic coefficients
    Real CD "Drag coefficient";
    Real CL "Lift coefficient";
    Real CY "Side force coefficient";
    Real Cl "Rolling moment coefficient";
    Real Cm "Pitching moment coefficient";
    Real Cn "Yawing moment coefficient";
    
  equation
    // Velocity magnitude
    V_mag = sqrt(v_body[1]^2 + v_body[2]^2 + v_body[3]^2);
    
    // Aerodynamic angles
    alpha = atan2(v_body[3], v_body[1]);
    beta = asin(v_body[2] / max(V_mag, 1e-6));
    
    // Mach number
    Mach = V_mag / Atmosphere.speedOfSound(altitude);
    
    // ========== PLACEHOLDER AERODYNAMIC COEFFICIENTS ==========
    // These should be replaced with actual aerodynamic data
    
    // Drag coefficient (example: function of Mach and alpha)
    CD = 0.15 + 0.3*alpha^2 + 0.1*Mach;
    
    // Lift coefficient
    CL = 1.5*sin(2*alpha);
    
    // Side force coefficient
    CY = -0.5*beta;
    
    // Rolling moment coefficient (due to sideslip and roll rate)
    Cl = -0.1*beta - 0.05*omega_body[1]*D_ref/max(V_mag, 1.0);
    
    // Pitching moment coefficient (due to angle of attack and pitch rate)
    Cm = -0.5*alpha - 0.1*omega_body[2]*L_ref/max(V_mag, 1.0);
    
    // Yawing moment coefficient (due to sideslip and yaw rate)
    Cn = 0.1*beta - 0.05*omega_body[3]*L_ref/max(V_mag, 1.0);
    
    // ========== END PLACEHOLDER COEFFICIENTS ==========
    
    // Aerodynamic forces in body frame
    // Convention: X-forward (drag opposes), Y-right (side force), Z-down (lift opposes)
    F_aerox[1] =  -q_dyn*S_ref*CD; // Drag (negative X)
    F_aerox[2] =   q_dyn*S_ref*CY;  // Side force
    F_aerox[3] =  -q_dyn*S_ref*CL; // Lift (negative Z for positive alpha)

    F_aero = F_aerox;
  
    //F_aero[1] = 0; // -q_dyn*S_ref*CD; // Drag (negative X)
    //F_aero[2] = 0; //  q_dyn*S_ref*CY;  // Side force
    //F_aero[3] = 0; // -q_dyn*S_ref*CL; // Lift (negative Z for positive alpha)
    
    // Aerodynamic moments in body frame
    M_aerox[1] =   q_dyn*S_ref*D_ref*Cl; // Rolling moment
    M_aerox[2] =   q_dyn*S_ref*L_ref*Cm; // Pitching moment
    M_aerox[3] =   q_dyn*S_ref*L_ref*Cn; // Yawing moment
  
    M_aero = M_aerox;

    //M_aero[1] = 0; //  q_dyn*S_ref*D_ref*Cl; // Rolling moment
    //M_aero[2] = 0; //  q_dyn*S_ref*L_ref*Cm; // Pitching moment
    //M_aero[3] = 0; //  q_dyn*S_ref*L_ref*Cn; // Yawing moment
    
  end AerodynamicsModel;

  // ============================================================================
  // MASS PROPERTIES MODEL
  // ============================================================================
  
  model MassPropertiesModel "Variable mass properties with fuel depletion"
    
    // Structural parameters
    parameter Real m_dry            = Constants.m_dry          "Dry mass [kg]";
    parameter Real m_fuel_initial   = Constants.m_fuel_initial "Initial fuel mass [kg]";
    parameter Real x_cm_dry         = Constants.x_cm_dry       "Dry CoM position from nose [m]";
    parameter Real x_fuel_empty     = Constants.x_fuel_empty   "Empty fuel tank CoM from nose [m]";
    parameter Real x_fuel_full      = Constants.x_fuel_full    "Full fuel tank CoM from nose [m]";
    parameter Real I_dry[3,3]       = Constants.I_dry          "Dry inertia tensor [kg·m^2]";
    parameter Real I_fuel_full[3,3] = Constants.I_fuel_full    "Full fuel inertia tensor [kg·m^2]";
    
    // Inputs
    input Real m_dot_fuel "Fuel mass flow rate [kg/s]";
    
    // States
    Real m_fuel(start=m_fuel_initial, fixed=true) "Current fuel mass [kg]";
    
    // Outputs
    output Real m_total "Total mass [kg]";
    output Real x_cm "Center of mass position from nose [m]";
    output Real I_body[3,3] "Inertia tensor at CoM in body frame [kg·m^2]";
    
  protected
    Real fuel_fraction "Fuel fraction remaining";
    Real x_fuel "Current fuel CoM position [m]";
    Real I_fuel[3,3] "Current fuel inertia tensor [kg·m^2]";
    Real dx_dry "Distance from total CoM to dry CoM [m]";
    Real dx_fuel "Distance from total CoM to fuel CoM [m]";
    
  equation
    // Fuel depletion
    der(m_fuel) = -m_dot_fuel;
    
    // Total mass
    m_total = m_dry + max(0, m_fuel);
    
    // Fuel fraction
    fuel_fraction = max(0, m_fuel) / m_fuel_initial;
    
    // Fuel CoM shifts as fuel depletes (linear interpolation)
    x_fuel = x_fuel_full*fuel_fraction + x_fuel_empty*(1 - fuel_fraction);
    
    // Total CoM position
    x_cm = (m_dry*x_cm_dry + max(0, m_fuel)*x_fuel) / m_total;
    
    // Distances from total CoM to component CoMs
    dx_dry = x_cm_dry - x_cm;
    dx_fuel = x_fuel - x_cm;
    
    // Scale fuel inertia with mass
    I_fuel = I_fuel_full * fuel_fraction;
    
    // Parallel axis theorem to get total inertia at combined CoM
    // I_total = I_dry + m_dry*[dy^2+dz^2  -dx*dy  -dx*dz; -dx*dy  dx^2+dz^2  -dy*dz; -dx*dz  -dy*dz  dx^2+dy^2]
    // For symmetric rocket (dy=dz=0), only X-offset matters
    
    I_body[1,1] = I_dry[1,1] + I_fuel[1,1];
    I_body[1,2] = I_dry[1,2] + I_fuel[1,2];
    I_body[1,3] = I_dry[1,3] + I_fuel[1,3];
    
    I_body[2,1] = I_body[1,2];
    I_body[2,2] = I_dry[2,2] + m_dry*dx_dry^2 + I_fuel[2,2] + max(0,m_fuel)*dx_fuel^2;
    I_body[2,3] = I_dry[2,3] + I_fuel[2,3];
    
    I_body[3,1] = I_body[1,3];
    I_body[3,2] = I_body[2,3];
    I_body[3,3] = I_dry[3,3] + m_dry*dx_dry^2 + I_fuel[3,3] + max(0,m_fuel)*dx_fuel^2;
    
  end MassPropertiesModel;

  // ============================================================================
  // THRUST MODEL
  // ============================================================================
  
  model ThrustModel "Thrust magnitude and direction with dynamics"
    
    // Engine parameters
    //parameter Real T_max = 700000.0 "Maximum thrust [N]";
    parameter Real Isp = 325.0 "Specific impulse [s]";
    parameter Real tau_thrust = 0.1 "Thrust response time constant [s]";
    parameter Real tau_direction = 0.05 "Direction response time constant [s]";
    
    // Commanded inputs
    input Real T_cmd "Commanded thrust magnitude [N]";
    input Real u_thrust_cmd[3] "Commanded thrust direction (unit vector in body frame)";
    
    // Actual thrust states (with dynamics)
    Real T_actual(start=0, fixed=true) "Actual thrust magnitude [N]";
    Real u_thrust[3](each start=0, each fixed=true) "Actual thrust direction (unit vector)";
    
    // Outputs
    output Real F_thrust[3] "Thrust force in body frame [N]";
    output Real M_thrust[3] "Thrust moment in body frame [N·m]";
    output Real m_dot "Propellant mass flow rate [kg/s]";
    
    // Thrust offset from CoM
    parameter Real x_thrust = -25.0 "Thrust application point from nose [m]";
    input Real x_cm "Center of mass position from nose [m]";
    
  protected
    Real u_cmd_norm "Normalization of commanded direction";
    Real dx_thrust "Thrust lever arm from CoM [m]";
    
  equation
    // Thrust magnitude dynamics (first-order lag)
    tau_thrust * der(T_actual) = T_cmd - T_actual;
    
    // Normalize commanded direction
    u_cmd_norm = sqrt(u_thrust_cmd[1]^2 + u_thrust_cmd[2]^2 + u_thrust_cmd[3]^2);
    
    // Thrust direction dynamics (first-order lag on each component)
    if u_cmd_norm > 1e-6 then
      tau_direction * der(u_thrust[1]) = u_thrust_cmd[1]/u_cmd_norm - u_thrust[1];
      tau_direction * der(u_thrust[2]) = u_thrust_cmd[2]/u_cmd_norm - u_thrust[2];
      tau_direction * der(u_thrust[3]) = u_thrust_cmd[3]/u_cmd_norm - u_thrust[3];
    else
      tau_direction * der(u_thrust[1]) = -u_thrust[1];
      tau_direction * der(u_thrust[2]) = -u_thrust[2];
      tau_direction * der(u_thrust[3]) = 1.0 - u_thrust[3]; // Default to +X direction
    end if;
    
    // Thrust force in body frame
    F_thrust = T_actual * u_thrust;
    
    // Thrust lever arm from CoM
    dx_thrust = x_thrust - x_cm;
    
    // Thrust moment (cross product: r × F, where r is in body frame)
    // r = [dx_thrust, 0, 0], F = T_actual * u_thrust
    M_thrust[1] = 0; // No roll moment from axial offset
    M_thrust[2] = -dx_thrust * F_thrust[3]; // Pitch moment from Z-force
    M_thrust[3] = dx_thrust * F_thrust[2];  // Yaw moment from Y-force
    
    // Mass flow rate
    m_dot = T_actual / (Isp * Constants.g0);
    
  end ThrustModel;

  // ============================================================================
  // MAIN ROCKET MODEL
  // ============================================================================
  
  model Rocket "Complete 6-DOF rocket with ECI integration and ECEF outputs"
    
    // ========== PARAMETERS ==========
    
    // Initial conditions in ECI frame
    parameter Real r0_ECI[3]      = {Constants.R_earth + 1, 0, 0};
    parameter Real v0_ECI[3]      = {0, 465.1, 0}; // Orbital velocity + some vertical
    parameter Real q0[4]          = {1, 0, 0, 0} "Initial attitude quaternion (ECI to Body)";
    parameter Real omega0_body[3] = {0, 0, 0} "Initial angular velocity body frame [rad/s]";
    
    // Reference time for ECEF transformations
    parameter Real t0 = 0 "Reference epoch time [s]";
    
    // ========== COMPONENT MODELS ==========
    
    MassPropertiesModel mass_model;
    ThrustModel thrust_model;
    AerodynamicsModel aero_model;
    
    // ========== STATE VARIABLES (ECI frame) ==========
    
    // Position and velocity in ECI
    Real r_ECI[3](start=r0_ECI, each fixed=true) "Position in ECI [m]";
    Real v_ECI[3](start=v0_ECI, each fixed=true) "Velocity in ECI [m/s]";
    
    // Attitude quaternion (ECI to Body transformation)
    Real q[4](start=q0, each fixed=true) "Quaternion [w,x,y,z] (ECI to Body)";
    
    // Angular velocity in body frame
    Real omega_body[3](start=omega0_body, each fixed=true) "Angular velocity body [rad/s]";
    
    // ========== INTERMEDIATE VARIABLES ==========
    
    Real r_mag "Position magnitude [m]";
    Real altitude "Altitude above sea level [m]";
    Real rho "Air density [kg/m^3]";
    Real q_dyn "Dynamic pressure [Pa]";
    
    // Rotation matrices
    Real R_ECI_to_Body[3,3] "Rotation matrix from ECI to Body";
    Real R_Body_to_ECI[3,3] "Rotation matrix from Body to ECI";
    
    // Velocity in body frame
    Real v_body[3] "Velocity in body frame [m/s]";
    Real v_bodz[3] "Velocity in body frame [m/s]";
    Real v_bodx[3] "Velocity in body frame [m/s]";

    
    // Forces in ECEF frame
    Real F_gravity_ECEF[3] "Gravity force in ECI [N]";
    Real F_thrust_ECEF[3] "Thrust force in ECI [N]";
    Real F_aero_ECEF[3] "Aerodynamic force in ECI [N]";
    Real F_total_ECEF[3] "Total force in ECI [N]";

    // Forces in ECI frame
    Real F_gravity_ECI[3] "Gravity force in ECI [N]";
    Real F_thrust_ECI[3] "Thrust force in ECI [N]";
    Real F_aero_ECI[3] "Aerodynamic force in ECI [N]";
    Real F_total_ECI[3] "Total force in ECI [N]";
    
    // Moments in body frame
    Real M_total_body[3] "Total moment in body frame [N·m]";
    
    // Quaternion derivative helper
    Real q_dot_omega[4] "Quaternion derivative from angular velocity";
    
    // Inertia matrix inverse (for angular acceleration)
    Real I_inv[3,3] "Inverse of inertia tensor";
    
    // ========== OUTPUT VARIABLES (ECEF frame) ==========
    
    Real r_ECEF[3] "Position in ECEF [m]";
    Real v_ECEF[3] "Velocity in ECEF [m/s]";
    Real lat(unit="rad") "Latitude [rad]";
    Real lon(unit="rad") "Longitude [rad]";
    Real alt_geodetic "Altitude above WGS84 ellipsoid [m]";
    Real euler_ECEF[3] "Euler angles in ECEF [yaw, pitch, roll] [rad]";
    
    // ========== COMMANDED INPUTS (to be connected) ==========
    
    input Real T_cmd(start = 0) "Commanded thrust magnitude [N]";
    input Real u_thrust_cmd[3] (start= {1, 0, 0}) "Commanded thrust direction in body frame";
    
  equation
    // ========== MASS PROPERTIES ==========
    
    mass_model.m_dot_fuel = thrust_model.m_dot;
    thrust_model.x_cm = mass_model.x_cm;
    
    // ========== POSITION MAGNITUDE AND ALTITUDE ==========
    
    r_mag = sqrt(r_ECI[1]^2 + r_ECI[2]^2 + r_ECI[3]^2);
    altitude = r_mag - Constants.R_earth;
    
    // ========== ATMOSPHERE ==========
    
    rho = Atmosphere.density(altitude);
    q_dyn = 0.5 * rho * (v_body[1]^2 + v_body[2]^2 + v_body[3]^2);
    
    // ========== ATTITUDE TRANSFORMATIONS ==========
    
    // Rotation matrices
    R_ECI_to_Body = Transformations.quaternionToRotationMatrix(q);
    R_Body_to_ECI = transpose(R_ECI_to_Body);
    
    // Velocity in body frame
    //v_body = R_ECI_to_Body * v_ECI;
    v_body = Transformations.velocityECItoBodyWoEarthRotation(v_ECI, r_ECI, q);
    v_bodz = R_ECI_to_Body * v_ECI - { -Constants.omega_earth * r_ECI[2], Constants.omega_earth * r_ECI[1], 0.0};
    v_bodx = v_body - v_bodz;

    
    // ========== THRUST MODEL ==========
    
    thrust_model.T_cmd = T_cmd;
    thrust_model.u_thrust_cmd = u_thrust_cmd;
    
    // ========== AERODYNAMICS ==========
    
    aero_model.v_body = v_body;
    aero_model.omega_body = omega_body;
    aero_model.altitude = altitude;
    aero_model.q_dyn = q_dyn;
    
    // ========== FORCES IN ECI FRAME ==========
    
    // Gravity (point mass, in ECI)
    F_gravity_ECI = -Constants.mu * mass_model.m_total * r_ECI / r_mag^3;
    
    // Thrust (transform from body to ECI)
    F_thrust_ECI = R_Body_to_ECI * thrust_model.F_thrust;
    
    // Aerodynamics (transform from body to ECI)
    F_aero_ECI = R_Body_to_ECI * aero_model.F_aero;
    
    // Total force
    F_total_ECI = F_gravity_ECI + F_thrust_ECI + F_aero_ECI;
    
    F_gravity_ECEF = Transformations.forceECItoECEF(F_gravity_ECI, time + t0);
    F_thrust_ECEF  = Transformations.forceECItoECEF(F_thrust_ECI,  time + t0);
    F_aero_ECEF    = Transformations.forceECItoECEF(F_aero_ECI,    time + t0);
    F_total_ECEF   = Transformations.forceECItoECEF(F_total_ECI,   time + t0);
    
    // ========== TRANSLATIONAL DYNAMICS (ECI) ==========
    
    der(r_ECI) = v_ECI;
    mass_model.m_total * der(v_ECI) = F_total_ECI;
    
    // ========== MOMENTS IN BODY FRAME ==========
    
    M_total_body = thrust_model.M_thrust + aero_model.M_aero;
    
    // ========== ROTATIONAL DYNAMICS ==========
    
    // Quaternion kinematics: q_dot = 0.5 * q ⊗ [0, omega_body]
    q_dot_omega = 0.5 * Transformations.quaternionMultiply(q, {0, omega_body[1], omega_body[2], omega_body[3]});
    der(q) = q_dot_omega;
    
    // Quaternion normalization constraint (optional, for numerical stability)
    // This can be handled by re-normalizing periodically or using a constraint
    // For now, we'll let it drift slightly; could add: 0 = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2 - 1;
    
    // Inverse of inertia tensor (for 3x3 symmetric matrix)
    I_inv = Modelica.Math.Matrices.inv(mass_model.I_body);
    
    // Euler's rotation equation: I*omega_dot + omega × (I*omega) = M
    // omega_dot = I^{-1} * (M - omega × (I*omega))
    der(omega_body) = I_inv * (M_total_body - cross(omega_body, mass_model.I_body * omega_body));
    
    // ========== OUTPUTS IN ECEF ==========
    
    // Transform position to ECEF
    r_ECEF = Transformations.ECItoECEF(r_ECI, time + t0);
    
    // Transform velocity to ECEF
    v_ECEF = Transformations.velocityECItoECEF(r_ECI, v_ECI, time + t0);
    
    // Geodetic coordinates
    (lat, lon, alt_geodetic) = Transformations.ECEFtoGeodetic(r_ECEF);
    
    // Euler angles (currently giving body orientation relative to ECI)
    // To get orientation relative to ECEF, need to compose rotations
    // For now, providing body orientation from quaternion
    euler_ECEF = Transformations.quaternionToEuler(q);
    
  end Rocket;


model RocketHoverControl
  "Cascaded PID control system for hover and horizontal translation"
  
  // ============================================
  // CONTROL INPUTS (from 6-DOF model/sensors)
  // ============================================
  Modelica.Blocks.Interfaces.RealInput altitude "Current altitude [m]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput velocity_z "Vertical velocity [m/s]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput position_x "Horizontal position X [m]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput position_y "Horizontal position Y [m]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput velocity_x "Horizontal velocity X [m/s]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput velocity_y "Horizontal velocity Y [m/s]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput roll "Roll angle [rad]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput pitch "Pitch angle [rad]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput yaw "Yaw angle [rad]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput roll_rate "Roll rate [rad/s]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput pitch_rate "Pitch rate [rad/s]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput yaw_rate "Yaw rate [rad/s]" annotation(Placement(visible = true));
  
  // ============================================
  // CONTROL OUTPUTS (to actuators)
  // ============================================
  Modelica.Blocks.Interfaces.RealOutput throttle_cmd "Throttle command [0.9..1.2]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealOutput gimbal_pitch_cmd "Gimbal pitch angle [rad]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealOutput gimbal_yaw_cmd "Gimbal yaw angle [rad]" annotation(Placement(visible = true));
  
  // ============================================
  // MISSION SETPOINTS
  // ============================================
  Modelica.Blocks.Interfaces.RealInput target_altitude "Target hover altitude [m]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput target_position_x "Target horizontal position X [m]" annotation(Placement(visible = true));
  Modelica.Blocks.Interfaces.RealInput target_position_y "Target horizontal position Y [m]" annotation(Placement(visible = true));
  
  // ============================================
  // PARAMETERS - Tune these!
  // ============================================
  
  // Vehicle parameters
  parameter Real vehicle_mass = 30000 "Rocket mass [kg]";
  parameter Real gravity = 9.81 "Gravity [m/s²]";
  parameter Real max_gimbal_angle = 0.15 "Maximum gimbal angle [rad]";
  
  // Altitude control (outer loop - slow)
  parameter Real Kp_alt = 0.8 "Altitude proportional gain";
  parameter Real Ki_alt = 0.05 "Altitude integral gain";
  parameter Real Kd_alt = 2.0 "Altitude derivative gain";
  parameter Real alt_deadband = 0.1 "Altitude deadband [m]";
  
  // Vertical velocity control (middle loop)
  parameter Real Kp_vel_z = 0.15 "Vertical velocity proportional gain";
  parameter Real Ki_vel_z = 0.02 "Vertical velocity integral gain";
  parameter Real max_vel_z_setpoint = 2.0 "Max vertical velocity setpoint [m/s]";
  
  // Horizontal position control (outer loop - slow)
  parameter Real Kp_pos_xy = 0.05 "Horizontal position proportional gain";
  parameter Real Ki_pos_xy = 0.002 "Horizontal position integral gain";
  parameter Real Kd_pos_xy = 0.3 "Horizontal position derivative gain";
  parameter Real pos_xy_deadband = 0.5 "Position deadband [m]";
  
  // Horizontal velocity control (middle loop)
  parameter Real Kp_vel_xy = 0.8 "Horizontal velocity proportional gain";
  parameter Real Ki_vel_xy = 0.05 "Horizontal velocity integral gain";
  parameter Real max_vel_xy_setpoint = 1.5 "Max horizontal velocity setpoint [m/s]";
  parameter Real max_accel_xy = 1.5 "Max horizontal acceleration [m/s²]";
  
  // Attitude control (inner loop - fast)
  parameter Real Kp_att = 3.0 "Attitude proportional gain";
  parameter Real Ki_att = 0.1 "Attitude integral gain";
  parameter Real Kd_att = 0.8 "Attitude derivative gain";
  parameter Real max_attitude = 0.19 "Maximum attitude angle [rad] - slightly less than limit";
  
  // Attitude rate control (innermost loop - very fast)
  parameter Real Kp_rate = 1.5 "Attitude rate proportional gain";
  parameter Real Ki_rate = 0.3 "Attitude rate integral gain";
  parameter Real max_rate = 0.2 "Maximum attitude rate [rad/s]";
  
  // Anti-windup limits
  parameter Real max_alt_integral = 50.0 "Max altitude integral term";
  parameter Real max_vel_integral = 10.0 "Max velocity integral term";
  parameter Real max_pos_integral = 20.0 "Max position integral term";
  parameter Real max_att_integral = 0.5 "Max attitude integral term";
  
  // ============================================
  // INTERNAL VARIABLES
  // ============================================
  
  // Altitude control chain
  Real altitude_error;
  Real altitude_integral(start=0);
  Real velocity_z_setpoint;
  Real velocity_z_error;
  Real velocity_z_integral(start=0);
  Real throttle_raw;
  
  // Horizontal position control chain
  Real position_x_error;
  Real position_y_error;
  Real position_x_integral(start=0);
  Real position_y_integral(start=0);
  Real velocity_x_setpoint;
  Real velocity_y_setpoint;
  Real velocity_x_error;
  Real velocity_y_error;
  Real velocity_x_integral(start=0);
  Real velocity_y_integral(start=0);
  Real accel_x_cmd;
  Real accel_y_cmd;
  
  // Attitude control chain
  Real roll_setpoint;
  Real pitch_setpoint;
  Real attitude_roll_error;
  Real attitude_pitch_error;
  Real attitude_roll_integral(start=0);
  Real attitude_pitch_integral(start=0);
  Real roll_rate_setpoint;
  Real pitch_rate_setpoint;
  Real roll_rate_error;
  Real pitch_rate_error;
  Real roll_rate_integral(start=0);
  Real pitch_rate_integral(start=0);
  Real gimbal_pitch_raw;
  Real gimbal_yaw_raw;
  
  // Hover stabilization flag
  Boolean hover_stable;
  Real hover_timer(start=0);
  parameter Real hover_stable_time = 2.0 "Time to consider hover stable [s]";
  
  Real velocity_x_setpoint_t;
  Real velocity_y_setpoint_t;
  Real accel_x_cmd_t;
  Real accel_y_cmd_t;
  Real pitch_setpoint_t;
  Real roll_setpoint_t;
  Real pitch_rate_setpoint_t;
  Real roll_rate_setpoint_t;
  Real velocity_z_setpoint_t;

equation
  // ============================================
  // ALTITUDE CONTROL (Vertical axis)
  // ============================================
  
  // Outer loop: Altitude → Vertical Velocity Setpoint
  altitude_error = target_altitude - altitude;
  
  // Apply deadband to reduce oscillations
  der(altitude_integral) = if abs(altitude_error) > alt_deadband then 
                             min(max(altitude_error, -max_alt_integral/Ki_alt), max_alt_integral/Ki_alt)
                           else 0;
  
  velocity_z_setpoint_t = Kp_alt * altitude_error + 
                        Ki_alt * altitude_integral + 
                        Kd_alt * (-velocity_z);  // Derivative term uses measured velocity
  
  // Limit velocity setpoint
  velocity_z_setpoint = min(max(velocity_z_setpoint_t, -max_vel_z_setpoint), max_vel_z_setpoint);
  
  // Middle loop: Vertical Velocity → Throttle Command
  velocity_z_error = velocity_z_setpoint - velocity_z;
  
  der(velocity_z_integral) = if abs(throttle_raw) < 1.15 then  // Anti-windup
                               min(max(velocity_z_error, -max_vel_integral/Ki_vel_z), max_vel_integral/Ki_vel_z)
                             else 0;
  
  // Calculate required throttle (normalized to hover = 1.0)
  throttle_raw = 1.0 + Kp_vel_z * velocity_z_error + Ki_vel_z * velocity_z_integral;
  
  // Clamp throttle to physical limits
  throttle_cmd = min(max(throttle_raw, 0.9), 1.2);
  
  // ============================================
  // HORIZONTAL POSITION CONTROL
  // ============================================
  
  // Check if hover is stable before translating
  hover_stable = abs(altitude_error) < 0.5 and abs(velocity_z) < 0.3;
  der(hover_timer) = if hover_stable then 1 else -hover_timer/0.5;  // Decay if not stable
  
  // Outer loop: Position → Velocity Setpoint (only when hover is stable)
  position_x_error = if hover_timer > hover_stable_time then (target_position_x - position_x) else 0;
  position_y_error = if hover_timer > hover_stable_time then (target_position_y - position_y) else 0;
  
  // Apply deadband
  der(position_x_integral) = if abs(position_x_error) > pos_xy_deadband then
                               min(max(position_x_error, -max_pos_integral/Ki_pos_xy), max_pos_integral/Ki_pos_xy)
                             else 0;
  der(position_y_integral) = if abs(position_y_error) > pos_xy_deadband then
                               min(max(position_y_error, -max_pos_integral/Ki_pos_xy), max_pos_integral/Ki_pos_xy)
                             else 0;
  
  velocity_x_setpoint_t = Kp_pos_xy * position_x_error + 
                        Ki_pos_xy * position_x_integral + 
                        Kd_pos_xy * (-velocity_x);
  velocity_y_setpoint_t = Kp_pos_xy * position_y_error + 
                        Ki_pos_xy * position_y_integral + 
                        Kd_pos_xy * (-velocity_y);
  
  // Limit velocity setpoints
  velocity_x_setpoint = min(max(velocity_x_setpoint_t, -max_vel_xy_setpoint), max_vel_xy_setpoint);
  velocity_y_setpoint = min(max(velocity_y_setpoint_t, -max_vel_xy_setpoint), max_vel_xy_setpoint);
  
  // Middle loop: Horizontal Velocity → Acceleration Command
  velocity_x_error = velocity_x_setpoint - velocity_x;
  velocity_y_error = velocity_y_setpoint - velocity_y;
  
  der(velocity_x_integral) = min(max(velocity_x_error, -max_vel_integral/Ki_vel_xy), max_vel_integral/Ki_vel_xy);
  der(velocity_y_integral) = min(max(velocity_y_error, -max_vel_integral/Ki_vel_xy), max_vel_integral/Ki_vel_xy);
  
  accel_x_cmd_t = 0; // ??? Kp_vel_xy * velocity_x_error + Ki_vel_xy * velocity_x_integral;
  accel_y_cmd_t = 0; // ??? Kp_vel_xy * velocity_y_error + Ki_vel_xy * velocity_y_integral;
  
  // Limit acceleration commands
  accel_x_cmd = min(max(accel_x_cmd_t, -max_accel_xy), max_accel_xy);
  accel_y_cmd = min(max(accel_y_cmd_t, -max_accel_xy), max_accel_xy);
  
  // Convert acceleration to attitude setpoints (small angle approximation)
  // For horizontal acceleration: a_x = g * tan(pitch) ≈ g * pitch
  pitch_setpoint_t = -accel_x_cmd / gravity;  // Negative because pitch forward accelerates in +X
  roll_setpoint_t = accel_y_cmd / gravity;     // Roll right accelerates in +Y
  
  // Limit attitude setpoints
  pitch_setpoint = min(max(pitch_setpoint_t, -max_attitude), max_attitude);
  roll_setpoint = min(max(roll_setpoint_t, -max_attitude), max_attitude);
  
  // ============================================
  // ATTITUDE CONTROL (Inner loop - FAST)
  // ============================================
  
  // Outer attitude loop: Attitude → Rate Setpoint
  attitude_roll_error = roll_setpoint - roll;
  attitude_pitch_error = pitch_setpoint - pitch;
  
  der(attitude_roll_integral) = if abs(roll_rate_setpoint) < max_rate * 0.9 then
                                  min(max(attitude_roll_error, -max_att_integral/Ki_att), max_att_integral/Ki_att)
                                else 0;
  der(attitude_pitch_integral) = if abs(pitch_rate_setpoint) < max_rate * 0.9 then
                                   min(max(attitude_pitch_error, -max_att_integral/Ki_att), max_att_integral/Ki_att)
                                 else 0;
  
  roll_rate_setpoint_t = Kp_att * attitude_roll_error + 
                       Ki_att * attitude_roll_integral + 
                       Kd_att * (-roll_rate);
  pitch_rate_setpoint_t = Kp_att * attitude_pitch_error + 
                        Ki_att * attitude_pitch_integral + 
                        Kd_att * (-pitch_rate);
  
  // Limit rate setpoints
  roll_rate_setpoint = min(max(roll_rate_setpoint_t, -max_rate), max_rate);
  pitch_rate_setpoint = min(max(pitch_rate_setpoint_t, -max_rate), max_rate);
  
  // Inner attitude loop: Rate → Gimbal Commands
  roll_rate_error = roll_rate_setpoint - roll_rate;
  pitch_rate_error = pitch_rate_setpoint - pitch_rate;
  
  der(roll_rate_integral) = min(max(roll_rate_error, -max_att_integral/Ki_rate), max_att_integral/Ki_rate);
  der(pitch_rate_integral) = min(max(pitch_rate_error, -max_att_integral/Ki_rate), max_att_integral/Ki_rate);
  
  gimbal_pitch_raw = Kp_rate * pitch_rate_error + Ki_rate * pitch_rate_integral;
  gimbal_yaw_raw = Kp_rate * roll_rate_error + Ki_rate * roll_rate_integral;
  
  // Limit gimbal commands
  gimbal_pitch_cmd = min(max(gimbal_pitch_raw, -max_gimbal_angle), max_gimbal_angle);
  gimbal_yaw_cmd = min(max(gimbal_yaw_raw, -max_gimbal_angle), max_gimbal_angle);

  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}})), 
             Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
end RocketHoverControl;


model RocketFlightExample "Example simulation of rocket flight"

  Rocket vehicle;
  RocketHoverControl control;
  
  // Override initial conditions for this example
  // Launch from equator at 100 km altitude, moving eastward
  //parameter Real r0_ECI[3] = {Rocket6DOF.Constants.R_earth + 100e3, 0, 0};
  //parameter Real v0_ECI[3] = {0, 7500, 100}; // Orbital velocity + some vertical
  
  // Simple thrust profile for demonstration
//protected
  Real thrust_magnitude;
  Real thrust_direction[3];

  //Real mass;
  //Real vel;
  //Real alt;
  //Real tm;
  //Real acc;
  //Real force;
  
function forceDescent
  input  Real mass;
  input  Real vel;
  input  Real alt;
  input  Real tm;
  output Real force;
protected
  Real acc;
algorithm
  acc   := -2 * (alt / tm^2 + vel / tm);
  force := mass * (acc - Constants.g0);
end forceDescent;

equation

  // ============================================
  // MISSION SETPOINTS
  // ============================================
  control.target_altitude   = 0 "Target hover altitude [m]";
  control.target_position_x = 0 "Target horizontal position X [m]";
  control.target_position_y = 0 "Target horizontal position Y [m]";

  // ============================================
  // CONTROL INPUTS (from 6-DOF model/sensors)
  // ============================================

  control.altitude   = vehicle.altitude  "Current altitude [m]";
  control.position_x = vehicle.v_ECEF[2]  "Horizontal position X [m]" annotation(Placement(visible = true));
  control.position_y = vehicle.v_ECEF[3]  "Horizontal position Y [m]" annotation(Placement(visible = true));

  control.velocity_z = vehicle.r_ECEF[1] "Vertical velocity [m/s]" annotation(Placement(visible = true));
  control.velocity_x = vehicle.r_ECEF[2] "Horizontal velocity X [m/s]" annotation(Placement(visible = true));
  control.velocity_y = vehicle.r_ECEF[3] "Horizontal velocity Y [m/s]" annotation(Placement(visible = true));

  control.roll       = vehicle.euler_ECEF[3] "Roll angle [rad]" annotation(Placement(visible = true));
  control.pitch      = vehicle.euler_ECEF[2] "Pitch angle [rad]" annotation(Placement(visible = true));
  control.yaw        = vehicle.euler_ECEF[1] "Yaw angle [rad]" annotation(Placement(visible = true));
  control.roll_rate  = vehicle.omega_body[3] "Roll rate [rad/s]" annotation(Placement(visible = true));
  control.pitch_rate = vehicle.omega_body[2] "Pitch rate [rad/s]" annotation(Placement(visible = true));
  control.yaw_rate   = vehicle.omega_body[1] "Yaw rate [rad/s]" annotation(Placement(visible = true));
  
  // ============================================
  // CONTROL OUTPUTS (to actuators)
  // ============================================
  if time < 100 then
    thrust_magnitude = 700000.0; // 600 kN
    thrust_direction = {1.0, 0.0, 0.0};    // Thrust mostly in X direction (forward), small pitch up
  else
    thrust_magnitude        = control.throttle_cmd * 580000.0;
    thrust_direction = {1.0, 0.0, 0.0}; // { 1.0, tan(control.gimbal_pitch_cmd), tan(control.gimbal_yaw_cmd) };
  end if;

  vehicle.T_cmd        = thrust_magnitude;
  vehicle.u_thrust_cmd = thrust_direction;
  /*
  //vehicle. = control.throttle_cmd "Throttle command [0.9..1.2]" annotation(Placement(visible = true));
  //vehicle. = control.gimbal_pitch_cmd "Gimbal pitch angle [rad]" annotation(Placement(visible = true));
  //vehicle. = control.gimbal_yaw_cmd "Gimbal yaw angle [rad]" annotation(Placement(visible = true));
  
  // Example thrust profile: burn for 30 seconds, then coast
  mass  = vehicle.mass_model.m_total;
  vel   = vehicle.v_ECEF[1];
  alt   = vehicle.altitude;
  tm    = if alt > 1 then sqrt(alt) else 1.0;
  acc   = -2 * (alt / tm^2 + vel / tm);
  force = mass * (acc + Constants.g0);

    thrust_magnitude = max(0.0, min(mass*2*Constants.g0, forceDescent(mass, vel, alt, tm)));
    thrust_direction = {1.0, 0.0, 0.0};

  // Connect to rocket inputs
  */
  
  when vehicle.altitude < 0.10 then
    terminate("\n\nLanded!\n\n");
  end when;
  
  when vehicle.mass_model.m_fuel < 0.10 then
    terminate("\n\n** No Fuel!\n\n");
  end when;

  annotation(experiment(StartTime=0, StopTime=300, Interval=0.001));
  
end RocketFlightExample;

end Rocket6DOF;
