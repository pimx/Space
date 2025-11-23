within Flights;

package Transformations
  "Coordinate transformations between ECEF, ECI, NED, and Body frames"
  
  constant Real a = 6378137.0 "WGS84 semi-major axis [m]";
  constant Real f = 1/298.257223563 "WGS84 flattening";
  constant Real b = a*(1-f) "WGS84 semi-minor axis [m]";
  constant Real e2 = 1-(b/a)^2 "Square of first eccentricity";
  constant Real omega_earth = 7.292115e-5 "Earth rotation rate [rad/s]";
  constant Real theta_GST0 = 1.74933340 "Initial Greenwich sidereal time [rad]";
  
//#############################################################################
//
//  pos_WGS_to_ECEF(lat,lon,alt) -> ecef
//  pos_ECEF_to_WGS(ecef) -> (lat,lon,alt)
//
//#############################################################################

  function pos_WGS_to_ECEF
    "Convert geodetic coordinates (lat, lon, alt) to ECEF"
    input Real lat "Latitude [rad]";
    input Real lon "Longitude [rad]";
    input Real alt "Altitude above ellipsoid [m]";
    output Real r_ECEF[3] "Position in ECEF frame [m]";
  protected
    Real N "Prime vertical radius of curvature";
    Real sin_lat, cos_lat, sin_lon, cos_lon;
  algorithm
    sin_lat := sin(lat);
    cos_lat := cos(lat);
    sin_lon := sin(lon);
    cos_lon := cos(lon);
    
    N := a / sqrt(1 - e2*sin_lat^2);
    
    r_ECEF[1] := (N + alt)*cos_lat*cos_lon;
    r_ECEF[2] := (N + alt)*cos_lat*sin_lon;
    r_ECEF[3] := (N*(1-e2) + alt)*sin_lat;
  end pos_WGS_to_ECEF;

  function pos_ECEF_to_WGS
    "Convert ECEF coordinates to geodetic (lat, lon, alt)"
    input Real r_ECEF[3] "Position in ECEF frame [m]";
    output Real lat "Latitude [rad]";
    output Real lon "Longitude [rad]";
    output Real alt "Altitude above ellipsoid [m]";
  protected
    Real p, theta, N;
    Real sin_lat, cos_lat;
    constant Real e_prime2 = e2/(1-e2);
    constant Integer max_iter = 10;
    Real lat_old;
  algorithm
    lon := atan2(r_ECEF[2], r_ECEF[1]);
    p := sqrt(r_ECEF[1]^2 + r_ECEF[2]^2);
    
    theta := atan2(r_ECEF[3]*a, p*b);
    lat := atan2(r_ECEF[3] + e_prime2*b*sin(theta)^3,
                 p - e2*a*cos(theta)^3);
    
    for i in 1:max_iter loop
      lat_old := lat;
      sin_lat := sin(lat);
      cos_lat := cos(lat);
      N := a / sqrt(1 - e2*sin_lat^2);
      alt := p/cos_lat - N;
      lat := atan2(r_ECEF[3], p*(1 - e2*N/(N+alt)));
      
      if abs(lat - lat_old) < 1e-12 then
        break;
      end if;
    end for;
    
    sin_lat := sin(lat);
    cos_lat := cos(lat);
    N := a / sqrt(1 - e2*sin_lat^2);
    alt := p/cos_lat - N;
  end pos_ECEF_to_WGS;
  
//#############################################################################
//
//                              Rotations
//
//  rotation_ECEF_to_NED(lat,lon) -> ROT
//  rotation_matrix_from_origin(ecef) -> ROT
//  greenwich_sidereal_time(time) -> gst
//  rotation_ECI_to_ECEF(time) -> ROT
//  rotation_ECEF_to_ECI(time) -> ROT
//  rotation_ECI_to_NED(time,ecef) -> ROT
//  rotation_NED_to_ECI(time,ecef) -> ROT
//  rotation_BODY_to_NED_quat(q,ROT) -> q
//  rotation_NED_to_BODY_quat(q,ROT) -> q
//  quaternion_derivative(q,omega_body) -> q
//
//#############################################################################

  function rotation_WGS_to_NED
    "Rotation matrix from ECEF to NED frame"
    input Real lat "Latitude [rad]";
    input Real lon "Longitude [rad]";
    output Real R[3,3] "Rotation matrix ECEF to NED";
  protected
    Real sin_lat, cos_lat, sin_lon, cos_lon;
  algorithm
    sin_lat := sin(lat);
    cos_lat := cos(lat);
    sin_lon := sin(lon);
    cos_lon := cos(lon);
    
    R[1,1] := -sin_lat*cos_lon;
    R[1,2] := -sin_lat*sin_lon;
    R[1,3] := cos_lat;
    
    R[2,1] := -sin_lon;
    R[2,2] := cos_lon;
    R[2,3] := 0;
    
    R[3,1] := -cos_lat*cos_lon;
    R[3,2] := -cos_lat*sin_lon;
    R[3,3] := -sin_lat;
  end rotation_WGS_to_NED;
  
  function rotation_matrix_from_origin
    "Get rotation matrix from ECEF origin position"
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real R[3,3] "Rotation matrix ECEF to NED";
  protected
    Real lat, lon, alt;
  algorithm
    (lat, lon, alt) := pos_ECEF_to_WGS(r_origin_ECEF);
    R := rotation_WGS_to_NED(lat, lon);
  end rotation_matrix_from_origin;
  
  function greenwich_sidereal_time
    "Calculate Greenwich sidereal time"
    input Real t "Time since epoch [s]";
    output Real theta_GST "Greenwich sidereal time [rad]";
  algorithm
    theta_GST := theta_GST0 + omega_earth * t;
    theta_GST := mod(theta_GST, 2*Modelica.Constants.pi);
  end greenwich_sidereal_time;
  
  function rotation_ECI_to_ECEF
    "Rotation matrix from ECI to ECEF frame"
    input Real t "Time since epoch [s]";
    output Real R[3,3] "Rotation matrix ECI to ECEF";
  protected
    Real theta;
    Real cos_theta, sin_theta;
  algorithm
    theta := greenwich_sidereal_time(t);
    cos_theta := cos(theta);
    sin_theta := sin(theta);
    
    R[1,1] := cos_theta;
    R[1,2] := sin_theta;
    R[1,3] := 0;
    
    R[2,1] := -sin_theta;
    R[2,2] := cos_theta;
    R[2,3] := 0;
    
    R[3,1] := 0;
    R[3,2] := 0;
    R[3,3] := 1;
  end rotation_ECI_to_ECEF;
  
  function rotation_ECEF_to_ECI
    "Rotation matrix from ECEF to ECI frame"
    input Real t "Time since epoch [s]";
    output Real R[3,3] "Rotation matrix ECEF to ECI";
  algorithm
    R := transpose(rotation_ECI_to_ECEF(t));
  end rotation_ECEF_to_ECI;
  
  function rotation_ECI_to_NED
    "Rotation matrix from ECI to NED frame"
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real R[3,3] "Rotation matrix ECI to NED";
  protected
    Real R_ECI_to_ECEF[3,3];
    Real R_ECEF_to_NED[3,3];
  algorithm
    R_ECI_to_ECEF := rotation_ECI_to_ECEF(t);
    R_ECEF_to_NED := rotation_matrix_from_origin(r_origin_ECEF);
    R := R_ECEF_to_NED * R_ECI_to_ECEF;
  end rotation_ECI_to_NED;
  
  function rotation_NED_to_ECI
    "Rotation matrix from NED to ECI frame"
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real R[3,3] "Rotation matrix NED to ECI";
  algorithm
    R := transpose(rotation_ECI_to_NED(t, r_origin_ECEF));
  end rotation_NED_to_ECI;

  // ============ BODY FRAME ROTATIONS ============
  
  function rotation_BODY_to_NED_euler
    "Rotation matrix from Body to NED using Euler angles (ZYX sequence)"
    input Real roll "Roll angle φ [rad] - rotation about body X";
    input Real pitch "Pitch angle θ [rad] - rotation about body Y";
    input Real yaw "Yaw angle ψ [rad] - rotation about body Z";
    output Real R[3,3] "Rotation matrix Body to NED";
  protected
    Real cos_roll, sin_roll;
    Real cos_pitch, sin_pitch;
    Real cos_yaw, sin_yaw;
  algorithm
    cos_roll := cos(roll);
    sin_roll := sin(roll);
    cos_pitch := cos(pitch);
    sin_pitch := sin(pitch);
    cos_yaw := cos(yaw);
    sin_yaw := sin(yaw);
    
    // ZYX Euler sequence: R = Rz(ψ) * Ry(θ) * Rx(φ)
    R[1,1] := cos_yaw*cos_pitch;
    R[1,2] := cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll;
    R[1,3] := cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll;
    
    R[2,1] := sin_yaw*cos_pitch;
    R[2,2] := sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll;
    R[2,3] := sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll;
    
    R[3,1] := -sin_pitch;
    R[3,2] := cos_pitch*sin_roll;
    R[3,3] := cos_pitch*cos_roll;
  end rotation_BODY_to_NED_euler;
  
  function rotation_NED_to_BODY_euler
    "Rotation matrix from NED to Body using Euler angles"
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    output Real R[3,3] "Rotation matrix NED to Body";
  algorithm
    // Transpose of Body to NED rotation
    R := transpose(rotation_BODY_to_NED_euler(roll, pitch, yaw));
  end rotation_NED_to_BODY_euler;
  
  function rotation_BODY_to_NED_quat
    "Rotation matrix from Body to NED using quaternion"
    input Real q[4] "Quaternion [q0, q1, q2, q3] where q0 is scalar part";
    output Real R[3,3] "Rotation matrix Body to NED";
  protected
    Real q0, q1, q2, q3;
    Real q0_sq, q1_sq, q2_sq, q3_sq;
    Real norm_sq;
  algorithm
    // Normalize quaternion
    norm_sq := q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2;
    q0 := q[1] / sqrt(norm_sq);
    q1 := q[2] / sqrt(norm_sq);
    q2 := q[3] / sqrt(norm_sq);
    q3 := q[4] / sqrt(norm_sq);
    
    q0_sq := q0^2;
    q1_sq := q1^2;
    q2_sq := q2^2;
    q3_sq := q3^2;
    
    // Rotation matrix from quaternion
    R[1,1] := q0_sq + q1_sq - q2_sq - q3_sq;
    R[1,2] := 2*(q1*q2 - q0*q3);
    R[1,3] := 2*(q1*q3 + q0*q2);
    
    R[2,1] := 2*(q1*q2 + q0*q3);
    R[2,2] := q0_sq - q1_sq + q2_sq - q3_sq;
    R[2,3] := 2*(q2*q3 - q0*q1);
    
    R[3,1] := 2*(q1*q3 - q0*q2);
    R[3,2] := 2*(q2*q3 + q0*q1);
    R[3,3] := q0_sq - q1_sq - q2_sq + q3_sq;
  end rotation_BODY_to_NED_quat;
  
  function rotation_NED_to_BODY_quat
    "Rotation matrix from NED to Body using quaternion"
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    output Real R[3,3] "Rotation matrix NED to Body";
  algorithm
    R := transpose(rotation_BODY_to_NED_quat(q));
  end rotation_NED_to_BODY_quat;
  
  function euler_rates_to_BODY_angular_velocity
    "Convert Euler angle rates to body angular velocity"
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    input Real roll_rate "Roll rate dφ/dt [rad/s]";
    input Real pitch_rate "Pitch rate dθ/dt [rad/s]";
    input Real yaw_rate "Yaw rate dψ/dt [rad/s]";
    output Real omega_body[3] "Angular velocity in body frame [rad/s] [p, q, r]";
  protected
    Real cos_roll, sin_roll;
    Real cos_pitch, sin_pitch;
  algorithm
    cos_roll := cos(roll);
    sin_roll := sin(roll);
    cos_pitch := cos(pitch);
    sin_pitch := sin(pitch);
    
    // Transformation from Euler rates to body rates
    omega_body[1] := roll_rate - sin_pitch*yaw_rate;
    omega_body[2] := cos_roll*pitch_rate + sin_roll*cos_pitch*yaw_rate;
    omega_body[3] := -sin_roll*pitch_rate + cos_roll*cos_pitch*yaw_rate;
  end euler_rates_to_BODY_angular_velocity;
  
  function body_angular_velocity_to_euler_rates
    "Convert body angular velocity to Euler angle rates"
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real omega_body[3] "Angular velocity in body frame [rad/s] [p, q, r]";
    output Real roll_rate "Roll rate dφ/dt [rad/s]";
    output Real pitch_rate "Pitch rate dθ/dt [rad/s]";
    output Real yaw_rate "Yaw rate dψ/dt [rad/s]";
  protected
    Real cos_roll, sin_roll;
    Real cos_pitch, sin_pitch, tan_pitch;
  algorithm
    cos_roll := cos(roll);
    sin_roll := sin(roll);
    cos_pitch := cos(pitch);
    sin_pitch := sin(pitch);
    tan_pitch := tan(pitch);
    
    // Kinematic equations (singular at pitch = ±90°)
    roll_rate := omega_body[1] + sin_roll*tan_pitch*omega_body[2] + cos_roll*tan_pitch*omega_body[3];
    pitch_rate := cos_roll*omega_body[2] - sin_roll*omega_body[3];
    yaw_rate := (sin_roll/cos_pitch)*omega_body[2] + (cos_roll/cos_pitch)*omega_body[3];
  end body_angular_velocity_to_euler_rates;
  
  function quaternion_derivative
    "Calculate quaternion derivative from body angular velocity"
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    input Real omega_body[3] "Angular velocity in body frame [rad/s]";
    output Real q_dot[4] "Quaternion derivative";
  protected
    Real Omega[4,4] "Omega matrix";
  algorithm
    // Omega matrix for quaternion kinematics
    Omega[1,1] := 0;
    Omega[1,2] := -omega_body[1];
    Omega[1,3] := -omega_body[2];
    Omega[1,4] := -omega_body[3];
    
    Omega[2,1] := omega_body[1];
    Omega[2,2] := 0;
    Omega[2,3] := omega_body[3];
    Omega[2,4] := -omega_body[2];
    
    Omega[3,1] := omega_body[2];
    Omega[3,2] := -omega_body[3];
    Omega[3,3] := 0;
    Omega[3,4] := omega_body[1];
    
    Omega[4,1] := omega_body[3];
    Omega[4,2] := omega_body[2];
    Omega[4,3] := -omega_body[1];
    Omega[4,4] := 0;
    
    // q_dot = 0.5 * Omega * q
    q_dot := 0.5 * Omega * q;
  end quaternion_derivative;
  
  // ============ BODY-NED TRANSFORMATIONS ============
  
  function velocity_BODY_to_NED_euler
    "Transform velocity from Body to NED frame using Euler angles"
    input Real v_body[3] "Velocity in body frame [m/s]";
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    output Real v_NED[3] "Velocity in NED frame [m/s]";
  protected
    Real R[3,3];
  algorithm
    R := rotation_BODY_to_NED_euler(roll, pitch, yaw);
    v_NED := R * v_body;
  end velocity_BODY_to_NED_euler;
  
  function velocity_NED_to_BODY_euler
    "Transform velocity from NED to Body frame using Euler angles"
    input Real v_NED[3] "Velocity in NED frame [m/s]";
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    output Real v_body[3] "Velocity in body frame [m/s]";
  protected
    Real R[3,3];
  algorithm
    R := rotation_NED_to_BODY_euler(roll, pitch, yaw);
    v_body := R * v_NED;
  end velocity_NED_to_BODY_euler;
  
  function velocity_BODY_to_NED_quat
    "Transform velocity from Body to NED frame using quaternion"
    input Real v_body[3] "Velocity in body frame [m/s]";
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    output Real v_NED[3] "Velocity in NED frame [m/s]";
  protected
    Real R[3,3];
  algorithm
    R := rotation_BODY_to_NED_quat(q);
    v_NED := R * v_body;
  end velocity_BODY_to_NED_quat;
  
  function velocity_NED_to_BODY_quat
    "Transform velocity from NED to Body frame using quaternion"
    input Real v_NED[3] "Velocity in NED frame [m/s]";
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    output Real v_body[3] "Velocity in body frame [m/s]";
  protected
    Real R[3,3];
  algorithm
    R := rotation_NED_to_BODY_quat(q);
    v_body := R * v_NED;
  end velocity_NED_to_BODY_quat;
  
  function force_BODY_to_NED_euler
    "Transform force/acceleration from Body to NED frame using Euler angles"
    input Real F_body[3] "Force/acceleration in body frame";
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    output Real F_NED[3] "Force/acceleration in NED frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_BODY_to_NED_euler(roll, pitch, yaw);
    F_NED := R * F_body;
  end force_BODY_to_NED_euler;
  
  function force_NED_to_BODY_euler
    "Transform force/acceleration from NED to Body frame using Euler angles"
    input Real F_NED[3] "Force/acceleration in NED frame";
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    output Real F_body[3] "Force/acceleration in body frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_NED_to_BODY_euler(roll, pitch, yaw);
    F_body := R * F_NED;
  end force_NED_to_BODY_euler;
  
  function force_BODY_to_NED_quat
    "Transform force/acceleration from Body to NED frame using quaternion"
    input Real F_body[3] "Force/acceleration in body frame";
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    output Real F_NED[3] "Force/acceleration in NED frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_BODY_to_NED_quat(q);
    F_NED := R * F_body;
  end force_BODY_to_NED_quat;
  
  function force_NED_to_BODY_quat
    "Transform force/acceleration from NED to Body frame using quaternion"
    input Real F_NED[3] "Force/acceleration in NED frame";
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    output Real F_body[3] "Force/acceleration in body frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_NED_to_BODY_quat(q);
    F_body := R * F_NED;
  end force_NED_to_BODY_quat;
  
  function acceleration_ECI_to_BODY_quat
    "Transform any vector from ECI frame to Body frame"
    input Real vec_ECI[3] "Vector in ECI frame";
    input Real q[4] "Quaternion from ECI to Body (q0, q1, q2, q3)";
    output Real vec_Body[3] "Vector in Body frame";

  protected
    Real R_ECI2Body[3,3] "Rotation matrix from ECI to Body frame";
    Real q0, q1, q2, q3;

  algorithm
    q0 := q[1];
    q1 := q[2];
    q2 := q[3];
    q3 := q[4];
    
    R_ECI2Body[1,1] := q0^2 + q1^2 - q2^2 - q3^2;
    R_ECI2Body[1,2] := 2*(q1*q2 + q0*q3);
    R_ECI2Body[1,3] := 2*(q1*q3 - q0*q2);
    
    R_ECI2Body[2,1] := 2*(q1*q2 - q0*q3);
    R_ECI2Body[2,2] := q0^2 - q1^2 + q2^2 - q3^2;
    R_ECI2Body[2,3] := 2*(q2*q3 + q0*q1);
    
    R_ECI2Body[3,1] := 2*(q1*q3 + q0*q2);
    R_ECI2Body[3,2] := 2*(q2*q3 - q0*q1);
    R_ECI2Body[3,3] := q0^2 - q1^2 - q2^2 + q3^2;
    
    vec_Body := R_ECI2Body * vec_ECI;
  end acceleration_ECI_to_BODY_quat;

  function acceleration_BODY_to_NED_euler
    "Transform acceleration including rotational effects"
    input Real a_body[3] "Linear acceleration in body frame [m/s²]";
    input Real omega_body[3] "Angular velocity in body frame [rad/s]";
    input Real alpha_body[3] "Angular acceleration in body frame [rad/s²]";
    input Real r_CG[3] "Position of point relative to CG in body frame [m]";
    input Real roll "Roll angle φ [rad]";
    input Real pitch "Pitch angle θ [rad]";
    input Real yaw "Yaw angle ψ [rad]";
    output Real a_NED[3] "Acceleration in NED frame [m/s²]";
  protected
    Real R[3,3];
    Real a_total_body[3];
  algorithm
    // Total acceleration at point including rotational effects
    // a_total = a_CG + alpha × r + omega × (omega × r)
    a_total_body := a_body + cross(alpha_body, r_CG) + 
                    cross(omega_body, cross(omega_body, r_CG));
    
    R := rotation_BODY_to_NED_euler(roll, pitch, yaw);
    a_NED := R * a_total_body;
  end acceleration_BODY_to_NED_euler;
  
  function acceleration_BODY_to_NED_quat
    "Transform acceleration including rotational effects using quaternion"
    input Real a_body[3] "Linear acceleration in body frame [m/s²]";
    input Real omega_body[3] "Angular velocity in body frame [rad/s]";
    input Real alpha_body[3] "Angular acceleration in body frame [rad/s²]";
    input Real r_CG[3] "Position of point relative to CG in body frame [m]";
    input Real q[4] "Quaternion [q0, q1, q2, q3]";
    output Real a_NED[3] "Acceleration in NED frame [m/s²]";
  protected
    Real R[3,3];
    Real a_total_body[3];
  algorithm
    a_total_body := a_body + cross(alpha_body, r_CG) + 
                    cross(omega_body, cross(omega_body, r_CG));
    
    R := rotation_BODY_to_NED_quat(q);
    a_NED := R * a_total_body;
  end acceleration_BODY_to_NED_quat;
  
  // ============ PREVIOUS ECEF/ECI FUNCTIONS ============
  
  function position_ECEF_to_NED
    "Transform position from ECEF to local NED frame"
    input Real r_ECEF[3] "Position in ECEF [m]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real r_local[3] "Position in local NED frame [m]";
  protected
    Real R[3,3];
    Real dr_ECEF[3];
  algorithm
    dr_ECEF := r_ECEF - r_origin_ECEF;
    R := rotation_matrix_from_origin(r_origin_ECEF);
    r_local := R * dr_ECEF;
  end position_ECEF_to_NED;
  
  function position_NED_to_ECEF
    "Transform position from local NED frame to ECEF"
    input Real r_local[3] "Position in local NED frame [m]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real r_ECEF[3] "Position in ECEF [m]";
  protected
    Real R[3,3];
    Real dr_ECEF[3];
  algorithm
    R := rotation_matrix_from_origin(r_origin_ECEF);
    dr_ECEF := transpose(R) * r_local;
    r_ECEF := r_origin_ECEF + dr_ECEF;
  end position_NED_to_ECEF;
  
  function position_ECI_to_ECEF
    "Transform position from ECI to ECEF frame"
    input Real r_ECI[3] "Position in ECI frame [m]";
    input Real t "Time since epoch [s]";
    output Real r_ECEF[3] "Position in ECEF frame [m]";
  protected
    Real R[3,3];
  algorithm
    R := rotation_ECI_to_ECEF(t);
    r_ECEF := R * r_ECI;
  end position_ECI_to_ECEF;
  
  function position_ECEF_to_ECI
    "Transform position from ECEF to ECI frame"
    input Real r_ECEF[3] "Position in ECEF frame [m]";
    input Real t "Time since epoch [s]";
    output Real r_ECI[3] "Position in ECI frame [m]";
  protected
    Real R[3,3];
  algorithm
    R := rotation_ECEF_to_ECI(t);
    r_ECI := R * r_ECEF;
  end position_ECEF_to_ECI;
  
  function position_ECI_to_NED
    "Transform position from ECI to local NED frame"
    input Real r_ECI[3] "Position in ECI frame [m]";
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real r_local[3] "Position in local NED frame [m]";
  protected
    Real r_ECEF[3];
  algorithm
    r_ECEF := position_ECI_to_ECEF(r_ECI, t);
    r_local := position_ECEF_to_NED(r_ECEF, r_origin_ECEF);
  end position_ECI_to_NED;
  
  function position_NED_to_ECI
    "Transform position from local NED frame to ECI"
    input Real r_local[3] "Position in local NED frame [m]";
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real r_ECI[3] "Position in ECI frame [m]";
  protected
    Real r_ECEF[3];
  algorithm
    r_ECEF := position_NED_to_ECEF(r_local, r_origin_ECEF);
    r_ECI := position_ECEF_to_ECI(r_ECEF, t);
  end position_NED_to_ECI;
  
  function velocity_ECEF_to_NED
    "Transform velocity from ECEF to local NED frame"
    input Real r_ECEF[3] "Position in ECEF [m]";
    input Real v_ECEF[3] "Velocity in ECEF frame [m/s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real v_local[3] "Velocity in local NED frame [m/s]";
  protected
    Real R[3,3];
    Real omega_ECEF[3];
    Real v_rel_ECEF[3];
  algorithm
    R := rotation_matrix_from_origin(r_origin_ECEF);
    omega_ECEF := {0, 0, omega_earth};
    v_rel_ECEF := v_ECEF - cross(omega_ECEF, r_ECEF - r_origin_ECEF);
    v_local := R * v_rel_ECEF;
  end velocity_ECEF_to_NED;
  
  function velocity_NED_to_ECEF
    "Transform velocity from local NED frame to ECEF"
    input Real r_ECEF[3] "Position in ECEF [m]";
    input Real v_local[3] "Velocity in local NED frame [m/s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real v_ECEF[3] "Velocity in ECEF frame [m/s]";
  protected
    Real R[3,3];
    Real omega_ECEF[3];
    Real v_rel_ECEF[3];
  algorithm
    R := rotation_matrix_from_origin(r_origin_ECEF);
    v_rel_ECEF := transpose(R) * v_local;
    omega_ECEF := {0, 0, omega_earth};
    v_ECEF := v_rel_ECEF + cross(omega_ECEF, r_ECEF - r_origin_ECEF);
  end velocity_NED_to_ECEF;
  
  function velocity_ECI_to_ECEF
    "Transform velocity from ECI to ECEF frame"
    input Real r_ECI[3] "Position in ECI frame [m]";
    input Real v_ECI[3] "Velocity in ECI frame [m/s]";
    input Real t "Time since epoch [s]";
    output Real v_ECEF[3] "Velocity in ECEF frame [m/s]";
  protected
    Real R[3,3];
    Real omega_Earth[3] "Earth rotation vector in ECI";
    Real r_ECEF[3];
  algorithm
    R := rotation_ECI_to_ECEF(t);
    r_ECEF := R * r_ECI;
    
    // Omega in ECI frame (Z-axis)
    omega_Earth := {0, 0, omega_earth};
    
    // v_ECEF = R*(v_ECI - omega × r_ECI)
    v_ECEF := R * (v_ECI - cross(omega_Earth, r_ECI));
  end velocity_ECI_to_ECEF;
  
  function velocity_ECEF_to_ECI
    "Transform velocity from ECEF to ECI frame"
    input Real r_ECEF[3] "Position in ECEF frame [m]";
    input Real v_ECEF[3] "Velocity in ECEF frame [m/s]";
    input Real t "Time since epoch [s]";
    output Real v_ECI[3] "Velocity in ECI frame [m/s]";
  protected
    Real R[3,3];
    Real omega_Earth[3] "Earth rotation vector in ECI";
    Real r_ECI[3];
  algorithm
    R := rotation_ECEF_to_ECI(t);
    r_ECI := R * r_ECEF;
    
    // Omega in ECI frame (Z-axis)
    omega_Earth := {0, 0, omega_earth};
    
    // v_ECI = R*v_ECEF + omega × r_ECI
    v_ECI := R * v_ECEF + cross(omega_Earth, r_ECI);
  end velocity_ECEF_to_ECI;
  
  function velocity_ECI_to_NED
    "Transform velocity from ECI to local NED frame"
    input Real r_ECI[3] "Position in ECI frame [m]";
    input Real v_ECI[3] "Velocity in ECI frame [m/s]";
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real v_local[3] "Velocity in local NED frame [m/s]";
  protected
    Real r_ECEF[3];
    Real v_ECEF[3];
  algorithm
    r_ECEF := position_ECI_to_ECEF(r_ECI, t);
    v_ECEF := velocity_ECI_to_ECEF(r_ECI, v_ECI, t);
    v_local := velocity_ECEF_to_NED(r_ECEF, v_ECEF, r_origin_ECEF);
  end velocity_ECI_to_NED;
  
  function velocity_NED_to_ECI
    "Transform velocity from local NED frame to ECI"
    input Real r_local[3] "Position in local NED frame [m]";
    input Real v_local[3] "Velocity in local NED frame [m/s]";
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real v_ECI[3] "Velocity in ECI frame [m/s]";
  protected
    Real r_ECEF[3];
    Real v_ECEF[3];
  algorithm
    r_ECEF := position_NED_to_ECEF(r_local, r_origin_ECEF);
    v_ECEF := velocity_NED_to_ECEF(r_ECEF, v_local, r_origin_ECEF);
    v_ECI := velocity_ECEF_to_ECI(r_ECEF, v_ECEF, t);
  end velocity_NED_to_ECI;
  function force_ECEF_to_NED
    "Transform force/acceleration vector from ECEF to local NED"
    input Real F_ECEF[3] "Force/acceleration in ECEF frame";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real F_local[3] "Force/acceleration in local NED frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_matrix_from_origin(r_origin_ECEF);
    F_local := R * F_ECEF;
  end force_ECEF_to_NED;

//#############################################################################
//
//                                  Forces
//  NED_to_ECEF(ned,ecef) > ecef
//  ECI_to_ECEF
//  ECEF_to_ECI
//  ECI_to_NED
//  NED_to_ECI
//
//#############################################################################


  function force_NED_to_ECEF
    "Transform force/acceleration vector from local NED to ECEF"
    input Real F_local[3] "Force/acceleration in local NED frame";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real F_ECEF[3] "Force/acceleration in ECEF frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_matrix_from_origin(r_origin_ECEF);
    F_ECEF := transpose(R) * F_local;
  end force_NED_to_ECEF;
  
  function force_ECI_to_ECEF
    "Transform force/acceleration vector from ECI to ECEF"
    input Real F_ECI[3] "Force/acceleration in ECI frame";
    input Real t "Time since epoch [s]";
    output Real F_ECEF[3] "Force/acceleration in ECEF frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_ECI_to_ECEF(t);
    F_ECEF := R * F_ECI;
  end force_ECI_to_ECEF;
  
  function force_ECEF_to_ECI
    "Transform force/acceleration vector from ECEF to ECI"
    input Real F_ECEF[3] "Force/acceleration in ECEF frame";
    input Real t "Time since epoch [s]";
    output Real F_ECI[3] "Force/acceleration in ECI frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_ECEF_to_ECI(t);
    F_ECI := R * F_ECEF;
  end force_ECEF_to_ECI;
  
  function force_ECI_to_NED
    "Transform force/acceleration vector from ECI to local NED"
    input Real F_ECI[3] "Force/acceleration in ECI frame";
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real F_local[3] "Force/acceleration in local NED frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_ECI_to_NED(t, r_origin_ECEF);
    F_local := R * F_ECI;
  end force_ECI_to_NED;
  
  function force_NED_to_ECI
    "Transform force/acceleration vector from local NED to ECI"
    input Real F_local[3] "Force/acceleration in local NED frame";
    input Real t "Time since epoch [s]";
    input Real r_origin_ECEF[3] "Origin position in ECEF [m]";
    output Real F_ECI[3] "Force/acceleration in ECI frame";
  protected
    Real R[3,3];
  algorithm
    R := rotation_NED_to_ECI(t, r_origin_ECEF);
    F_ECI := R * F_local;
  end force_NED_to_ECI;

end Transformations;
