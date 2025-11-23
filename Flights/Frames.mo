within Flights;

package Frames "Coordinate frame transformations between BODY, ECEF, and ECI using quaternions"
  
  package Types "Type definitions for coordinate frames"
    type Quaternion = Real[4] "Quaternion [w, x, y, z] where w is scalar part";
    type Vector3D = Real[3] "3D vector [x, y, z]";
    type RotationMatrix = Real[3,3] "3x3 rotation matrix";
    
    type FrameType = enumeration(
      BODY "Body-fixed frame",
      ECEF "Earth-Centered Earth-Fixed frame",
      NED "North-East-Down frame (optional)",
      ECI "Earth-Centered Inertial frame") "Coordinate frame types";
  end Types;
  
  package Constants "Physical constants"
    constant Real omega_earth = 7.2921159e-5 "Earth rotation rate [rad/s]";
    constant Real omega_earth_vector[3] = {0, 0, omega_earth} 
      "Earth rotation vector in ECI frame [rad/s]";
  end Constants;
  
  package QuaternionOps "Quaternion operations"
    
    function normalize "Normalize quaternion to unit length"
      input Types.Quaternion q_in;
      output Types.Quaternion q_out;
    protected
      Real norm;
    algorithm
      norm := sqrt(q_in[1]^2 + q_in[2]^2 + q_in[3]^2 + q_in[4]^2);
      q_out := q_in / norm;
    end normalize;
    
    function conjugate "Quaternion conjugate (inverse for unit quaternions)"
      input Types.Quaternion q;
      output Types.Quaternion q_conj;
    algorithm
      q_conj := {q[1], -q[2], -q[3], -q[4]};
    end conjugate;
    
    function multiply "Quaternion multiplication: q1 * q2"
      input Types.Quaternion q1;
      input Types.Quaternion q2;
      output Types.Quaternion q_out;
    algorithm
      q_out[1] := q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] - q1[4]*q2[4];
      q_out[2] := q1[1]*q2[2] + q1[2]*q2[1] + q1[3]*q2[4] - q1[4]*q2[3];
      q_out[3] := q1[1]*q2[3] - q1[2]*q2[4] + q1[3]*q2[1] + q1[4]*q2[2];
      q_out[4] := q1[1]*q2[4] + q1[2]*q2[3] - q1[3]*q2[2] + q1[4]*q2[1];
    end multiply;
    
    function rotateVector "Rotate vector by quaternion: q * v * q'"
      input Types.Quaternion q;
      input Types.Vector3D v;
      output Types.Vector3D v_rot;
    protected
      Types.Quaternion q_v;
      Types.Quaternion q_conj;
      Types.Quaternion q_temp;
      Types.Quaternion q_result;
    algorithm
      // Convert vector to quaternion [0, v]
      q_v := {0, v[1], v[2], v[3]};
      
      // Get conjugate
      q_conj := conjugate(q);
      
      // q * q_v
      q_temp := multiply(q, q_v);
      
      // (q * q_v) * q'
      q_result := multiply(q_temp, q_conj);
      
      // Extract vector part
      v_rot := {q_result[2], q_result[3], q_result[4]};
    end rotateVector;
    
    function toRotationMatrix "Convert quaternion to rotation matrix"
      input Types.Quaternion q;
      output Types.RotationMatrix R;
    protected
      Real w, x, y, z;
      Real w2, x2, y2, z2;
    algorithm
      w := q[1]; x := q[2]; y := q[3]; z := q[4];
      w2 := w*w; x2 := x*x; y2 := y*y; z2 := z*z;
      
      R[1,1] := w2 + x2 - y2 - z2;
      R[1,2] := 2*(x*y - w*z);
      R[1,3] := 2*(x*z + w*y);
      
      R[2,1] := 2*(x*y + w*z);
      R[2,2] := w2 - x2 + y2 - z2;
      R[2,3] := 2*(y*z - w*x);
      
      R[3,1] := 2*(x*z - w*y);
      R[3,2] := 2*(y*z + w*x);
      R[3,3] := w2 - x2 - y2 + z2;
    end toRotationMatrix;
    
    function fromRotationMatrix "Convert rotation matrix to quaternion"
      input Types.RotationMatrix R;
      output Types.Quaternion q;
    protected
      Real trace;
      Real S;
    algorithm
      trace := R[1,1] + R[2,2] + R[3,3];
      
      if trace > 0 then
        S := sqrt(trace + 1.0) * 2; // S = 4*w
        q[1] := 0.25 * S;
        q[2] := (R[3,2] - R[2,3]) / S;
        q[3] := (R[1,3] - R[3,1]) / S;
        q[4] := (R[2,1] - R[1,2]) / S;
      elseif (R[1,1] > R[2,2]) and (R[1,1] > R[3,3]) then
        S := sqrt(1.0 + R[1,1] - R[2,2] - R[3,3]) * 2; // S = 4*x
        q[1] := (R[3,2] - R[2,3]) / S;
        q[2] := 0.25 * S;
        q[3] := (R[1,2] + R[2,1]) / S;
        q[4] := (R[1,3] + R[3,1]) / S;
      elseif R[2,2] > R[3,3] then
        S := sqrt(1.0 + R[2,2] - R[1,1] - R[3,3]) * 2; // S = 4*y
        q[1] := (R[1,3] - R[3,1]) / S;
        q[2] := (R[1,2] + R[2,1]) / S;
        q[3] := 0.25 * S;
        q[4] := (R[2,3] + R[3,2]) / S;
      else
        S := sqrt(1.0 + R[3,3] - R[1,1] - R[2,2]) * 2; // S = 4*z
        q[1] := (R[2,1] - R[1,2]) / S;
        q[2] := (R[1,3] + R[3,1]) / S;
        q[3] := (R[2,3] + R[3,2]) / S;
        q[4] := 0.25 * S;
      end if;
      
      q := normalize(q);
    end fromRotationMatrix;
    
    function derivative "Compute quaternion derivative from angular velocity"
      input Types.Quaternion q "Current quaternion";
      input Types.Vector3D omega "Angular velocity in body frame [rad/s]";
      output Types.Quaternion q_dot "Quaternion derivative";
    protected
      Real w, x, y, z;
      Real wx, wy, wz;
    algorithm
      w := q[1]; x := q[2]; y := q[3]; z := q[4];
      wx := omega[1]; wy := omega[2]; wz := omega[3];
      
      q_dot[1] := 0.5 * (-x*wx - y*wy - z*wz);
      q_dot[2] := 0.5 * ( w*wx + y*wz - z*wy);
      q_dot[3] := 0.5 * ( w*wy - x*wz + z*wx);
      q_dot[4] := 0.5 * ( w*wz + x*wy - y*wx);
    end derivative;
    
  end QuaternionOps;
  
  package PositionTransforms "Position transformations"
    
    function ECI_to_ECEF "Transform position from ECI to ECEF"
      input Types.Vector3D r_eci "Position in ECI frame [m]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D r_ecef "Position in ECEF frame [m]";
    protected
      Real c, s;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      r_ecef[1] :=  c*r_eci[1] + s*r_eci[2];
      r_ecef[2] := -s*r_eci[1] + c*r_eci[2];
      r_ecef[3] := r_eci[3];
    end ECI_to_ECEF;
    
    function ECEF_to_ECI "Transform position from ECEF to ECI"
      input Types.Vector3D r_ecef "Position in ECEF frame [m]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D r_eci "Position in ECI frame [m]";
    protected
      Real c, s;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      r_eci[1] := c*r_ecef[1] - s*r_ecef[2];
      r_eci[2] := s*r_ecef[1] + c*r_ecef[2];
      r_eci[3] := r_ecef[3];
    end ECEF_to_ECI;
    
    function ECEF_to_BODY "Transform position from ECEF to BODY"
      input Types.Vector3D r_ecef "Position in ECEF frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      output Types.Vector3D r_body "Position in BODY frame [m]";
    algorithm
      r_body := QuaternionOps.rotateVector(q_body_ecef, r_ecef);
    end ECEF_to_BODY;
    
    function BODY_to_ECEF "Transform position from BODY to ECEF"
      input Types.Vector3D r_body "Position in BODY frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      output Types.Vector3D r_ecef "Position in ECEF frame [m]";
    protected
      Types.Quaternion q_ecef_body;
    algorithm
      q_ecef_body := QuaternionOps.conjugate(q_body_ecef);
      r_ecef := QuaternionOps.rotateVector(q_ecef_body, r_body);
    end BODY_to_ECEF;
    
    function ECI_to_BODY "Transform position from ECI to BODY"
      input Types.Vector3D r_eci "Position in ECI frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D r_body "Position in BODY frame [m]";
    protected
      Types.Vector3D r_ecef;
    algorithm
      r_ecef := ECI_to_ECEF(r_eci, theta);
      r_body := ECEF_to_BODY(r_ecef, q_body_ecef);
    end ECI_to_BODY;
    
    function BODY_to_ECI "Transform position from BODY to ECI"
      input Types.Vector3D r_body "Position in BODY frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D r_eci "Position in ECI frame [m]";
    protected
      Types.Vector3D r_ecef;
    algorithm
      r_ecef := BODY_to_ECEF(r_body, q_body_ecef);
      r_eci := ECEF_to_ECI(r_ecef, theta);
    end BODY_to_ECI;
    
  end PositionTransforms;
  
  package VelocityTransforms "Velocity transformations"
    
    function ECI_to_ECEF "Transform velocity from ECI to ECEF (rotating frame)"
      input Types.Vector3D v_eci "Velocity in ECI frame [m/s]";
      input Types.Vector3D r_eci "Position in ECI frame [m]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D v_ecef "Velocity in ECEF frame [m/s]";
    protected
      Real c, s;
      Types.Vector3D v_transport;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      // Rotate velocity vector
      v_transport[1] :=  c*v_eci[1] + s*v_eci[2];
      v_transport[2] := -s*v_eci[1] + c*v_eci[2];
      v_transport[3] := v_eci[3];
      
      // Subtract velocity due to Earth rotation: v_ecef = v_eci - omega_earth x r_ecef
      // omega_earth x r = [0, 0, omega] x [x, y, z] = [-omega*y, omega*x, 0]
      v_ecef[1] := v_transport[1] + Constants.omega_earth * ( s*r_eci[1] - c*r_eci[2]);
      v_ecef[2] := v_transport[2] + Constants.omega_earth * ( c*r_eci[1] + s*r_eci[2]);
      v_ecef[1] := v_transport[1] - Constants.omega_earth * ( s*r_eci[1] - c*r_eci[2]); // ???
      v_ecef[2] := v_transport[2] - Constants.omega_earth * ( c*r_eci[1] + s*r_eci[2]); // ???
      v_ecef[3] := v_transport[3];
    end ECI_to_ECEF;
    
    function ECEF_to_ECI "Transform velocity from ECEF to ECI (rotating frame)"
      input Types.Vector3D v_ecef "Velocity in ECEF frame [m/s]";
      input Types.Vector3D r_ecef "Position in ECEF frame [m]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D v_eci "Velocity in ECI frame [m/s]";
    protected
      Real c, s;
      Types.Vector3D v_temp;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      // Add velocity due to Earth rotation: v_eci = v_ecef + omega_earth x r_ecef
      v_temp[1] := v_ecef[1] - Constants.omega_earth * r_ecef[2];
      v_temp[2] := v_ecef[2] + Constants.omega_earth * r_ecef[1];
      v_temp[3] := v_ecef[3];
      
      // Rotate to ECI
      v_eci[1] := c*v_temp[1] - s*v_temp[2];
      v_eci[2] := s*v_temp[1] + c*v_temp[2];
      v_eci[3] := v_temp[3];
    end ECEF_to_ECI;
    
    function ECEF_to_BODY "Transform velocity from ECEF to BODY (rotating frame)"
      input Types.Vector3D v_ecef "Velocity in ECEF frame [m/s]";
      input Types.Vector3D r_ecef "Position in ECEF frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Types.Vector3D omega_body_ecef "Angular velocity of BODY w.r.t. ECEF in BODY frame [rad/s]";
      output Types.Vector3D v_body "Velocity in BODY frame [m/s]";
    protected
      Types.Vector3D v_transport;
      Types.Vector3D r_body;
      Types.Vector3D omega_cross_r;
    algorithm
      // Rotate velocity vector to BODY frame
      v_transport := QuaternionOps.rotateVector(q_body_ecef, v_ecef);
      
      // Rotate position vector to BODY frame
      r_body := QuaternionOps.rotateVector(q_body_ecef, r_ecef);
      
      // Compute omega x r in BODY frame
      omega_cross_r := cross(omega_body_ecef, r_body);
      
      // v_body = R * v_ecef - omega x r
      v_body := v_transport - omega_cross_r;
    end ECEF_to_BODY;
    
    function BODY_to_ECEF "Transform velocity from BODY to ECEF (rotating frame)"
      input Types.Vector3D v_body "Velocity in BODY frame [m/s]";
      input Types.Vector3D r_body "Position in BODY frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Types.Vector3D omega_body_ecef "Angular velocity of BODY w.r.t. ECEF in BODY frame [rad/s]";
      output Types.Vector3D v_ecef "Velocity in ECEF frame [m/s]";
    protected
      Types.Quaternion q_ecef_body;
      Types.Vector3D omega_cross_r;
      Types.Vector3D v_temp;
    algorithm
      // Compute omega x r in BODY frame
      omega_cross_r := cross(omega_body_ecef, r_body);
      
      // v_temp = v_body + omega x r
      v_temp := v_body + omega_cross_r;
      
      // Rotate to ECEF frame
      q_ecef_body := QuaternionOps.conjugate(q_body_ecef);
      v_ecef := QuaternionOps.rotateVector(q_ecef_body, v_temp);
    end BODY_to_ECEF;
    
  end VelocityTransforms;
  
  package AccelerationTransforms "Acceleration transformations"
    
    function ECI_to_ECEF "Transform acceleration from ECI to ECEF (rotating frame)"
      input Types.Vector3D a_eci "Acceleration in ECI frame [m/s^2]";
      input Types.Vector3D v_eci "Velocity in ECI frame [m/s]";
      input Types.Vector3D r_eci "Position in ECI frame [m]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D a_ecef "Acceleration in ECEF frame [m/s^2]";
    protected
      Real c, s;
      Types.Vector3D a_transport;
      Types.Vector3D r_ecef;
      Real omega;
    algorithm
      c := cos(theta);
      s := sin(theta);
      omega := Constants.omega_earth;
      
      // Rotate acceleration
      a_transport[1] :=  c*a_eci[1] + s*a_eci[2];
      a_transport[2] := -s*a_eci[1] + c*a_eci[2];
      a_transport[3] := a_eci[3];
      
      // Position in ECEF
      r_ecef[1] :=  c*r_eci[1] + s*r_eci[2];
      r_ecef[2] := -s*r_eci[1] + c*r_eci[2];
      r_ecef[3] := r_eci[3];
      
      // a_ecef = a_transport - 2*omega x v_ecef - omega x (omega x r_ecef)
      // Coriolis: -2*omega x v
      // Centrifugal: -omega x (omega x r) = omega^2 * [r_x, r_y, 0]
      
      // Note: v_ecef needs to be computed from v_eci first
      // For simplicity in this function, we'll compute the terms directly
      
      // Centrifugal acceleration
      a_ecef[1] := a_transport[1] + 2*omega*(s*v_eci[1] - c*v_eci[2]) + omega^2*r_ecef[1];
      a_ecef[2] := a_transport[2] + 2*omega*(c*v_eci[1] + s*v_eci[2]) + omega^2*r_ecef[2];
      a_ecef[1] := a_transport[1] - 2*omega*(s*v_eci[1] - c*v_eci[2]) - omega^2*r_ecef[1];  // ???
      a_ecef[2] := a_transport[2] - 2*omega*(c*v_eci[1] + s*v_eci[2]) - omega^2*r_ecef[2];  // ???
      a_ecef[3] := a_transport[3];
    end ECI_to_ECEF;
    
    function ECEF_to_ECI "Transform acceleration from ECEF to ECI (rotating frame)"
      input Types.Vector3D a_ecef "Acceleration in ECEF frame [m/s^2]";
      input Types.Vector3D v_ecef "Velocity in ECEF frame [m/s]";
      input Types.Vector3D r_ecef "Position in ECEF frame [m]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D a_eci "Acceleration in ECI frame [m/s^2]";
    protected
      Real c, s;
      Types.Vector3D a_temp;
      Real omega;
    algorithm
      c := cos(theta);
      s := sin(theta);
      omega := Constants.omega_earth;
      
      // a_temp = a_ecef + 2*omega x v_ecef + omega x (omega x r_ecef)
      a_temp[1] := a_ecef[1] - 2*omega*v_ecef[2] - omega^2*r_ecef[1];
      a_temp[2] := a_ecef[2] + 2*omega*v_ecef[1] - omega^2*r_ecef[2];
      a_temp[3] := a_ecef[3];
      
      // Rotate to ECI
      a_eci[1] := c*a_temp[1] - s*a_temp[2];
      a_eci[2] := s*a_temp[1] + c*a_temp[2];
      a_eci[3] := a_temp[3];
    end ECEF_to_ECI;
    
    function ECEF_to_BODY "Transform acceleration from ECEF to BODY (rotating frame)"
      input Types.Vector3D a_ecef "Acceleration in ECEF frame [m/s^2]";
      input Types.Vector3D v_ecef "Velocity in ECEF frame [m/s]";
      input Types.Vector3D r_ecef "Position in ECEF frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Types.Vector3D omega_body_ecef "Angular velocity of BODY w.r.t. ECEF in BODY frame [rad/s]";
      input Types.Vector3D alpha_body_ecef "Angular acceleration of BODY w.r.t. ECEF in BODY frame [rad/s^2]";
      output Types.Vector3D a_body "Acceleration in BODY frame [m/s^2]";
    protected
      Types.Vector3D a_transport;
      Types.Vector3D v_body;
      Types.Vector3D r_body;
      Types.Vector3D coriolis;
      Types.Vector3D centrifugal;
      Types.Vector3D euler;
    algorithm
      // Rotate acceleration to BODY frame
      a_transport := QuaternionOps.rotateVector(q_body_ecef, a_ecef);
      
      // Rotate velocity to BODY frame
      v_body := QuaternionOps.rotateVector(q_body_ecef, v_ecef);
      
      // Rotate position to BODY frame
      r_body := QuaternionOps.rotateVector(q_body_ecef, r_ecef);
      
      // Coriolis acceleration: -2*omega x v
      coriolis := -2.0 * cross(omega_body_ecef, v_body);
      
      // Centrifugal acceleration: -omega x (omega x r)
      centrifugal := -cross(omega_body_ecef, cross(omega_body_ecef, r_body));
      
      // Euler acceleration: -alpha x r
      euler := -cross(alpha_body_ecef, r_body);
      
      // Total acceleration in BODY frame
      a_body := a_transport + coriolis + centrifugal + euler;
    end ECEF_to_BODY;
    
    function BODY_to_ECEF "Transform acceleration from BODY to ECEF (rotating frame)"
      input Types.Vector3D a_body "Acceleration in BODY frame [m/s^2]";
      input Types.Vector3D v_body "Velocity in BODY frame [m/s]";
      input Types.Vector3D r_body "Position in BODY frame [m]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Types.Vector3D omega_body_ecef "Angular velocity of BODY w.r.t. ECEF in BODY frame [rad/s]";
      input Types.Vector3D alpha_body_ecef "Angular acceleration of BODY w.r.t. ECEF in BODY frame [rad/s^2]";
      output Types.Vector3D a_ecef "Acceleration in ECEF frame [m/s^2]";
    protected
      Types.Quaternion q_ecef_body;
      Types.Vector3D coriolis;
      Types.Vector3D centrifugal;
      Types.Vector3D euler;
      Types.Vector3D a_temp;
    algorithm
      // Coriolis acceleration: 2*omega x v
      coriolis := 2.0 * cross(omega_body_ecef, v_body);
      
      // Centrifugal acceleration: omega x (omega x r)
      centrifugal := cross(omega_body_ecef, cross(omega_body_ecef, r_body));
      
      // Euler acceleration: alpha x r
      euler := cross(alpha_body_ecef, r_body);
      
      // Remove rotating frame effects
      a_temp := a_body + coriolis + centrifugal + euler;
      
      // Rotate to ECEF frame
      q_ecef_body := QuaternionOps.conjugate(q_body_ecef);
      a_ecef := QuaternionOps.rotateVector(q_ecef_body, a_temp);
    end BODY_to_ECEF;
    
  end AccelerationTransforms;
  
  package AngularVelocityTransforms "Angular velocity transformations"
    
    function ECEF_to_BODY "Transform angular velocity from ECEF to BODY"
      input Types.Vector3D omega_ecef "Angular velocity in ECEF frame [rad/s]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      output Types.Vector3D omega_body "Angular velocity in BODY frame [rad/s]";
    algorithm
      omega_body := QuaternionOps.rotateVector(q_body_ecef, omega_ecef);
    end ECEF_to_BODY;
    
    function BODY_to_ECEF "Transform angular velocity from BODY to ECEF"
      input Types.Vector3D omega_body "Angular velocity in BODY frame [rad/s]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      output Types.Vector3D omega_ecef "Angular velocity in ECEF frame [rad/s]";
    protected
      Types.Quaternion q_ecef_body;
    algorithm
      q_ecef_body := QuaternionOps.conjugate(q_body_ecef);
      omega_ecef := QuaternionOps.rotateVector(q_ecef_body, omega_body);
    end BODY_to_ECEF;
    
    function ECI_to_ECEF "Transform angular velocity from ECI to ECEF"
      input Types.Vector3D omega_eci "Angular velocity in ECI frame [rad/s]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D omega_ecef "Angular velocity in ECEF frame (includes Earth rotation) [rad/s]";
    protected
      Real c, s;
      Types.Vector3D omega_rot;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      // Rotate to ECEF
      omega_rot[1] :=  c*omega_eci[1] + s*omega_eci[2];
      omega_rot[2] := -s*omega_eci[1] + c*omega_eci[2];
      omega_rot[3] := omega_eci[3];
      
      // Add Earth rotation contribution
      omega_ecef := omega_rot - Constants.omega_earth_vector;
    end ECI_to_ECEF;
    
    function ECEF_to_ECI "Transform angular velocity from ECEF to ECI"
      input Types.Vector3D omega_ecef "Angular velocity in ECEF frame [rad/s]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D omega_eci "Angular velocity in ECI frame [rad/s]";
    protected
      Real c, s;
      Types.Vector3D omega_temp;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      // Add Earth rotation
      omega_temp := omega_ecef + Constants.omega_earth_vector;
      
      // Rotate to ECI
      omega_eci[1] := c*omega_temp[1] - s*omega_temp[2];
      omega_eci[2] := s*omega_temp[1] + c*omega_temp[2];
      omega_eci[3] := omega_temp[3];
    end ECEF_to_ECI;
    
    function relativeBodyECEF "Compute relative angular velocity of BODY w.r.t. ECEF"
      input Types.Vector3D omega_body_eci "Angular velocity of BODY w.r.t. ECI in BODY frame [rad/s]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      output Types.Vector3D omega_body_ecef "Angular velocity of BODY w.r.t. ECEF in BODY frame [rad/s]";
    protected
      Types.Vector3D omega_earth_body;
    algorithm
      // Transform Earth rotation to BODY frame
      omega_earth_body := QuaternionOps.rotateVector(q_body_ecef, Constants.omega_earth_vector);
      
      // Relative angular velocity
      omega_body_ecef := omega_body_eci - omega_earth_body;
    end relativeBodyECEF;
    
  end AngularVelocityTransforms;
  
  package AngularAccelerationTransforms "Angular acceleration transformations"
    
    function ECEF_to_BODY "Transform angular acceleration from ECEF to BODY"
      input Types.Vector3D alpha_ecef "Angular acceleration in ECEF frame [rad/s^2]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      input Types.Vector3D ignored_omega_body_ecef  "Angular velocity of BODY w.r.t. ECEF in BODY frame [rad/s]";
      input Types.Vector3D ignored_omega_ecef_body "Angular velocity for frame transformation [rad/s]";
      output Types.Vector3D alpha_body "Angular acceleration in BODY frame [rad/s^2]";
    protected
      Types.Vector3D alpha_transport;
      //Types.Vector3D omega_cross_alpha;
    algorithm
      // Rotate angular acceleration to BODY frame
      alpha_transport := QuaternionOps.rotateVector(q_body_ecef, alpha_ecef);
      
      // Transport theorem correction: alpha_body = R*alpha_ecef + omega x (R*alpha_ecef)
      // Simplified: alpha_body = alpha_transport (for rigid body transformation)
      alpha_body := alpha_transport;
    end ECEF_to_BODY;
    
    function BODY_to_ECEF "Transform angular acceleration from BODY to ECEF"
      input Types.Vector3D alpha_body "Angular acceleration in BODY frame [rad/s^2]";
      input Types.Quaternion q_body_ecef "Quaternion from ECEF to BODY";
      output Types.Vector3D alpha_ecef "Angular acceleration in ECEF frame [rad/s^2]";
    protected
      Types.Quaternion q_ecef_body;
    algorithm
      q_ecef_body := QuaternionOps.conjugate(q_body_ecef);
      alpha_ecef := QuaternionOps.rotateVector(q_ecef_body, alpha_body);
    end BODY_to_ECEF;
    
    function ECI_to_ECEF "Transform angular acceleration from ECI to ECEF"
      input Types.Vector3D alpha_eci "Angular acceleration in ECI frame [rad/s^2]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D alpha_ecef "Angular acceleration in ECEF frame [rad/s^2]";
    protected
      Real c, s;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      // Earth has constant rotation rate, so no additional terms
      alpha_ecef[1] :=  c*alpha_eci[1] + s*alpha_eci[2];
      alpha_ecef[2] := -s*alpha_eci[1] + c*alpha_eci[2];
      alpha_ecef[3] := alpha_eci[3];
    end ECI_to_ECEF;
    
    function ECEF_to_ECI "Transform angular acceleration from ECEF to ECI"
      input Types.Vector3D alpha_ecef "Angular acceleration in ECEF frame [rad/s^2]";
      input Real theta "Earth rotation angle [rad]";
      output Types.Vector3D alpha_eci "Angular acceleration in ECI frame [rad/s^2]";
    protected
      Real c, s;
    algorithm
      c := cos(theta);
      s := sin(theta);
      
      alpha_eci[1] := c*alpha_ecef[1] - s*alpha_ecef[2];
      alpha_eci[2] := s*alpha_ecef[1] + c*alpha_ecef[2];
      alpha_eci[3] := alpha_ecef[3];
    end ECEF_to_ECI;
    
  end AngularAccelerationTransforms;
  
  package Utilities "Utility functions"
    
    function cross "Cross product of two 3D vectors"
      input Types.Vector3D a;
      input Types.Vector3D b;
      output Types.Vector3D c;
    algorithm
      c[1] := a[2]*b[3] - a[3]*b[2];
      c[2] := a[3]*b[1] - a[1]*b[3];
      c[3] := a[1]*b[2] - a[2]*b[1];
    end cross;
    
    function dot "Dot product of two 3D vectors"
      input Types.Vector3D a;
      input Types.Vector3D b;
      output Real result;
    algorithm
      result := a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    end dot;
    
    function norm "Euclidean norm of a 3D vector"
      input Types.Vector3D v;
      output Real result;
    algorithm
      result := sqrt(v[1]^2 + v[2]^2 + v[3]^2);
    end norm;
    
    function skewSymmetric "Create skew-symmetric matrix from vector"
      input Types.Vector3D v;
      output Real[3,3] S;
    algorithm
      S[1,1] := 0;        S[1,2] := -v[3];   S[1,3] := v[2];
      S[2,1] := v[3];     S[2,2] := 0;       S[2,3] := -v[1];
      S[3,1] := -v[2];    S[3,2] := v[1];    S[3,3] := 0;
    end skewSymmetric;
    
  end Utilities;
  
  annotation(
    Documentation(info="<html>
<h1>CoordinateFrames Package</h1>
<p>
This package provides comprehensive coordinate frame transformations between:
</p>
<ul>
<li><b>ECI</b> - Earth-Centered Inertial frame</li>
<li><b>ECEF</b> - Earth-Centered Earth-Fixed frame</li>
<li><b>BODY</b> - Body-fixed frame</li>
</ul>

<h2>Features</h2>
<ul>
<li>Quaternion-based attitude representation</li>
<li>Position, velocity, and acceleration transformations</li>
<li>Angular velocity and angular acceleration transformations</li>
<li>Proper handling of rotating reference frames (transport theorem)</li>
<li>Coriolis and centrifugal acceleration effects</li>
<li>Earth rotation rate modeling</li>
</ul>

<h2>Usage Example</h2>
<pre>
// Transform position from ECI to BODY
r_body = PositionTransforms.ECI_to_BODY(r_eci, q_body_ecef, theta);

// Transform velocity from BODY to ECI
v_eci = VelocityTransforms.BODY_to_ECEF(v_body, r_body, q_body_ecef, omega_body_ecef);
v_eci = VelocityTransforms.ECEF_to_ECI(v_ecef, r_ecef, theta);

// Update quaternion from angular velocity
q_dot = QuaternionOps.derivative(q, omega);
</pre>

<h2>Coordinate Frame Conventions</h2>
<ul>
<li><b>ECI</b>: Z-axis points to North celestial pole, X-axis points to vernal equinox</li>
<li><b>ECEF</b>: Z-axis points to North pole, X-axis through Greenwich meridian, rotates with Earth</li>
<li><b>BODY</b>: Fixed to the vehicle/body, typically X-forward, Z-down (or up for rockets)</li>
</ul>

<h2>Important Notes</h2>
<ul>
<li>Quaternions use Hamilton convention: [w, x, y, z]</li>
<li>Earth rotation rate: 7.2921159e-5 rad/s</li>
<li>Velocity/acceleration transforms include rotating frame effects</li>
<li>theta parameter is the Earth rotation angle from ECI X-axis to ECEF X-axis</li>
</ul>
</html>"));
  
end Frames;