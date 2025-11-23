within Flights;
  
model Aerodynamics "6-DOF aerodynamic forces and moments"
    
    // Geometric parameters
    parameter Real S_ref = Constants.S_ref "Reference area [m^2]";
    parameter Real L_ref = Constants.L_ref "Reference length [m]";
    parameter Real D_ref = Constants.D_ref "Reference diameter [m]";
    
    // Inputs
    input Real v_body[3] "Velocity in body frame [m/s]";
    input Real omega_body[3] "Angular velocity in body frame [rad/s]";
    input Real altitude "Altitude [m]";
    
    // Outputs
    output Real F_aero[3] "Aerodynamic force in body frame [N]";
    output Real M_aero[3] "Aerodynamic moment in body frame [N·m]";
    
    Real rho "Atmosphere density";
    Real q_dyn "Dynamic pressure [Pa]";
    Real F_aerox[3] "Aerodynamic force in body frame [N]";
    Real M_aerox[3] "Aerodynamic moment in body frame [N·m]";
    Real v_body_norm[3];
  // Aerodynamic angles
    Real alpha "Angle of attack [rad]";
    Real beta "Sideslip angle [rad]";
    Real V_mag2 "Velocity magnitude^2 [(m/s)^2]";
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

  
    V_mag2 = v_body[1]^2 + v_body[2]^2 + v_body[3]^2;       // Velocity magnitude^2
    V_mag  = sqrt(V_mag2);                                  // Velocity magnitude
    Mach   = V_mag / Atmosphere.speedOfSound(altitude);     // Mach number
    rho = Atmosphere.density(altitude);
    q_dyn = 0.5 * rho * V_mag2;

    // Aerodynamic angles
    alpha = 0; // atan2(v_body[3], v_body[1]);
    beta  = 0; // asin(v_body[2] / max(V_mag, 1e-6));
    
  
    
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

    //F_aero = F_aerox;
    v_body_norm = Modelica.Math.Vectors.normalize(v_body);
    F_aero = v_body_norm * F_aerox[1];
  
    //F_aero[1] = 0; // -q_dyn*S_ref*CD; // Drag (negative X)
    //F_aero[2] = 0; //  q_dyn*S_ref*CY;  // Side force
    //F_aero[3] = 0; // -q_dyn*S_ref*CL; // Lift (negative Z for positive alpha)
    
    // Aerodynamic moments in body frame
    M_aerox[1] =   q_dyn*S_ref*D_ref*Cl; // Rolling moment
    M_aerox[2] =   q_dyn*S_ref*L_ref*Cm; // Pitching moment
    M_aerox[3] =   q_dyn*S_ref*L_ref*Cn; // Yawing moment
  
    //M_aero = M_aerox;
    M_aero = {0,0,0};

    //M_aero[1] = 0; //  q_dyn*S_ref*D_ref*Cl; // Rolling moment
    //M_aero[2] = 0; //  q_dyn*S_ref*L_ref*Cm; // Pitching moment
    //M_aero[3] = 0; //  q_dyn*S_ref*L_ref*Cn; // Yawing moment
    
  end Aerodynamics;
