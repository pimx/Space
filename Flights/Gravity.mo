within Flights;

model Gravity
    input     Real pos_ECEF[3];
    output    Real grav_ECEF[3];
    output    Real grav_mag;
  protected
    // Local variables
    Real r "Geocentric radius";
    Real r2, r5;
    Real z2_r2 "z^2/r^2 ratio";
    Real g_r, g_z "Gravity components in spherical";
    // Constants from Types
    constant Real Re = Constants.R_earth;
    constant Real mu = Constants.mu;
    constant Real J2 = Constants.J2;
equation
    // Geocentric radius
    r = sqrt(pos_ECEF[1]^2 + pos_ECEF[2]^2 + pos_ECEF[3]^2);
    r2 = r^2;
    r5 = r^5;
    z2_r2 = pos_ECEF[3]^2 / r2;
    
    // J2 gravity model in ECEF (spherical approximation)
    // Radial component
    g_r = -mu/r2 * (1 + 1.5*J2*(Re/r)^2 * (1 - 5*z2_r2));
    
    // Z component correction
    g_z = -mu/r2 * (1 + 1.5*J2*(Re/r)^2 * (3 - 5*z2_r2));
    
    // Gravity in ECEF
    grav_ECEF[1] = g_r * pos_ECEF[1] / r;
    grav_ECEF[2] = g_r * pos_ECEF[2] / r;
    grav_ECEF[3] = g_z * pos_ECEF[3] / r;
    
    // Gravity magnitude
    grav_mag = sqrt(grav_ECEF[1]^2 + grav_ECEF[2]^2 + grav_ECEF[3]^2);
end Gravity;
