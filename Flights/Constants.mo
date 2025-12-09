within Flights;

package Constants

    constant Real pos0_ECEF[3] = Transformations.pos_WGS_to_ECEF(0,0,0)    "положение начала системы координат NED (старт)";

    constant Real q_NB_V[4] = {1.0/sqrt(2.0), 0, 1.0/sqrt(2.0), 0};      // pitch = -90° (nose up) with roll = 0° and yaw = 0°:
    constant Real q_NB_H[4] = {1, 0, 0, 0};                      // Horizontal, nose North:
    constant Real q_NB_R[4] = {0.5, 0, 0.5, sqrt(2.0)};          // Vertical up, but rotated 90° (nose East when viewed from above):

    // Earth parameters
    constant Real mu               = 3.986004418e14   "Earth gravitational parameter [m^3/s^2]";
    constant Real omega_earth      = 7.2921159e-5     "Earth rotation rate [rad/s]";
    constant Real R_earth          = 6378137.0        "WGS84 equatorial radius [m]";
    constant Real f_earth          = 1/298.257223563  "WGS84 flattening";
    constant Real e_earth          = sqrt(2*f_earth - f_earth^2) "WGS84 eccentricity";
    constant Real gravity          = g0               "Gravity [m/s²]";
    constant Real g0               = 9.80665          "Standard gravity [m/s^2]";
    constant Real J2               = 1.082626683e-3 "Earth J2 coefficient";
    constant Real flattening       = 1.0/298.257223563 "Earth flattening (WGS84)";
    constant Real e2               = 2*flattening - flattening^2 "Earth eccentricity squared";

    // mass and inertial parameters
    constant Real S_ref            = 7.0              "Reference area [m^2]";
    constant Real L_ref            = 30.0             "Reference length [m]";
    constant Real D_ref            = 3.0              "Reference diameter [m]";
    constant Real M_dry            = 24000.0          "Dry mass [kg]";
    constant Real M_fuel           = 40000.0          "Fuel mass [kg]";
    /*
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
    constant Real Kp_rate  = 1.5 "Attitude rate proportional gain";
    constant Real Ki_rate  = 0.3 "Attitude rate integral gain";
    constant Real max_rate = 0.2 "Maximum attitude rate [rad/s]";
    
    // Anti-windup limits
    constant Real max_alt_integral = 50.0 "Max altitude integral term";
    constant Real max_vel_integral = 10.0 "Max velocity integral term";
    constant Real max_pos_integral = 20.0 "Max position integral term";
    constant Real max_att_integral = 0.50 "Max attitude integral term";
    */

end Constants;
