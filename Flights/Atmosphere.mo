  within Flights;
  
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
