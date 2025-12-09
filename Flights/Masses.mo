within Flights;

model Masses
  
  // Mass properties
  parameter Real massDry      = Constants.M_dry  "Mass [kg]";       // масса сухого аппарата
  parameter Real massFuelInit = Constants.M_fuel "Mass [kg]";       // масса топлива
  parameter Real D = Constants.D_ref;                           // диаметр корпуса
  parameter Real L = Constants.L_ref;                           // длина корпуса
  parameter Real I_xx = 1.0 / 2.0 * (D/2)^2;                    // 100  "Moment of inertia [kg⋅m²]";
  parameter Real I_yy = 1.0 / 4.0 * (D/2)^2 + 1.0 / 12.0 * L^2; // 1000 "Moment of inertia [kg⋅m²]";
  parameter Real I_zz = 1.0 / 4.0 * (D/2)^2 + 1.0 / 12.0 * L^2; // 1000 "Moment of inertia [kg⋅m²]";
  
  // input
  input  Real rateFuel;                                              // масса аппарата с топливом

  // output
  output Real mass;
  output Real inertia[3,3];
  output Real[3] centerPosition;

  Real massFuel(start=massFuelInit);

equation
    der(massFuel) = -rateFuel;
    mass    = massDry + max(0, massFuel);
    inertia = diagonal({I_xx*mass, I_yy*mass, I_zz*mass});
    centerPosition = {10,0,0};
end Masses;
