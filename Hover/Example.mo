within Hover;

model Example "Example usage of the rocket dynamics model"
  
	import Modelica.Math.*;
	//import Modelica.Units.SI;
  
	// Instantiate the rocket model
	RocketModel rocket; //(
//		m = 30000,  // 5000 kg rocket
//		J = diagonal({2500, 10000, 10000}),  // Typical rocket inertia
//		n_forces = 2  // Two additional forces (e.g., aerodynamic drag and lift)
//	);
  
	// Time-varying thrust profile
	//Real thrust_profile;

	Real	hoverPosition[3] (start={0, 0.1, 0.2});
	Real	hoverVelocity[3];
  
equation
	// Simple thrust profile
	//thrust_profile = if time < 50 then rocket.g * rocket.m + 200 else 
	//				 if time < 90 then rocket.g * rocket.m - 100 else 0;
  
	// Thrust inputs
	rocket.requiredMagnitude = rocket.g * rocket.m - 1100*(hoverPosition[1] - 20.0) - 10000*hoverVelocity[1];
	rocket.centerPosition    = { 10,     0, 0 };  // Thrust applied 2m below COM
	rocket.requiredDirection = Vectors.normalize({  1, -hoverPosition[2]-hoverVelocity[2], -hoverPosition[3]-hoverVelocity[3] });  // Thrust in +z body direction

	hoverPosition = rocket.r_world;
	hoverVelocity = rocket.v_world;
  
	/*
	// Aerodynamic drag (simplified)
	rocket.force_magnitude[1] = 0.5 * 1.225 * 50 * 
		sqrt(rocket.v_world[1]^2 + rocket.v_world[2]^2 + rocket.v_world[3]^2)^2;
	rocket.force_point_body[1,:] = {0, 0, 1};  // Drag at nose
	rocket.force_direction_body[1,:] = if sqrt(rocket.v_world[1]^2 + 
		rocket.v_world[2]^2 + rocket.v_world[3]^2) > 0.1 then
		-transpose(rocket.R) * rocket.v_world / 
		sqrt(rocket.v_world[1]^2 + rocket.v_world[2]^2 + rocket.v_world[3]^2)
		else {0, 0, 0};
	
	// Side force (e.g., from fins or control surfaces)
	rocket.force_magnitude[2] = 1000 * sin(0.5*time);  // Oscillating side force
	rocket.force_point_body[2,:] = {0, 0, -3};  // Applied at tail
	rocket.force_direction_body[2,:] = {1, 0, 0};  // In +x body direction
	*/
  
  annotation(experiment(StopTime=100, NumberOfIntervals=10000));
  
end Example;
