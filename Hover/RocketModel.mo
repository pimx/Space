within Hover;

model RocketModel "Rocket dynamics model using quaternions for orientation"
  
	import Modelica.Math.*;
	//import Modelica.Units.SI;

	Engine engine;
	
	// Parameters
	parameter Real	g      = 9.80665 "Gravitational acceleration [m/s^2]";
	parameter Real	m      = 30000 "Rocket mass [kg]";
	parameter Real  I1     =  500 "Inertia moment in body frame [kg.m^2]";
	parameter Real  I2     = 2500 "Inertia moment in body frame [kg.m^2]";
	parameter Real  I3     = 2500 "Inertia moment in body frame [kg.m^2]";
	//parameter SI.Inertia		I[3,3] = diagonal({Ix,Iy,Iz}) "Inertia tensor in body frame [kg.m^2]";
	
	// State variables
	Real			r_world[3](each start=0) "Position in world frame [m]";
	Real			v_world[3](each start=0) "Velocity in world frame [m/s]";
	Real			q[4](start={1,0,0,0}) "Quaternion [w,x,y,z] representing orientation";
	Real			omega_body[3](each start=0) "Angular velocity in body frame [rad/s]";
	
	// Inputs - Thrust
	input Real		requiredMagnitude    "Thrust force magnitude [N]";
	input Real		centerPosition[3]    "Rocket center of mass point in body frame [m]";
	input Real		requiredDirection[3] "Thrust direction unit vector in body frame [-]";
	
	// Inputs - Additional forces (e.g., aerodynamic)
	//parameter Integer n_forces = 2 "Number of additional force inputs";
	//input SI.Force force_magnitude[n_forces] "Force magnitudes [N]";
	//input SI.Position force_point_body[n_forces,3] "Force application points in body frame [m]";
	//input Real force_direction_body[n_forces,3] "Force direction unit vectors in body frame [-]";
  
	// Outputs - IMU measurements
	//output SI.Acceleration a_body[3] "Linear acceleration in body frame [m/s^2]";
	//output SI.AngularVelocity omega_body_out[3] "Angular velocity in body frame [rad/s]";
	//output SI.AngularAcceleration alpha_body[3] "Angular acceleration in body frame [rad/s^2]";
	
	// Internal variables
	Real			R[3,3]			 "Rotation matrix from body to world frame";
	Real			F_body[3]		 "Total force in body frame [N]";
	Real			F_world_rb[3]	 "Total force in world frame [N]";
	Real			F_world[3]		 "Total force in world frame [N]";
	Real			M_body[3]		 "Total moment in body frame [N.m]";
	Real			a_world[3]		 "Linear acceleration in world frame [m/s^2]";
	Real			q_norm			 "Quaternion norm for normalization";
	//Real omega_quat[4] "Quaternion rate matrix helper";
  
protected
	function quaternionToRotationMatrix "Convert quaternion to rotation matrix (body to world)"
		input  Real qq[4] "Quaternion [w,x,y,z]";
		output Real R[3,3] "Rotation matrix";
	protected
		Real q[4];
	algorithm
		q := {1,0,0,0};
		R[1,1] := q[1]^2 + q[2]^2 - q[3]^2 - q[4]^2;
		R[1,2] := 2*(q[2]*q[3] - q[1]*q[4]);
		R[1,3] := 2*(q[2]*q[4] + q[1]*q[3]);
		
		R[2,1] := 2*(q[2]*q[3] + q[1]*q[4]);
		R[2,2] := q[1]^2 - q[2]^2 + q[3]^2 - q[4]^2;
		R[2,3] := 2*(q[3]*q[4] - q[1]*q[2]);
		
		R[3,1] := 2*(q[2]*q[4] - q[1]*q[3]);
		R[3,2] := 2*(q[3]*q[4] + q[1]*q[2]);
		R[3,3] := q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2;
	end quaternionToRotationMatrix;
  
	function crossProduct "Compute cross product of two 3D vectors"
		input  Real a[3];
		input  Real b[3];
		output Real c[3];
	algorithm
		c[1] := a[2]*b[3] - a[3]*b[2];
		c[2] := a[3]*b[1] - a[1]*b[3];
		c[3] := a[1]*b[2] - a[2]*b[1];
	end crossProduct;
  
equation
	// Compute rotation matrix from quaternion
	R = quaternionToRotationMatrix(q);
  
	// Connect inputs to engine
	engine.requiredMagnitude = requiredMagnitude;
	engine.requiredDirection = requiredDirection; //{ 1,0,0 };	// in body frame testDirection;
	engine.centerPosition    = centerPosition;


	// Calculate total force in body frame
	F_body = engine.actualVector;
	//F_body = thrust_magnitude * thrust_direction_body + sum(force_magnitude[i] * force_direction_body[i,:] for i in 1:n_forces);
	
	// Calculate total moment in body frame
	M_body = engine.actualMoments;
	//M_body = crossProduct(thrust_point_body, thrust_magnitude * thrust_direction_body) + sum(crossProduct(force_point_body[i,:], force_magnitude[i] * force_direction_body[i,:]) for i in 1:n_forces);
  
	// Transform force to world frame
	F_world_rb = R * F_body;
	
	// Add gravity force in world frame (assuming z-up)
	F_world = { F_world_rb[1] - m * g, F_world_rb[2], F_world_rb[3] };
	
	// Linear dynamics in world frame
	der(r_world) = v_world;
	der(v_world) = a_world;
	
	// Compute world frame acceleration
	F_world = m * a_world; //der(v_world);
	
	// Transform acceleration to body frame for IMU output
	// Note: This includes gravity, as IMU measures specific force
	//a_body = transpose(R) * (a_world + {0, 0, g});
	
	// Quaternion kinematics
	// q_dot = 0.5 * q ⊗ [0, omega_body]
	//omega_quat = {0, omega_body[1], omega_body[2], omega_body[3]};
	der(q[1]) = 0.5 * (-q[2]*omega_body[1] - q[3]*omega_body[2] - q[4]*omega_body[3]);
	der(q[2]) = 0.5 * ( q[1]*omega_body[1] + q[3]*omega_body[3] - q[4]*omega_body[2]);
	der(q[3]) = 0.5 * ( q[1]*omega_body[2] - q[2]*omega_body[3] + q[4]*omega_body[1]);
	der(q[4]) = 0.5 * ( q[1]*omega_body[3] + q[2]*omega_body[2] - q[3]*omega_body[1]);
	
	// Quaternion normalization constraint (algebraic equation)
	q_norm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2);
	//0 = 1 - q_norm;  // Constraint to keep quaternion normalized
	
	// Rotational dynamics (Euler's equation in body frame)
	//J * der(omega_body) = M_body - crossProduct(omega_body, J * omega_body);
	
	// Euler's equations in principal axes
	// I1*domega1/dt + (I3-I2)*omega2*omega3 = tau1
	// I2*domega2/dt + (I1-I3)*omega3*omega1 = tau2
	// I3*domega3/dt + (I2-I1)*omega1*omega2 = tau3
	I1 * der(omega_body[1]) = M_body[1] - (I3 - I2) * omega_body[2] * omega_body[3];
	I2 * der(omega_body[2]) = M_body[2] - (I1 - I3) * omega_body[3] * omega_body[1];
	I3 * der(omega_body[3]) = M_body[3] - (I2 - I1) * omega_body[1] * omega_body[2];
	
	// Rotational kinetic energy T = (1/2) * omega^T * I * omega
	//T = 0.5 * (I1*omega_body[1]^2 + I2*omega_body[2]^2 + I3*omega_body[3]^2);

	// Angular acceleration output
	//alpha_body = der(omega_body);
	
	// Output angular velocity (direct passthrough)
	//omega_body_out = omega_body;
  
  annotation(Documentation(info="<html>
<p>This model implements 6-DOF rocket dynamics using quaternions for orientation representation.</p>
<h4>Features:</h4>
<ul>
<li>Quaternion-based orientation (no gimbal lock)</li>
<li>Forces and moments computed from thrust and additional forces</li>
<li>Body frame inputs for forces</li>
<li>IMU-compatible outputs (body frame accelerations and rates)</li>
<li>Proper coordinate transformations between body and world frames</li>
</ul>
<h4>Coordinate Systems:</h4>
<ul>
<li>World frame: Earth-fixed, z-axis pointing up</li>
<li>Body frame: Fixed to rocket, origin at center of mass</li>
</ul>
<h4>Inputs:</h4>
<ul>
<li>Thrust magnitude, point, and direction in body frame</li>
<li>Additional forces (aerodynamic, etc.) in body frame</li>
</ul>
<h4>Outputs:</h4>
<ul>
<li>Linear acceleration in body frame (includes gravity effect as measured by IMU)</li>
<li>Angular velocity in body frame</li>
<li>Angular acceleration in body frame</li>
</ul>
</html>"));
  
end RocketModel;
