within Platform;

model Imu "Inertial Measurement Unit"
  import SI = Modelica.Units.SI;
  
  // Parameters
  parameter Real accelNoise = 0.02 "Accelerometer noise [m/s^2]";
  parameter Real gyroNoise  = 0.005 "Gyroscope noise [rad/s]";
  parameter Real accelBias  = 0.01 "Accelerometer bias [m/s^2]";
  parameter Real gyroBias   = 0.002 "Gyroscope bias [rad/s]";
  

  // Input from Landing
  Real acc[3];
  Real vel[3];
  Real pos[3];
  Real omega[3];
  Real euler[3];

  // Output variables
  //Real imu_acc[3];
  Real imu_vel[3];
  Real imu_pos[3];
  Real imu_omega[3];
  Real imu_euler[3];
  
  // State variables
  Real meas_acc[3](start={0,0,0});
  Real meas_vel[3](start={0,0,0});
  Real meas_pos[3](start={0,0,0});
  Real meas_omega[3](start={0,0,0});
  Real meas_euler[3](start={0,0,0});

  Real c1,c2,c3,s1,s2,s3;
  
equation

  // Add noise and bias to measurements
  meas_acc = acc + 
    {accelBias, accelBias, accelBias} +
    {accelNoise*sin(1000*time), 
     accelNoise*sin(2000*time), 
     accelNoise*sin(3000*time)};
  
  meas_omega = omega + 
    {gyroBias, gyroBias, gyroBias} +
    {gyroNoise*sin(4000*time), 
     gyroNoise*sin(5000*time), 
     gyroNoise*sin(6000*time)};
  
  // Integration
  der(meas_vel) = meas_acc;
  der(meas_pos) = meas_vel;
  
  c1 = cos(meas_euler[1]); s1 = sin(meas_euler[1]);
  c2 = cos(meas_euler[2]); s2 = sin(meas_euler[2]);
  c3 = cos(meas_euler[3]); s3 = sin(meas_euler[3]);

  der(meas_euler[1]) = meas_omega[1] + meas_omega[2]*s1*s2/c2 + meas_omega[3]*c1*s2/c2;
  der(meas_euler[2]) = meas_omega[2]*c1 - meas_omega[3]*s1;
  der(meas_euler[3]) = meas_omega[2]*s1/c2 + meas_omega[3]*c1/c2;

  //imu_acc   = meas_acc;

  imu_vel   = meas_vel;
  imu_pos   = meas_pos;
  imu_omega = meas_omega;
  imu_euler = meas_euler;

  //imu_vel   = vel;
  //imu_pos   = pos;
  //imu_omega = omega;
  //imu_euler = euler;

  
end Imu;
