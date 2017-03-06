# Extracorporeal_Experience

3D pointclouds merge through an internet server and pointcloud-based mesh-display into a virtual reality headset

Behaviour : 
  - 3D pointclouds received from multiple Kinects and sent to a server 
  - 3D pointclouds received from the server and merged into a single one using downsampling weighted by likelihood of 3D-points
  - Meshing of the pointcloud by neighbours approach and display into a virtual reality headset

Used materials: 
  - 3 Kinect cameras
  - 1 OSVR (Open-Source Virtual Reality headset)

Used tools: Visual Studio, Unity 3D

Kinect Client Unity: 
  - Unity package for receiving the pointcloud, meshing it, and displaying it into the headset 
Kinect Client WebGL:
  - Server Client resources
Kinect Server:
  - Server package: Kinect360Server is for multiple kinects, KinectOneServer is for one kinect and KinectServer contains the server parameters

Launching the experience: 
  - Put corresponding ip adresses on the Unity application (mesh object parameters) and on the kinect application (KinectServer/ServerParameters.cs)
  - Set up the code in Kinect360Server/Program.cs for the right number of kinects
  - Launch the two applications
