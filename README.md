# Masters Thesis

<h1>Model Validation for a Two-Wheeled Vehicle using Multibody Simulation and Experimental Data</h1>

<img src="https://user-images.githubusercontent.com/38962235/90955651-acd76c00-e47f-11ea-93b7-9d495311a948.PNG" alt="BicycleModel">

<h2>Running the Simulation</h2>
<p>
  <ul>
    <li>Open the Simulink Project</li>
    <li>Run the Model file from Models/BicycleHome.slx, this is the main Simulation File</li>
    <li>Script file is loaded automoatically to run the model, in order to make changes in the simulation parameters use the file from Script/BicycleInit.m</li>
</ul>
</p>

<p>Git repository of my masters thesis submitted to LRS, TU Kaiserslautern. The thesis covers the following objectives:
<ul>
  <li>Modeling a non-holonomic wheel model that follows the velocity constraints</li>
  <li>Development of a multibody simulation of a four-body bicycle model, the wheels are modeled to follow the nonholonomic constraints.</li>
  <li>The self-stabilization behavior of the bicycle to be maintained by the simulation.</li>
  <li>Validation of multibody model through anaytical methods using Eigen Value Analysis comparision with Linear Models as well as with Experimental data from a real bicycle</li>
  <li>Establishing the controllability of the bicycle model</li>
  <li>Using the controller developed, the bicycle model is tested with various path following (reference tracking) scenarios, in order to establish the controllability of the deisgned model</li>
</ul>
</p>

<h2> Modeling a non-holonomic constrainted bicycle wheel </h2>
<p>The non-holonomic constrainted wheel model follows the constraints not on the position but on velocity as follows 
<img src="https://render.githubusercontent.com/render/math?math=\dot{x} = v cos{\varphi}"> and <img src="https://render.githubusercontent.com/render/math?math=\dot{x} = v sin{\varphi}">, where <img src="https://render.githubusercontent.com/render/math?math=v"> is the forward velocity of the wheel
</p>

<p>
  The wheel geometry is defined kinematically in SimMecahnics as shown in the following figure:
  <img src="https://user-images.githubusercontent.com/38962235/90957757-26775600-e490-11ea-804f-8dd92d91e1aa.PNG" alt="WheelCoordinateSystem">
</p>

<p>
  A dynamic model is then developed that generates longitudinal and lateral tyre forces at the wheel-road interface as shwon below:
  <img src="https://user-images.githubusercontent.com/38962235/90957841-9b4a9000-e490-11ea-90ac-c1b70dbbb5ab.PNG" alt="DynamicTyreModel">
</p>

<p>
  This dynamic tyre model generates longituna and lateral forces based on the longitudunal and lateral slips respectively. Further other wheel-road interface related forces and  moments are genereted and added to the wheel model in the simulation.
</p>

<p>
Further the calculated tyre forces are modeled as a first order response to the slips. This dynamic behaviour which is also called as relaxation length in the tyre accounts to the distance the wheel travels before it reaches 63% of the steady-state value in the force build-up. This kind of dynamic behavior behavior acts like a mass-spring system in series
that gives the stiffness to the wheel longitudinal and lateral axis.

This wheel model contains the following dynamics:
<ul>
  <li>vertical normal force</li>
  <li>longitudinal force with relaxation length</li>
  <li>lateral force with relaxation length</li>
  <li>camber thrust due to wheel camber with the road (there is no relaxation length associated with camber thrust)</li>
  <li>Self aligning moment with relaxation length</li>
</ul>
</p>
<h2>Tricycle Model</h2>

<p>
Using the wheel model developed in the simulation, a tricycle model is developed using these wheels in order to establish the presence of centripetal forces while turning that maintain previously mentioned non-holonomic constraints. A pendulum is attached to the body of the tricycle model, which deflects as the tricycle turns into a corner. Reading the sensor values attached to this pendulum in the simulation will show the presense of forces that the tricycle body experiences while turning into a corner.

<img src="https://user-images.githubusercontent.com/38962235/90958096-4d368c00-e492-11ea-9297-d2f478414948.PNG" alt="Tricyclemodel">
</p>

<h2>Bicycle Model</h2>
<p>
After establishing that the wheel model certainly follows the non-holonomic constraints, a bicycle model is developed using these wheels in the simulator as shwon in the figure
<img src="https://user-images.githubusercontent.com/38962235/90958137-9b4b8f80-e492-11ea-8878-b0dcaad78607.PNG" alt="BlueBike">
</p>
<p>
The self-stability of the bicycle model developed can be explored running the simultion in various conditions, one of the these tests produced during simulation is as shown
<img src="https://user-images.githubusercontent.com/38962235/90958155-d0f07880-e492-11ea-8db5-867772f962a6.PNG" alt="SelfStableMotion" class="center">
</p>
<p>
This is in accordance with the Eigenvalue analysis perfomed in lienar bicycle model. It can be seen form this figure that for any small perturbation, the lean angle and the seering angle of the bicycle damps out slowly and maintains the steady motion, self correction the external perturbation and therby maintaining stability without the influnce of a rider. This phenomenon is also refered as "Ghost Riding". This also serves one of an important characteristic while designing controllers for reference tracking when stability of the system is already achieved.
</p>
<p>
Further dynamics of the bike are established such as a non-minimal phase behavior. Where any small perturbation causes the steering to turn initially in the oppoiste direction and then self-correct itself in the direction of turning. This phenomenon is also called as "Counter - Steering" among motorcyclists and used extensively during motocycle racing to effectively turn into a corner. This non-minimal phase behavior can be seen through sensor readings from the simualtion as shown below
<img src="https://user-images.githubusercontent.com/38962235/90958313-eb772180-e493-11ea-9fa1-ef626944db18.PNG" alt="NonMinimalPhaseBehavior" class="center">
</p>

<h2>Controllabiltiy Analysis</h2>
  <p>
    After establishing the dynamcis of the bicycle and validating the the simulation model as per the requirements, in the next step a controllability analysis is performed on the bicycle model using state feedback controller.
  </p>
  
  <p>
  A full state-feedback controller is designed using LQR technique for tuning, for a reference trajectory of a circular motion, the following path is traced by the simualtion model
  <img src="https://user-images.githubusercontent.com/38962235/90958537-6856cb00-e495-11ea-9789-ab704c62a233.PNG" alt="PathFollowing">
</p>
