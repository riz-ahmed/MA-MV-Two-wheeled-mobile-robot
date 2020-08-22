# Masters Thesis

<h1>Model Validation for a Two-Wheeled Vehicle using Multibody Simulation and Experimental Data</h1>

<img src="https://user-images.githubusercontent.com/38962235/90955651-acd76c00-e47f-11ea-93b7-9d495311a948.PNG" alt="BicycleModel">

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
The wheel geometry is defined kinematically in SimMecahnics as shown in the following figure:
<img src="https://user-images.githubusercontent.com/38962235/90957757-26775600-e490-11ea-804f-8dd92d91e1aa.PNG" alt="WheelCoordinateSystem">
A dynamic model is then developed that generates longitudinal and lateral tyre forces at the wheel-road interface as shwon below:
<img src="https://user-images.githubusercontent.com/38962235/90957841-9b4a9000-e490-11ea-90ac-c1b70dbbb5ab.PNG" alt="DynamicTyreModel">
This dynamic tyre model generates longituna and lateral forces based on the longitudunal and lateral slips respectively. Further other wheel-road interface related forces and moments are genereted and added to the wheel model in the simulation.

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

<h2>Tricycle Model</h2>

Using the wheel model developed in the simulation, a tricycle model is developed using these wheels in order to establish the presence of centripetal forces while turning that maintain previously mentioned non-holonomic constraints. A pendulum is attached to the body of the tricycle model, which deflects as the tricycle turns into a corner. Reading the sensor values attached to this pendulum in the simulation will show the presense of forces that the tricycle body experiences while turning into a corner.

<img src="https://user-images.githubusercontent.com/38962235/90958096-4d368c00-e492-11ea-9297-d2f478414948.PNG" alt="Tricyclemodel">

<h2>Bicycle Model</h2>
