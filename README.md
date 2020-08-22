# Masters Thesis

<h1>Model Validation for a Two-Wheeled Vehicle using Multibody Simulation and Experimental Data</h1>

<img src="https://user-images.githubusercontent.com/38962235/90955651-acd76c00-e47f-11ea-93b7-9d495311a948.PNG" alt="Italian Trulli">

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
