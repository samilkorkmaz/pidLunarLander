# pidLunarLander
PID and Bang-Bang demo with lunar lander game

Game live on https://samilkorkmaz.github.io/pidLunarLander/

<img width="475" height="289" alt="image" src="https://github.com/user-attachments/assets/ba08aea8-4231-48a0-94fb-97560383320a" />

Use **plotLander.py** to plot saved log files:

<img width="388" height="287" alt="image" src="https://github.com/user-attachments/assets/8697a342-049d-49da-9fed-dd0cb3facda0" />

TODO:
* Add maxThrust as input to Thrust Dynamics panel.
* Initial values for inputs should be read from UI instead of waiting for a UI event to update them.
* Start from horizontal alignment with initial horizontal velocity:
<img width="449" height="408" alt="image" src="https://github.com/user-attachments/assets/4f1f789d-6659-437d-9478-d6476c460f89" />

* Use Powered Descent Guidance (Model Predictive Control - MPC)
* Split code into 1. Physics engine, 2. Controller, 3. Actuator model, 4. Renderer
* In Bang-Bang, calculate switching altitude once instead of continuously --> fragile, see comment in code
* Variable mass option: If fuel is infinite, mass decrease should stop at a predefined empty vehile mass m0
* Inject error:  
  * Noise in altitude sensor
  * Noise in thrust
