# pidLunarLander
PID demo with lunar lander game

Game live on https://samilkorkmaz.github.io/pidLunarLander/

Use **plotLander.py** to plot saved log file.

TODO:
* Split into 1. Physics engine, 2. Controller, 3. Actuator model, 4. Renderer
* In Bang-Bang, calculate switching altitude once instead of continuously --> fragile, see comment in code
* Variable mass option: If fuel is infinite, mass decrease should stop at a predefined empty vehile mass m0
* Inject error:  
  * Noise in altitude sensor
  * Noise in thrust
