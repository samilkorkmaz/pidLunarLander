# pidLunarLander
PID demo with lunar lander game

Game live on https://samilkorkmaz.github.io/pidLunarLander/

Use **plotLander.py** to plot saved log file.

TODO:
* In Bang-Bang, calculate switching altitude once instead of continuously
* Variable mass option: If fuel is infinite, mass decrease should stop at a predefined empty vehile mass m0
* Inject error:  
  * Noise in altitude sensor
  * Noise in thrust
* Make thrust realized a first order system so that thrust changes smoothly. Add thrust realized to log.
  
