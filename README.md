# ADAS Class8 trucks #

**This repo is work in progress**

## Objective ##
This repo gauges solvers for Non-Linear Model Predictive Control using a simple Cart-pole system - resulting in using these solvers for developing controllers for class8 trucks.

## Contents ##
- This repo contains HVE simulations for class8 long haul trucks crash reconstructions. 
- It contains codes for NMPC cartpole system for analysis of solver performance.
-  It contains controllers for ADAS of class 8 trucks.

## HVE simulations ##
This will be added soon

## Dependencies ##
- [Matlab/Simulink](https://www.mathworks.com/products/matlab.html)
- [Acados Solver](https://github.com/acados/acados) - Please install the solver from their Github page.
- [HVE simulator](https://edccorp.com/index.php/hve-software/hve) - **Requires License**.

## Installation ##
```bash
# Clone the repo
git clone https://github.com/rohanNkhaire/ADAS_class8_trucks.git
```

## Usage ##
```bash
# Run the fmincon cart-pole system
# Run the following file in Matlab
VSC.m
```

## Note ##
The HVE simulations are reconstructed crash scenarios from police reports specifically for class8 long haul trucks between the year 2016 to 2020.