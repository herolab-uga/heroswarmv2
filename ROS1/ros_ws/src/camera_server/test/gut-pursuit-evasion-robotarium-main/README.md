# gut-pursuit-evasion-robotarium
Software codes for running the Game-theoretic Utility Tree (GUT) algorithm for the multi-robot Pursuit-Evasion problem in the [Robotarium](https://www.robotarium.gatech.edu/) simulator-hardware multi-robot testbed.

## Abstract
Underlying relationships among multiagent systems (MAS) in hazardous scenarios can be represented as game-theoretic models. In adversarial environments, the adversaries can be intentional or unintentional based on their needs and motivations. Agents will adopt suitable decision-making strategies to maximize their current needs and minimize their expected costs. This paper extends the new hierarchical network-based model, termed [Game-theoretic Utility Tree (GUT)](https://arxiv.org/abs/2004.10950), to arrive at a cooperative pursuit strategy to catch an evader in the Pursuit-Evasion game domain. We verify and demonstrate the performance of the proposed method using the [Robotarium platform](https://www.robotarium.gatech.edu/) compared to the conventional constant bearing (CB) and pure pursuit (PP) strategies. The experiments demonstrated the effectiveness of the GUT, and the performances validated that the GUT could effectively organize cooperation strategies, helping the group with fewer advantages achieve higher performance.

> Paper: [Game-theoretic Utility Tree for Multi-Robot Cooperative Pursuit Strategy](https://arxiv.org/abs/2206.01109)

### GUT Building
<div align = center>
<img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/GUT-PE-overview.png" height="205" alt="GUT-PE-overview"><img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/gut_pursuit_overview.png" height="205" alt="gut_pursuit_overview"/>
</div>

## Experiments Setup
This implementation requires Robotarium Python Simulator.

### Install Robotarium Python Simulator 
Use the original source below and make sure you install it as per their instructions
```
Check the instruction: https://github.com/robotarium/robotarium_python_simulator
```

### Download this GUT-Pursuit repository's Code
```
$ git clone https://github.com/herolab-uga/gut-pursuit-evasion-robotarium.git
```

### To Run the simulations and algorithms
1. Contant Bearing (CB) pursuit strategy with 1/3/5 Pursuers (to capture 1 evader):
```
pyhton pursuit_game_1/3/5vs1_cb.py 
```
2. Pure Pursuit (PP) strategy with 1/3/5 Pursuers (to capture 1 evader):
```
pyhton pursuit_game_1/3/5vs1_pp.py 
```
3. Cooperative Game-theoretic Utility Tree (GUT) based pursuit strategy with 1/3/5 Pursuers (to capture 1 evader):
```
cd ~/pursuit_game
pyhton gut_pursuit_game_1/3/5vs1.py 
```

## Demonstration: `Constant Bearing (CB)` vs `Pure Pursuit (PP)` vs `GUT`
> 1 Pursuer chasing 1 Evader 
    <div align = center>
    <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/pursuit_game_1vs1_cb.gif" height="133" width="237" title="Constant Bearing (CB)">   <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/pursuit_game_1vs1_pp.gif" height="133" width="237" alt="Pure Pursuit (PP)">      <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/gut_pursuit_game_1vs1.gif" height="133" width="237" alt="GUT_1v1"/>
    </div>
    
> 3 Pursuers chasing 1 Evader 
    <div align = center>
    <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/cb.gif" height="133" width="237" title="Constant Bearing (CB)">   <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/pp.gif" height="133" width="237" alt="Pure Pursuit (PP)">      <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/gut_pursuit.gif" height="133" width="237" alt="GUT_3v1"/>
    </div>
    
> 5 Pursuers chasing 1 Evader 
    <div align = center>
    <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/pursuit_game_cb_5vs1.gif" height="133" width="237" title="Constant Bearing (CB)">   <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/pursuit_game_pp_5vs1.gif" height="133" width="237" alt="Pure Pursuit (PP)">      <img src="https://github.com/herolab-uga/gut-pursuit-evasion-robotarium/blob/main/figures/gut_pursuit_game_5vs1.gif" height="133" width="237" alt="GUT_5v1"/>
    </div>


## Core contributors

* **Qin Yang** - Ph.D. student

* **Dr.Ramviyas Parasuraman** - Lab Director


## Heterogeneous Robotics (HeRoLab)

Heterogeneous Robotics Research Lab (HeRoLab) of the University of Georgia.

Please contact hero at uga . edu for any queries

http://hero.uga.edu/

<p align="center">
<img src="http://hero.uga.edu/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>



