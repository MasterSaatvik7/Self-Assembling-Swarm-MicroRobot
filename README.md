# Self-Assembling Swarm MicroRobot Simulation

This project simulates a **self-assembling swarm of microrobots** using **Pygame & Python**. It demonstrates swarm behaviors such as alignment, cohesion, and separation, with customizable parameters and visualization tools.

## Features

- **Swarm Dynamics**: Implements boid-like behaviors for movement and interaction(inspired from the Boid's Algorithm)
- **Formation Following**: Robots can follow predefined or dynamically drawn shapes.
- **Real-Time Visualization**: Uses Pygame for interactive and animated visualization.
- **Configurable Parameters**: Adjust speed, radius, and other attributes to explore various swarm dynamics.

## Requirements

Before running the script, ensure you have the following dependencies installed:

- Python 3.8 or higher(the code was developed in Python 3.11.9)
- pygame==2.6.0
- numpy==1.26.4
- scipy==1.14.0
- scikit-learn==1.5.2

You can install the required libraries using:

```bash
pip install -r requirements.txt
```

## How to Run

1. Clone this repository or download the script.
2. Run the script using Python:

   ```bash
   python Self\ Assembling\ Swarm\ MicroRobot.py
   ```

3. Observe the simulation in the Pygame window.
4. Use Left Mouse button to draw custom shapes for the Microrobots to assemble.
5. Zoom using **+**, **-** Keys

## Parameters

You can modify the following constants in the script to adjust the simulation:

- `SCREEN_WIDTH` and `SCREEN_HEIGHT`: Dimensions of the simulation window.
- `NUM_BOIDS`: Number of swarm agents.
- `MAX_SPEED` and `MAX_FORCE`: Control the speed and agility of agents.
- `NEIGHBOR_RADIUS` and `SEPARATION_RADIUS`: Define interaction ranges.
- `GROUP_RADIUS`: Specifies the radius for group formations.

## Controls

- **Mouse Input**: Draw shapes on the screen for the swarm to follow.
- **Key Input**: Additional interactivity, if specified in the script.

## Visualization

- Agents are represented as moving points or shapes, with colors indicating state or type.
- Formation shapes can be visualized dynamically as they are drawn.

## Simulation Results

![Simulation Results](https://github.com/MasterSaatvik7/Self-Assembling-Swarm-MicroRobot/raw/master/Video/Simulation%20Results.gif)


## Contributing

Feel free to fork this repository and submit pull requests for:

- Enhancements to swarm behavior
- Additional visualization options
- Optimizations and new features

## License

This project is open-source and free to use code.

## Acknowledgments

Inspired by swarm robotics principles and collective behavior research.

---

Let me know if you'd like to customize any section further or add more details!
