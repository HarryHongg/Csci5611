# Features and timestamp
## Single Agent Navigation, Multiple Agents Planning, and Crowd Simulation
They are shown in the whole video. I used TTC to calculate the avoidance force. It is obviously shown at the bottom left at beginning of the video.  
## Particle system
I implemented particle system which appear on about 20 second of the video. The particles appear from goals as an agent reaches.  
They follow a specific pattern in general. But the movement of a single particle is random.  
## 3D Rendering & Camera
This is shown in the whole video.  
## User Scenario Editing and Realtime User Interaction  
The user scenario editing and realtime user Interaction are shown after 20 seconds of the video.  
Right and left mouse click change the first agent and the first obstacle's positions.  
The path will be planned immediately after the changes as well as the PRM nodes if needed.

# List of Features
Single Agent Navigation, Multiple Agents Planning, Particle system, 3D Rendering & Camera, User Scenario Editing, Realtime User Interaction, Crowd Simulation.

# Tool used
Processing  

# Difficulties
I was having difficulties in implementing a crowd simulation. I didn't know how to apply avoidance forces because the agents are on a fixed track. I was worried that the avoidance force would make them hit an obstacle.  
Until I applied the avoidance forces directly to the agents like in the previous activity. And adjusted the parameters so that they could adjust their positions within a small range to avoid collisions.
