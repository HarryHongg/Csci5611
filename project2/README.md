# Features and timestamp
## Cloth Simulation
It is shown in the cloth.mp4. I used a matrix to hold the particles which form the cloth. For each particle, I compute the force from at most eight nearest particles.  
## Air Drag for Cloth
It is shown in the colth.mp4 and air.mp4. After 15 second of the cloth.mp4, it shows vary of winds speed. At beggining of the air.mp4, the cloth falls with air drag. Then, it is blows up and falls without air drag at about 23 second. For each particles, I compute its part of air drag in an fixed area.  
## User Interaction
It is shown in the cloth.mp4. After 15 second of the cloth.mp4, it shows the change of wind speed by keyboard and particles picked by mouse.
## Ripping/Tearing  
It is shown in the Rip.mp4. After 8 second of the Rip.mp4, it shows some particles are ripped from the cloth and the result of the cloth. I used a list like neighbor list to hold connection relationship. Forces between particles are only computed when they are connected. If one particle is too far away from others, it lost connection with others.
## 2D Rigid Body Simulation
It is shown in the rigid_body.mp4. It shows how rigid bodys react with ground and rest on the ground. It is based on rigid body exercise.


# List of Features
Cloth Simulation, Air Drag for Cloth, User Interaction, Ripping/Tearing, 2D Rigid Body Simulation

# Tool used
Processing, PVector

# Difficulties
Collision detection between rigid bodies. It is very difficult to calculate whether a corner has collided with other edges of a rigid body.
