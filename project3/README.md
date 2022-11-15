![1](https://user-images.githubusercontent.com/57870719/201820393-538500f7-53c6-4669-91c5-78bebfbd1c25.png)

![2](https://user-images.githubusercontent.com/57870719/201820413-b03840a1-a830-4a99-bf32-c2789d3bc551.png)





https://user-images.githubusercontent.com/57870719/201821473-bf9e2f5c-e03a-4895-9c94-c3bfb3e786d9.mp4



# Features and timestamp
## Single-arm IK and Multi-arm IK  
There are two arms fixed on the shoulders. Their goal of fist is the mouse. The first video shows the move of arms.  
At the beginning of the video, the goal is in the reachable workspace of at least one arm.  
After 9 seconds, the video show the goal is out of reachable workspace.

## Joint limits  
Joint limits are setted by adding minimum and maximum on computed angles
The upper arms cannot pass over head and body. It is showed in the whole video.  
And I also simulated the joint limit of knees. It is showed after 14 second of the video.
If there is not joint limit, the animation is showed in the video of Alternative IK Solver.

## User Interaction
The goal of arms are the position of mouse.  
The goal of legs changes when user press left or right (walking).

## Moving IK and Re-rooting IK
The roots of arms and legs are fixed on the relative positions of body. When the position of body changes, the roots changes.  
Also, as the position of body changes, the X coordinate of goal of legs changes alternatively. The speed is two times of speed of body.  
The lift of a foot is a sin function of the distance of the leg moves.
The video shows walking after 14 seconds.

## Alternative IK Solver
I used FABRIK as the alternatiev IK solver. It does IK based on the positions of joints and end effector. No angle is computed.  
Therefore, FABRIK does not support joint limits on the arms well.  
In the setting of no joint limit, both FABRIK and the original IK performed very well.  
However, original IK contains more calculation compare to FABRIK.
Theoretically, original IK gives solution which does no dependents on current positions. It can gives multiple solutions.
FABRIK find solution based on current position. It only finds one solution.

https://user-images.githubusercontent.com/57870719/201820455-d275b561-67d0-44d7-bb98-7f2ba401e1d3.mp4


# List of Features
Single-arm IK  
Multi-arm IK  
Joint limits  
User Interaction  
Moving IK  
Re-rooting IK  
Alternative IK Solver  
# Tool used
Processing

# Difficulties
It is very hard to set joint limit in FABRIK.  
FABRIK finds positions of joints, then I know the angle by three joints. I cannot set limit on any angle.
