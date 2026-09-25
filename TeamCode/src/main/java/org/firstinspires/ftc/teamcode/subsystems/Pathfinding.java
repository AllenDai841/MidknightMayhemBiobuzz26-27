package org.firstinspires.ftc.teamcode.subsystems;

public class Pathfinding {
}
/*Pseudocode for pathfinding ig
game starts
if we shoot first, shoot le balls
if not, wait for hive to tip, update the current tip of the hive, start collecting balls
Call move functions until sensor detects ball
Robot receives coordinates of the ball
Robot calls move function to get to the ball
Robot also activates the intake
Robot repeats until it has enough balls (3-4)
Robot then moves into position to fire
Robot fires
U can use apriltags to see if the hive tipped or not
Update most recent hive tip and which hive is down
----------IMPORTANT-----------
If at any point a sensor detects a obstacle
See if it is at a preregistered obstacle point
if is, go around it
if not, it is a robot
Continue going forward at a fast velocity (hopefully butts then out of the way)
If still there, go sideways, then forwards, then continue
----------OTHER----------
If we had the most recent tip
Still collect balls
Once we have collected the balls
position urself in front of the tipped hive
Send the ball beneath the hive (hopefully) giving it to ur teammate
===========================
If robot detects enemy nectar (color sensor)
reverses intake
hits it out of the way ig



*/