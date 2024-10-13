# How it works
## Mediapipe

Google mediapipe is a great library that can get the coordinates of 21 joints of any hand from a picture or live video feed. They also have other models like full body joint detection and facial feature tracking etc but for this project the hands feature was enough.

Mediapipe captures the points of my hand and i can simply look at the x,y coordinate for the tip of my middle finger to know how to translate the end effector.

The problem comes with depth as an RGB camera does not natively support depth and the use of AruCo markers and calibration would make the system too bloated and not ideal for the average consumer. 

## Calculating depth with just RGB
I came up with a solution that i didnt know existed but it made the most sense to me. The idea is that if you hold an apple close to the camera, it appears bigger (takes up more pixels) and further away it gets smaller. This works because the apples size stays fixed. If the user could keep their hand in some arbitrary fixed position then we could apply some kind of linear regression model to find how the "area" of the hand relates to the true distance from the webcam.

After some time thinking of positions the hand should be in to make the most sense, i decided upon a flat palm with the fingers closed together facing the webcam. This was simple to remember and ensured the model could always track the joints well to reduce anomolous events from happening.


## Enforcing the flat palm
To enforce the flat palm there many options and the standard way would have been to train a data set or do some kind of RNN/CNN approach which frankly i had no idea about. To me this seemed overkill and the easiest way to do it would be to find the angle each link makes with the vertical axis and then find the percentage error between the goal and target values. If they were below, say 5% then the computer would think the person has their palms facing the camera in the right way.

Using angles as opposed to coordinates meant the palm facing the camera would work anywhere within the laptops field of view as opposed to a specific location.

## Finding the "area"
Theres lots of approaches one could go about finding how many pixels the hand takes up or what the area of its outline is. To me those methods seemed computationally expensive and would fail in certain lighting conditions or occlusion. 

I went about finding the mean of the lengths of each link which would obviously increase as you get closer to the camera and decrease further away. This formed a negative correlated graph between hand "area" and true distance from the camera

With some matlab code and the `fminunc` function we found the line representing this correlation and after doing some testing it worked great. The only problem with this approach is that tilting the hand forwards or backwards while keeping your wrist fixed in plae would make the computer think the hand is going closer/further away. To fix this, i intend to use mediapipes z value which is the relative distance the joints are from the wrist. This should take into account the tilting

We ended up with `y = 0.143053x - 0.013' as the equation 

# The ROS2 architecture
## loading the camera
The `camera_controller` node opens up the camera and begins publishing the hands `x,y,z` coordinates as a `Point` message type on the topic `target_positions`

```bash
ros2 run RoboPipe camera_controller
```

## loading the arm to listen to commands and execute
The PyBullet arm is loaded and listens to commands on the `target_posotions` topic with the node `arm_control`

```bash
ros2 run RoboPipe arm_control
```

## Other nodes
One node called `z_sine` publishes messages on the `target_positions` to move the end effector in a sine wave along a straight line. This is useful to test the system or to debug problems

`random_move` publishes random movements every second for the arm to get to within the robots working space

`load_arm` just loads the arm in pybullet but does not listen to commands. useful as a template to work on for other nodes


# Code snippets
## Finding if the hand is a palm
```python
def is_palm(hand_coordinates, recognition_threshold, hand_side):
    percentage_error = 0
    if hand_side == "right":
        target_angles = [-2.52545687852896, -1.9483040078665688, -1.6103067854398168, -1.7002641084906414, 0.4339406264284531, -1.6126579050081975, -1.5747849194347137, -1.5298671963002544, 1.3646553259549035, -1.6218483263299037, -1.5968216501477754, -1.597339725176179, 1.3905249169753788, -1.5950140621519084, -1.5936207638922135, -1.6183316125880256, 1.3856806181487065, -1.560501843787499, -1.6247245652958373, -1.6636383731265538]
    elif hand_side == "left":
        target_angles = [-0.6471584741095117, -1.2338149080123801, -1.5680533711070337, -1.3390927308736622, 2.5762135399244217, -1.5333480331588123, -1.6020179945648463, -1.6217731619466849, 1.7657050993126022, -1.5262688124548656, -1.552629543476536, -1.53839238787029, 1.7539772707880155, -1.5342519286040701, -1.4900642516486404, -1.4658452828809432, 1.7811441058091946, -1.5726404662857638, -1.465885433114545, -1.4255914565456953]
    angles = [] 
    for i in range(21):
        if i != 20:
            x = hand_coordinates[i][0]
            y = hand_coordinates[i][1]
            x_next = hand_coordinates[i+1][0]
            y_next = hand_coordinates[i+1][1]
            angle = math.atan2(y_next-y, x_next-x)
            angles.append(angle)

    for j in range(21):
        if j != 20:
            percentage_error += abs((target_angles[j] - angles[j])/target_angles[j]) * 100

    if percentage_error/20 <= recognition_threshold:
        return True
    else:
        return False
```
## Move KUKA arm to a point
```python
    def move_arm(self, msg):
        targetPos = [msg.x, msg.y, msg.z]
        
        # Perform inverse kinematics to move the end effector to the target position
        jointPoses = p.calculateInverseKinematics(self.robotId, self.endEffectorIndex, targetPos, solver=self.ikSolver,
                                                maxNumIterations=self.maxIter, residualThreshold=self.threshold)

        # Set the joint positions to move the robot arm
        for i in range(len(jointPoses)):
            p.setJointMotorControl2(self.robotId, i, p.POSITION_CONTROL, jointPoses[i])
        p.stepSimulation()
```
# Future improvements
For the futire i want the software to somehow get my hands orientation and not just position. Ideally i want to hand to work outside the frame of the camera and the movement to map 1:1 for real life robots with the users waist origin being the origin of the arm. This could be achieved with a smart bracelet with a 9 axis IMU and position tracking code. The problem is this will drift so the movements have to be quick before re-callibration or we add visual markers around the braclet for a hybrid, computer/IMU design 

The programmed movements should also be able to be replayed and the whole system should scale up to any industrial robot arm. With the help of GPT, i envision that paramaters can be autofilled like how to scale hand movement s based on the size of the robot and what to do when theres an obstacle in the way etc

