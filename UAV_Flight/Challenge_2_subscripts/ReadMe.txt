These scripts are not meant to be the actual challenge 2 scripts. 
They are only here for testing each subdivided task required for challenge 2.

Challenge 2: ArUco Pathfinding

Part 1: Takeoff
- UGV is stationary at this moment
- UAV arms and takes off to +4 ft in the air

Part 2: Searching
- UGV is still not moving at this moment
- UAV will being the search pattern to find the goal aruco marker

    Seaching Algorithm: Snake pattern
    - The drone will move in a snake-like pattern across the whole field
    - As the drone moves, it will constantly update its current position in the form of (x,y)
        - When the drone finds the goal, it will tell the UGV its coords and will fly back to the UGV
    Steps:
    1. Drone moves forward ~15 yards
    2. Drone turns 90 degrees left/right (depending on the starting location)
    3. Drone moves forward 5 feet
    4. Drone turns 90 degrees left/right (do the same direction as step 2, this should make a U shape)
    5. Drome moves forward ~15 yards
    6. Drone turns 90 degrees right/left (do the opposite direction as steps 2 & 4)
    7. Drone moves forward 5 feet
    8. Drone turn 90 degrees right/left (choose the same direction as step 6)
    9. Repeat until goal ArUco marker is found

Part 3: Communication
- UAV sends the coords of the marker is it flying above to the UGV
- UGV begins moving towards the coords at a rate of >0.2 mph

Part 4: Reconnect/Descent
- UGV continues moving towards the goal
- UAV flies back to the starting position, aka (0,0), and searchs for the UGV on its way back
    - When the UAV finds the UGV's ArUco marker, the drone will begin the landing procedure to ensure it lands on the UGV properly

Part 5: Completion
- As a unit, the UGV carries the drone to the goal and stops moving once it it within the 5 feet radius of the goal marker

