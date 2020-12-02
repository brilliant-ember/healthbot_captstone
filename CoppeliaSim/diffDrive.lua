function sysCall_init()
--Note the faster the right motor the more LEFT the robot turns (Assuming zero speed for left motor)
    lJoint = sim.getObjectHandle('LeftMotor')
    rJoint = sim.getObjectHandle('RightMotor')
    
    -- proximity sensors
    frontSonar = sim.getObjectHandle('frontSonar')
    rightSonar = sim.getObjectHandle('rightSonar')
    leftSonar = sim.getObjectHandle('leftSonar')

    -- Constants
    sonarMaxRange = 0.4 
    rawSpeed = 1.5
    distanceBtwnWheels = 1.0300e-01
    wheelRadius = 6.0000e-02
    -- negative for right turn positive for left turn
    turnRate = 0.12
    minAllowedDistanceFromWall = 0.12
    
    --variables
    angularVelocity = 0
    velocityRight = 0
    velocityLeft = 0
    
    sim.setJointTargetVelocity(lJoint, rawSpeed)
    sim.setJointTargetVelocity(rJoint, rawSpeed)
    
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
        frontDistance = getSonarDistance(frontSonar)
        rightDistance = getSonarDistance(rightSonar)
        leftDistance = getSonarDistance(leftSonar)
        
        print({frontDistance, rightDistance, leftDistance})
        freeTbl = getFreeDirection(frontDistance, rightDistance, leftDistance)
        
        if (not freeTbl['front']) then
    
            if(freeTbl['right']) then 
                turn('right')
            elseif (freeTbl['left']) then
                turn('left')
            else
                turnToFurthestWall(frontDistance, rightDistance, leftDistance)
            end
        else
            turn('front')
        end
        
        
        turnAwayFromCloseSideWall(rightDistance, leftDistance)


end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details

function turnToFurthestWall(frontDistance, rightDistance, leftDistance)
    -- right distance from wall is largest
    if (rightDistance > leftDistance and right>frontDistance) then
        turn('right')
    -- left distance is largest
    elseif(leftDistance > rightDistance and leftDistance > frontDistance) then
        turn('left')
    -- front distance is largesr than or equal any of the two sides
    elseif(frontDistance >= rightDistance or frontDistance >= leftDistance) then
        turn('front')
    end
    
end

function getSonarDistance(sensor)
    local detected, distance
    detected, distance = sim.readProximitySensor(sensor)
    if(detected<1) then
        distance = sonarMaxRange
    end
    return distance
end


function turnAwayFromCloseSideWall(rightDistance, leftDistance)
-- if there's a wall to the right or left that is too close, this func will turn thr bot away
-- from that wall
    mul = 1
    if (rightDistance < 0.065 or leftDistance < 0.067) then
        mul = 30
    end
    if (rightDistance <= minAllowedDistanceFromWall) then
        print('rightWall too close')
        turn('left', mul)
    elseif (leftDistance <= minAllowedDistanceFromWall) then
        print('leftWall too close')
        turn('right', mul)
    end

end
function getFreeDirection(frontDistance, rightDistance, leftDistance)
-- if direction is free in  a dir, the table will be true at its key, if direction is not free will be false
-- ex free['front'] ==true if there's no object infront of front sonar
    free = {}
    free['front'] = true
    free['right'] = true
    free['left'] = true
    if frontDistance < sonarMaxRange then
        free['front'] = false
        print("object front")
    end
    if rightDistance < sonarMaxRange then
        free['right'] = false
        print("object right")
    end
    if leftDistance < sonarMaxRange then
        free['left'] = false
        print("object left")
    end
    
    return free
end

function turn(dir, multiplier)
    multiplier = multiplier or 1
    print('turning ' .. dir)
    if (dir == 'front') then
        angularVelocity = 0
    elseif (dir == 'right') then
        angularVelocity = angularVelocity - turnRate * multiplier
    elseif (dir == 'left') then
        angularVelocity = angularVelocity + turnRate * multiplier
    else
        error("invalid Direction to the turn function")
    end

    calculateWheelsVelocities2()
end

function calculateWheelsVelocities()
-- Doesnt work as intended use the 2nd function calculateWheelsVelocities2()
    velocityRight = 0.3*(2*rawSpeed + angularVelocity*distanceBtwnWheels)/(2* wheelRadius)
    velocityLeft = 0.3*(2*rawSpeed - angularVelocity*distanceBtwnWheels)/(2* wheelRadius)
    sim.setJointTargetVelocity(lJoint, velocityLeft)
    sim.setJointTargetVelocity(rJoint, velocityRight)
end

function calculateWheelsVelocities2()
    velocityRight = rawSpeed + (distanceBtwnWheels/2) * angularVelocity
    velocityLeft = rawSpeed - (distanceBtwnWheels/2) * angularVelocity
    sim.setJointTargetVelocity(lJoint, velocityLeft)
    sim.setJointTargetVelocity(rJoint, velocityRight)
end



