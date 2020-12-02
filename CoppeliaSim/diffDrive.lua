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
        
        freeTbl = getFreeDirection(frontDistance, rightDistance, leftDistance)
        
        if (not freeTbl['front'])then
            twist(-0.12)
        else
            -- subtract angular vlocity from itself to make it zero
            twist(-angularVelocity)
        end    

end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details


function getSonarDistance(sensor)
    local detected, distance
    detected, distance = sim.readProximitySensor(sensor)
    if(detected<1) then
        distance = sonarMaxRange
    end
    return distance
end

function getFreeDirection(frontDistance, rightDistance, leftDistance)
-- if direction is free say fron the free table will be true, if direction is not free will be false
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

function twist(angle)
    angularVelocity = angularVelocity + angle
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



