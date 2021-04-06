function sysCall_init()

    -- this gets the handle to the object owning the script so RobotBase in this case
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    
    -- Note the faster the right motor the more LEFT the robot turns (Assuming zero speed for left motor)
    lJoint = sim.getObjectHandle('LeftMotor')
    rJoint = sim.getObjectHandle('RightMotor')
    
    --sensors
    frontSonar = sim.getObjectHandle('frontSonar')
    rightSonar = sim.getObjectHandle('rightSonar')
    leftSonar = sim.getObjectHandle('leftSonar')
    frontCamera = sim.getObjectHandle('frontCamera')
    frontCamera2 = sim.getObjectHandle('frontCamera2')

    -- Constants
    sonarMaxRange = 0.4 
    rawSpeed = 10
    distanceBtwnWheels = 1.0300e-01
    wheelRadius = 6.0000e-02
    -- negative for right turn positive for left turn
    turnRate = 0.1
    minAllowedDistanceFromWall = 0.32
    sideDistanceLimit = 0.27
    
    --variables
    angularVelocity = 0
    velocityRight = 0
    velocityLeft = 0
    
    sim.setJointTargetVelocity(lJoint, rawSpeed)
    sim.setJointTargetVelocity(rJoint, rawSpeed)
    
    -- launch the ros client
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos, "Ros interface found konoyaro")
        -- prepare topic names
        local symtime = sim.getSystemTime(-1) * 10000
        local leftMotorTopicName = "leftMotorSpeed" .. symtime
        local rightMotorTopicName = "rightMotorSpeed" .. symtime
        local sensorTopicName = "sensorTrigger" .. symtime
        local simulationTimeTopicName = "simtime" .. symtime
        local cameraTopicName = 'hbot_camera'
        local cameraTopicName = 'camera/image_raw' -- quick change to test orbslam, change later
        local cameraTopicName2 = 'hbot_camera2'

        -- publishers
        sensorPub = simROS.advertise('/'..sensorTopicName, 'std_msgs/Bool')
        simTimePub = simROS.advertise('/'..simulationTimeTopicName, 'std_msgs/Float32')
        cameraPub=simROS.advertise('/'..cameraTopicName, 'sensor_msgs/Image')
        cameraPub2=simROS.advertise('/'..cameraTopicName2, 'sensor_msgs/Image')
        -- camera topic info
        cameraInfoPub=simROS.advertise('/camera/camera_info', 'sensor_msgs/CameraInfo')
        --odometryPub=simROS.advertise('/odom', 'nav_msgs/Odometry')
        encoderPub=simROS.advertise('/sim_encoder', 'std_msgs/Float32MultiArray')

        --prepare subscribers (comment like and subscribe ;) )
         simROS.publisherTreatUInt8ArrayAsString(cameraPub) 
         simROS.publisherTreatUInt8ArrayAsString(cameraPub2)
        -- simROS.publisherTreatUInt8ArrayAsString(cameraInfoPub)
         -- treat uint8 arrays as strings (much faster, tables/arrays are slow in Lua)
         
         --left_ns = "/left_wheel"
         --right_ns = "/right_wheel"
         --rightWheelSub = simROS.subscribe(right_ns.."/control_effort", 'std_msgs/Float64', right_wheel_velocity_callback)
        -- leftWheelSub = simROS.subscribe(left_ns.."/control_effort", 'std_msgs/Float64', left_wheel_velocity_callback)
         wheelControlSub = simROS.subscribe('/vel_wheels', 'std_msgs/Float32MultiArray', 'controlWheels')        
    
        -- start the client if running from external script.
        --result = sim.launchExecutable(
    else
        sim.addLog(sim.verbosity_scripterrors, "ROS interface was not found. Cannot run.")
        error("simRos isnt true, was roscore running before launching copelliasim?")
    end
    
end
function getOdom()
    local result, wL, wR
    result, wL = sim.getObjectFloatParameter(lJoint, sim.jointfloatparam_velocity)
    result, wR = sim.getObjectFloatParameter(rJoint, sim.jointfloatparam_velocity)
    -- wL and wR are angular velocity, should look the same as velocity set in the initfunctino
    -- if everything is correctly set up
    local msg={}
    msg['data'] = {wR, wL}
    simROS.publish(encoderPub, msg )
    --print({"vr: ", wR, "vl", wL})
end

function controlWheels(msg)
-- these are the messages from the ros controller
    local vr = msg['data'][1]
    local vl = msg['data'][2]
    print({"ros_sub, vr: ", vr, "vl:", vl})
    sim.setJointTargetVelocity(rJoint, vr)
    sim.setJointTargetVelocity(lJoint, vl)
end

function sysCall_actuation()
    -- put your actuation code here
end

function publish_camerainfo(w, h)

    view_angle = 60*math.pi/180
    --sim.visionfloatparam_perspective_angle (1004): float parameter : perspective projection angle
    viewing_angle_id = 1004
    sim.getObjectFloatParameter(frontCamera, viewing_angle_id, view_angle)
    f_x = (w/2)/math.tan(view_angle/2);
    f_y = f_x;
    CameraInfo={}
    CameraInfo['header']={seq=0,stamp=sim.getSystemTime(), frame_id="camera_link"}
    CameraInfo['height']=h
    CameraInfo['width']=w
    CameraInfo['distortion_model']="plumb_bob"
    CameraInfo['D']={0, 0, 0, 0, 0}
   -- CameraInfo['K']={f_x, 0, w/2, 0, f_y, h/2, 0, 0, 1}
   -- CameraInfo['R']={1, 0, 0, 0, 1, 0, 0, 0, 1}
   -- CameraInfo['P']={f_x, 0, w/2, 0, 0, f_y, h/2, 0, 0, 0, 1, 0}
   CameraInfo['K']={0, 0, 0, 0, 0, 0, 0, 0, 1}
   CameraInfo['R']={1, 0, 0, 0, 1, 0, 0, 0, 1}
   CameraInfo['P']={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}
    CameraInfo['binning_x']= 1
    CameraInfo['binning_y']= 1
    CameraInfo['roi']= {x_offset=0, y_offset=0, width = 0, height = 0, do_rectify= false}
    simROS.publish(cameraInfoPub,CameraInfo)

end

function sysCall_sensing()
        if simROS then   
        -- Publish the image of the vision sensors
        local data,w,h=sim.getVisionSensorCharImage(frontCamera)
        d={}
        d['header']={stamp=simROS.getTime(), frame_id="a"}
        d['height']=h
        d['width']=w
        d['encoding']='rgb8'
        d['is_bigendian']=1
        d['step']=w*3
        d['data']=data
        simROS.publish(cameraPub,d)
        -- I am only using one camera for now, so only publishh its info
        publish_camerainfo(w,h)
        getOdom()
--        print({"vr: ", sim.getObjectVelocity(rJoint), "vl", sim.getObjectVelocity(lJoint)})

        -- now for the second camera
        local data2,w2,h2=sim.getVisionSensorCharImage(frontCamera2)
        d2={}
        d2['header']={stamp=simROS.getTime(), frame_id="b"}
        d2['height']=h2
        d2['width']=w2
        d2['encoding']='rgb8'
        d2['is_bigendian']=1
        d2['step']=w*3
        d2['data']=data2
        simROS.publish(cameraPub2, d2)
    end
    
        frontDistance = getSonarDistance(frontSonar)
        rightDistance = getSonarDistance(rightSonar)
        leftDistance = getSonarDistance(leftSonar)
        
        --print({frontDistance, rightDistance, leftDistance})
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
        --print(sim.getObjectVelocity(lJoint))

end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details

function turnToFurthestWall(frontDistance, rightDistance, leftDistance)
    -- right distance from wall is largest
    if (rightDistance > leftDistance and rightDistance>frontDistance) then
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
    if (rightDistance < sideDistanceLimit or leftDistance < sideDistanceLimit) then
        mul = 100
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
    --print('turning ' .. dir)
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

function getTransformStamped(objHandle,name,relTo,relToName)
    -- This function retrieves the stamped transform for a specific object
    t=sim.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end
----- abanodoned functions kept for ref only below####
function publish_odometry()
--- abandoned, doenst work kept for reference. odom publishing will be in the ros node not here
    tf_world_car=getTransformStamped(robotHandle,'base_link',-1,'world')
    linearVelocity, angularVelocity= sim.getObjectVelocity(robotHandle)

    odom = 
    {
        header=
        {
            stamp= sim.getSimulationTime(),
            frame_id= 'world'
        },
        child_frame_id= 'base_link',
        pose=   
        {
            pose=  --use tf information
            {
                position= 
                {
                    x=tf_world_car.transform.translation.x,
                    y=tf_world_car.transform.translation.y,
                    z=tf_world_car.transform.translation.z
                },
                orientation= 
                {
                    x=tf_world_car.transform.rotation.x,
                    y=tf_world_car.transform.rotation.y,
                    z=tf_world_car.transform.rotation.z,
                    w=tf_world_car.transform.rotation.w
                }
            }
        },
        twist=
        {
            twist=
            {
                linear=
                {
                    x=linearVelocity[1],
                    y=linearVelocity[2],
                    z=linearVelocity[3]
                },
                angular=
                {
                    x=angularVelocity[1],
                    y=angularVelocity[2],
                    z=angularVelocity[3]
                }
            }
        }
    }
    simROS.publish(odometryPub, odom)
end


function right_wheel_velocity_callback(msg)
    print("right msg")
    --local vr = msg['data'][1]
    --wL = msg['data'][2]
   -- print({"ros_sub, vr: ", vr})
end
function left_wheel_velocity_callback(msg)
    print("left")
    --local vl = msg['data'][1]
    --wL = msg['data'][2]
    --print({"ros_sub, vl: ", vl})
end
