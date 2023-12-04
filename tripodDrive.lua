-- Toggle switch triggers servo movement sequence
-- Servos wait at each position for hotshoe feedback
-- that shutter has taken picture
local PARAM_TABLE_KEY = 73

assert(param:add_table(PARAM_TABLE_KEY, "TP_", 8), 'could not add param table')

-- create two parameters. The param indexes (2nd argument) must
-- be between 1 and 63. All added parameters are floats, with the given
-- default value (4th argument).
assert(param:add_param(PARAM_TABLE_KEY, 1, 'YAW_STEP', 36), 'could not add TP_YAW_STEP')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'PTCH_UP', 40), 'could not add TP_PTCH_UP')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'PTCH_DN', 24), 'could not add TP_PTCH_DN')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'PTCH_TRM',74), 'could not add TP_PTCH_TRM')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'MV_DLY', 500), 'could not add TP_MV_DLY')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'PIC_DLY', 10), 'could not add TP_PIC_DLY')
assert(param:add_param(PARAM_TABLE_KEY, 7, 'TIMEOUT', 4000), 'could not add TP_TIMEOUT')
assert(param:add_param(PARAM_TABLE_KEY, 8, 'ARM_WAIT', 15000), 'could not add TP_ARM_WAIT')
-- gcs:send_text(0, string.format("Added 6 parameters"))

local start_button_number = 1 -- Start Button
local stop_button_number = 2 -- Stop Button

local trigger_button_state = false
local start_button_new_state = button:get_button_state(start_button_number)
local stop_button_new_state = button:get_button_state(stop_button_number)

local K_MOUNT_YAW = 6 -- Function Number for Yaw Mount
local K_MOUNT_PITCH = 7 -- Function Number Pitch Mount

local deg2pwm = 2.778 -- Convert degrees to PWM

local YAW_STEP = param:get("TP_YAW_STEP")*deg2pwm -- Yaw step in PWM
local PITCH_STEP_UP = param:get("TP_PTCH_UP")*deg2pwm -- Pitch step in PWM
local PITCH_STEP_DN = param:get("TP_PTCH_DN")*deg2pwm -- Pitch step in PWM

local cur_yaw_step = -1
local cur_pitch_step = false

local trig_hotshoe_state = false
local hotshoe_pin = 50 -- AUX 1 for input
gpio:pinMode(hotshoe_pin,0) -- set AUX 1 to input
local camera_feedback = gpio:read(hotshoe_pin)
local takePic = false
local picCount = 0 -- Current Counter of what picture we are on
local picTotal = 3 -- Number of pictures per station

local yaw_min = 1000
local yaw_max = 2000
local pitch_trim = 1000 + (param:get("TP_PTCH_TRM")*deg2pwm)
local yaw_cmd = yaw_max
local pitch_cmd = pitch_trim

local pitch_pack = 1500 -- Zenith (Straight Up)
local pitch_home = 1000 -- Nadir (Straight Down)
local yaw_pack = 1500
local packPosition = false

local movementDelay = param:get("TP_MV_DLY") -- Amount of time (ms) to wait before triggering first picture of each station, to allow servos to move to location
local nextPicDelay = param:get("TP_PIC_DLY") -- Amount of time (ms) to wait between pictures at a single station

local picCmdTime
local picTimeout = param:get("TP_TIMEOUT") -- Amount of time (ms) to wait between pictures at a single station 

local abortPicture = false
local waitArm = param:get("TP_ARM_WAIT") -- Amount of time (ms) to wait after arming to allow geotagging session to begin

function armVehicle()
    arming:arm()
    return step_servo, waitArm
end

function disarmVehicle()
    arming:disarm()
    return check_button, 1
end

function checkAbort()
    stop_button_new_state = button:get_button_state(stop_button_number) -- Switching Toggle exits scan
    if stop_button_new_state == trigger_button_state then
        gcs:send_text(0, "Picture Aborted")
        abortPicture = true
    end
end

function take_pic()
    checkAbort()
    if abortPicture == true then
        return reset_home, 1
    end

    if takePic == false then -- Trigger Servo Pin for Camera Trigger
        rc:run_aux_function(9, 2) -- Trigger Take Picture Auxillary Function
        gcs:send_text(0, "Trigger "..tostring(picCount+1).." Picture")
        takePic = true
        picCmdTime = millis()
    end

    -- Resend the picture command if we haven't gotten a picture after picTimeout
    if millis() - picCmdTime > picTimeout then
        takePic = false
        gcs:send_text(0, "Trigger "..tostring(picCount+1).." Picture Again")
    end

    camera_feedback = gpio:read(hotshoe_pin)
    if camera_feedback == trig_hotshoe_state and takePic == true then -- Hotshoe Feedback
        picCount = picCount + 1
        gcs:send_text(0, "Picture "..tostring(picCount).." Taken")
        takePic = false
        camera_feedback = true
        if picCount >= picTotal then
            picCount = 0
            return step_servo, 1
        else
            return take_pic, nextPicDelay
        end
    else
        return take_pic, 1
    end
end

function step_servo() -- Step Servo command through Sequence
    checkAbort()
    if abortPicture == true then
        return reset_home, 1000
    end
    cur_yaw_step = cur_yaw_step + 1
    yaw_cmd =  math.floor(yaw_max - cur_yaw_step*YAW_STEP)

    if cur_pitch_step == false then -- Move to 23 degrees below horizon
        pitch_cmd =  math.ceil(pitch_trim - PITCH_STEP_DN)
        cur_pitch_step = true
    else -- Move to 28 degrees above horizon
        pitch_cmd =  math.ceil(pitch_trim + PITCH_STEP_UP)
        cur_pitch_step = false
    end

    if yaw_cmd > yaw_min then
        gcs:send_text(0, "Pitch Angle: "..tostring(math.floor(pitch_cmd/deg2pwm - pitch_trim/deg2pwm)).." Yaw Angle: "..tostring(math.ceil(yaw_max/deg2pwm - yaw_cmd/deg2pwm)))
        SRV_Channels:set_output_pwm(K_MOUNT_YAW, yaw_cmd)
        SRV_Channels:set_output_pwm(K_MOUNT_PITCH, pitch_cmd)
        return take_pic, movementDelay
    else
        return reset_home, 1
    end
end

function reset_home() -- Resets Servos to Home position and resets step counts
    gcs:send_text(0, "Resetting Servos to Home")
    SRV_Channels:set_output_pwm(K_MOUNT_PITCH, pitch_home)
    SRV_Channels:set_output_pwm(K_MOUNT_YAW, yaw_max)
    cur_yaw_step = -1
    picCount = 0
    abortPicture = false
    cur_pitch_step = false
    takePic = false
    packPosition = false
    return disarmVehicle, 1000
end

function pack() -- Resets Servos to Packing Position
    gcs:send_text(0, "Packing Position")
    SRV_Channels:set_output_pwm(K_MOUNT_PITCH, pitch_pack)
    SRV_Channels:set_output_pwm(K_MOUNT_YAW, yaw_pack)
    packPosition = true
    return check_button, 1000
end

function check_button() -- Check Toggle switch state
    -- gcs:send_text(0, "Press a button to trigger")
    updateParams()
    start_button_new_state = button:get_button_state(start_button_number)
    stop_button_new_state = button:get_button_state(stop_button_number)

    if start_button_new_state == trigger_button_state then
        if packPosition == false then -- Not currently in packing position
            return armVehicle, 1
        else
            return reset_home, 100 -- We are in Packing position, send to home position
        end
    end

    if stop_button_new_state == trigger_button_state then
        if packPosition == false then -- Not currently in packing position
            return pack, 100
        end
    end

    return check_button, 50 -- reschedules the loop (1hz)
end

function updateParams()
    YAW_STEP = param:get("TP_YAW_STEP")*deg2pwm
    PITCH_STEP_UP = param:get("TP_PTCH_UP")*deg2pwm
    PITCH_STEP_DN = param:get("TP_PTCH_DN")*deg2pwm
    pitch_trim = 1000 + (param:get("TP_PTCH_TRM")/deg2pwm)
    movementDelay = param:get("TP_MV_DLY")
    nextPicDelay = param:get("TP_PIC_DLY")
    picTimeout = param:get("TP_TIMEOUT")
    waitArm = param:get("TP_ARM_WAIT")
end

return reset_home, 1000 -- Reset to home to start