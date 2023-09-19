-- Toggle switch triggers servo movement sequence
-- Servos wait at each position for hotshoe feedback
-- that shutter has taken picture

local start_button_number = 1 -- Start Button
local stop_button_number = 2 -- Stop Button

local trigger_button_state = false
local start_button_new_state = button:get_button_state(start_button_number)
local stop_button_new_state = button:get_button_state(stop_button_number)

local K_MOUNT_YAW = 6 -- Function Number for Yaw Mount
local K_MOUNT_PITCH = 7 -- Function Number Pitch Mount

local deg2pwm = 2.778 -- Convert degrees to PWM

local YAW_STEP = 36*deg2pwm -- Yaw step in PWM
local PITCH_STEP_UP = 37*deg2pwm -- Pitch step in PWM
local PITCH_STEP_DN = 24*deg2pwm -- Pitch step in PWM

local cur_yaw_step = -1
local cur_pitch_step = false

local trig_hotshoe_state = false
gpio:pinMode(55,0) -- set AUX 6 to input
local camera_feedback = gpio:read(55)
local takePic = false
local picCount = 0 -- Current Counter of what picture we are on
local picTotal = 3 -- Number of pictures per station

local fileNum = 0
local file_name = "Scan"..tostring(fileNum)..".csv"
local file

-- index for the data and table
local lat = 1
local long = 2
local pitch = 3
local yaw = 4
local log_data = {}

local yaw_min = 1000
local yaw_max = 2000
local pitch_trim = 1625
local yaw_cmd = yaw_max
local pitch_cmd = pitch_trim

local pitch_pack = 1000
local yaw_pack = 1500
local packPosition = false

function write_to_file()
    if not file then
      error("Could not open file")
    end
    local curr_loc = ahrs:get_location() -- Create location object at current location
    if curr_loc == nil then
        log_data[lat] = -1
        log_data[long] = -1 
    else
        log_data[lat] = curr_loc:lat()
        log_data[long] = curr_loc:lng()
    end
    log_data[pitch] = SRV_Channels:get_output_pwm(K_MOUNT_PITCH)
    log_data[yaw] = SRV_Channels:get_output_pwm(K_MOUNT_YAW)

    -- write data
    -- separate with comas and add a carriage return
    file:write(tostring(millis()) .. ", " .. table.concat(log_data,", ") .. "\n")
    -- make sure file is upto date
    file:flush()
    if picCount >= picTotal then
        picCount = 0
        return step_servo, 500
    else
        return take_pic, 1
    end
end

function take_pic()
    stop_button_new_state = button:get_button_state(stop_button_number) -- Switching Toggle exits scan
    if stop_button_new_state == trigger_button_state then
        gcs:send_text(0, "Picture Aborted")
        takePic = false
        return reset_home, 100
    end

    if takePic == false then -- Trigger Servo Pin for Camera Trigger
        rc:run_aux_function(9, 2) -- Trigger Take Picture Auxillary Function
        gcs:send_text(0, "Trigger "..tostring(picCount+1).." Picture")
        takePic = true
    end
    camera_feedback = gpio:read(55)
    if camera_feedback == trig_hotshoe_state and takePic == true then -- Hotshoe Feedback
        picCount = picCount + 1
        gcs:send_text(0, "Picture "..tostring(picCount).." Taken")
        takePic = false
        return write_to_file, 100
    else
        return take_pic, 1
    end
end

function step_servo() -- Step Servo command through Sequence
    stop_button_new_state = button:get_button_state(stop_button_number)-- Switching Stop Button exits scan
    if stop_button_new_state == trigger_button_state then
        gcs:send_text(0, "Scan Aborted")
        takePic = false
        return reset_home, 100
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
        return take_pic, 400
    else
        return reset_home, 1000
    end
end

function reset_home() -- Resets Servos to Home position and step counts
    SRV_Channels:set_output_pwm(K_MOUNT_PITCH, pitch_trim)
    SRV_Channels:set_output_pwm(K_MOUNT_YAW, yaw_max)
    cur_yaw_step = -1
    cur_pitch_step = false
    takePic = false
    packPosition = false
    gcs:send_text(0, "Resetting Servos to Home")
    return check_button, 1000
end

function pack() -- Resets Servos to Packing Position
    SRV_Channels:set_output_pwm(K_MOUNT_PITCH, pitch_pack)
    SRV_Channels:set_output_pwm(K_MOUNT_YAW, yaw_pack)
    packPosition = true
    gcs:send_text(0, "Packing Position")
    return check_button, 1000
end

function check_button() -- Check Toggle switch state

    start_button_new_state = button:get_button_state(start_button_number)
    stop_button_new_state = button:get_button_state(stop_button_number)

    if start_button_new_state == trigger_button_state then
        if packPosition == false then -- Not currently in packing position
            return step_servo, 100
        else
            return reset_home, 100 -- We are in Packing position, send to home position
        end
    end

    if stop_button_new_state == trigger_button_state then
        if packPosition == false then -- Not currently in packing position
            return pack, 100
        end
    end

    return check_button, 500 -- reschedules the loop (1hz)
end

file = io.open(file_name, "a") -- Open and append new file
if not file then
    error("Could not make file")
else
    gcs:send_text(0, "Starting "..file_name)
end

-- write the CSV header
file:write('Time Stamp(ms), Lat, Long, Pitch (PWM), Yaw (PWM)\n')
file:flush()

return reset_home, 1000 -- Set Servos to home on boot