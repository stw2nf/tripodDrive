-- Toggle switch triggers servo movement sequence
-- Servos wait at each position for hotshoe feedback
-- that shutter has taken picture

local button_number = 1 -- the button number we want to read, as defined in AP_Button
local trigger_button_state = false
local button_new_state = button:get_button_state(button_number)
local last_button_state = button_new_state

local K_MOUNT_YAW = 6 -- Function Number for Yaw Mount
local K_MOUNT_PITCH = 7 -- Function Number Pitch Mount

local deg2pwm = 2.778 -- Convert degrees to PWM

local YAW_STEP = 36*deg2pwm -- Yaw step in PWM
local PITCH_STEP_UP = 37*deg2pwm -- Pitch step in PWM
local PITCH_STEP_DN = 24*deg2pwm -- Pitch step in PWM

local cur_yaw_step = -1
local cur_pitch_step = false

local trig_hotshoe_state = false
gpio:pinMode(54,0) -- set AUX 5 to input
local camera_feedback = gpio:read(54)
local takePic = false
local picCount = 0 -- Current Counter of what picture we are on
local picTotal = 3 -- Number of pictures per station

local fileNum = -1
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
    button_new_state = button:get_button_state(button_number) -- Switching Toggle exits scan
    if button_new_state ~= trigger_button_state then
        gcs:send_text(0, "Picture Aborted")
        last_button_state = button_new_state
        takePic = false
        return reset_home, 100
    end

    if takePic == false then -- Trigger Servo Pin for Camera Trigger
        rc:run_aux_function(9, 2) -- Trigger Take Picture Auxillary Function
        gcs:send_text(0, "Trigger " ..tostring(picCount+1).." Picture")
        takePic = true
    end
    camera_feedback = gpio:read(54)
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
    button_new_state = button:get_button_state(button_number)-- Switching Toggle exits scan
    if button_new_state ~= trigger_button_state then
        gcs:send_text(0, "Scan Aborted")
        takePic = false
        last_button_state = button_new_state
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
        return take_pic, 500
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
    if file then -- See if there is a file open
        io.close(file) -- Close the currently open file
        file = nil
    end
    gcs:send_text(0, "Resetting Servos to Home")
    return check_button, 1000
end

function check_button() -- Check Toggle switch state

  button_new_state = button:get_button_state(button_number)

  -- the button has changes since the last loop
  if button_new_state ~= last_button_state then
    last_button_state = button_new_state
    if button_new_state == trigger_button_state then
        if file then -- See if there is a file open
            io.close(file) -- Close the currently open file
            file = nil
        end

        fileNum = fileNum + 1 -- Iterate Scan number for new file
        file_name = "Scan"..tostring(fileNum)..".csv"
        file = io.open(file_name, "a") -- Open and append new file
        if not file then
            error("Could not make file")
        else
            gcs:send_text(0, "Starting "..file_name)
        end

        -- write the CSV header
        file:write('Time Stamp(ms), Lat, Long, Pitch (PWM), Yaw (PWM)\n')
        file:flush()

        return step_servo(), 100
    else
        return reset_home, 100
    end
  end

  return check_button, 1000 -- reschedules the loop (1hz)
end

return reset_home, 1000 -- Set Servos to home on boot