-- Toggle switch triggers servo movement sequence
-- Servos wait at each position for hotshoe feedback
-- that shutter has taken picture
local secWeek = 604800 -- Number of seconds in a week
local inititalDay = 4 -- Original Day GPS time began
local initialYear = 1980 -- Original Year GPS time began
local utcOffset = 18 -- Offset in seconds of UTC time
local secDay = 86400 -- Extra Seconds in leap year
local utc_sec_year = 31536000 -- Seconds in a year (No leap year)
local utc_sec_leap_year = utc_sec_year + secDay

local days_in_month_non_leap = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
local days_in_month_leap = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
local days_in_month

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
local hotshoe_pin = 50 -- AUX 1 for input
gpio:pinMode(hotshoe_pin,0) -- set AUX 1 to input
local camera_feedback = gpio:read(hotshoe_pin)
local takePic = false
local picCount = 0 -- Current Counter of what picture we are on
local picTotal = 3 -- Number of pictures per station

local file_name
local file

-- index for the data and table
local lat = 1
local long = 2
local pitch = 3
local yaw = 4
local log_data = {}

local yaw_min = 1000
local yaw_max = 2000
local pitch_trim = 1206
local yaw_cmd = yaw_max
local pitch_cmd = pitch_trim

local pitch_pack = 1500 -- Zenith (Straight Up)
local pitch_home = 1000 -- Nadir (Straight Down)
local yaw_pack = 1500
local packPosition = false

local movementDelay = 500 -- Amount of time (ms) to wait before triggering first picture of each station, to allow servos to move to location
local nextPicDelay = 10 -- Amount of time (ms) to wait between pictures at a single station

function calc_DateTime(gps_week, gps_ms)
    local tot_gps_sec = (gps_week*secWeek)+(gps_ms/1000)
    local total_utc_sec = (tot_gps_sec+utcOffset) - utc_sec_year
    local utc_year = 1 -- Years since 1980

    while total_utc_sec >= utc_sec_year do
        if (utc_year % 4 == 0 and (utc_year % 100 ~= 0 or utc_year % 400 == 0)) then
            total_utc_sec = total_utc_sec - utc_sec_leap_year
        else
            total_utc_sec = total_utc_sec - utc_sec_year
        end
        utc_year = utc_year + 1
    end

    if (utc_year % 4 == 0 and (utc_year % 100 ~= 0 or utc_year % 400 == 0)) then
        days_in_month = days_in_month_leap
    else
        days_in_month = days_in_month_non_leap
    end
    
    local utc_days = (total_utc_sec/secDay)+inititalDay
    local utc_sec = total_utc_sec - ((utc_days - inititalDay)*secDay)
    local utc_hour = utc_sec/3600
    local utc_min = (utc_sec - (utc_hour*3600))/60
    --local utc_sec = utc_sec - utc_hour*3600 - utc_min*60
    local month = 1
    while utc_days >= days_in_month[month] do
        utc_days = utc_days - days_in_month[month]
        month = month + 1
    end
    utc_days = utc_days + 1
    
    file_name = tostring(month).."."..tostring(utc_days).."."..tostring(initialYear + utc_year).."_"..tostring(utc_hour).."_"..tostring(utc_min)..".csv"
    file = io.open(file_name, "a") -- Open and append new file
    if not file then
        error("Could not make file")
    else
        gcs:send_text(0, "Starting "..file_name)
    end
    
    -- write the CSV header
    file:write('Time Stamp(ms), Lat, Long, Pitch (PWM), Yaw (PWM)\n')
    file:flush()
    return reset_home, 1000
end

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
        return take_pic, nextPicDelay
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
    camera_feedback = gpio:read(hotshoe_pin)
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
        return take_pic, movementDelay
    else
        return reset_home, 1000
    end
end

function reset_home() -- Resets Servos to Home position and step counts
    gcs:send_text(0, "Resetting Servos to Home")
    SRV_Channels:set_output_pwm(K_MOUNT_PITCH, pitch_home)
    SRV_Channels:set_output_pwm(K_MOUNT_YAW, yaw_max)
    cur_yaw_step = -1
    cur_pitch_step = false
    takePic = false
    packPosition = false
    return check_button, 1000
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

function wait_GPS()
    local curr_loc = ahrs:get_location() -- Create location object at current location
    if curr_loc == nil then
        gcs:send_text(0, "Waiting for GPS Fix")
        return wait_GPS, 1000
    else
        local week = gps:time_week(0)
        local week_sec = gps:time_week_ms(0)
        return calc_DateTime(week, week_sec)
    end
end

return wait_GPS, 1000 -- Wait for GPS fix to open timestamped log file