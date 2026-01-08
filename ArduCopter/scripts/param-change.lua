-- RC switch channel and threshold for preset selection
-- PWM > TH selects preset A, PWM <= TH selects preset B
local SW_CH=6; local TH=1500

-- Timing constants for I-term management during preset switching
-- I_HOLD_MS: Duration to hold I-terms at minimum after switching (ms)
-- I_RAMP_MS: Duration to ramp I-terms from minimum to target (ms)
-- LOOP_MS: Main loop period (ms)
local I_HOLD_MS=500; local I_RAMP_MS=2000; local LOOP_MS=50

-- Minimum I-term value to satisfy PreArm parameter validation
-- ATC_RAT_*_ILMI default is 0.05, so we use 0.05 to ensure PreArm passes
local I_MIN = 0.05

-- Clamp value x between a and b
local function clamp(x,a,b) 
  if x<a then return a elseif x>b then return b else return x end 
end

-- Cache Parameter objects to avoid recreating them each time
local param_cache = {}
local function get_param_obj(name)
  if param_cache[name] == nil then
    param_cache[name] = Parameter(name)
  end
  return param_cache[name]
end

-- Set parameter with error reporting
-- Converts values to float explicitly to avoid uint32_t issues
local function setp(n,v) 
  if v == nil or type(v) ~= "number" then
    return false
  end
  -- Force float conversion
  v = (tonumber(v) or 0.0) + 0.0
  
  -- Set using both Parameter object and param:set() for reliability
  local p = get_param_obj(n)
  local ok1 = p:set(v)
  local ok2 = param:set(n, v)
  
  return ok1 and ok2
end

-- Check if parameter name is an I-term (ends with _I or _IMAX)
local function isp_i(name) 
  return (string.find(name,"_I$")~=nil) or (string.find(name,"_IMAX$")~=nil) 
end

-- Preset A: SITL default values for standard quadcopter (no payload)
local PRESET_A = {
  ATC_RAT_RLL_P=0.15, ATC_RAT_RLL_I=0.2, ATC_RAT_RLL_D=0.003,
  ATC_RAT_PIT_P=0.15, ATC_RAT_PIT_I=0.2, ATC_RAT_PIT_D=0.003,
  ATC_RAT_YAW_P=0.2, ATC_RAT_YAW_I=0.1, ATC_RAT_YAW_D=0.0,
  ATC_ANG_RLL_P=4.5, ATC_ANG_PIT_P=4.5, ATC_ANG_YAW_P=4.0,
  PSC_VELXY_P=2.0, PSC_VELXY_I=1.0, PSC_VELXY_D=0.25,
  PSC_VELZ_P=5.0,  PSC_VELZ_I=2.0,  PSC_VELZ_D=0.0,
  PSC_POSXY_P=1.0, PSC_POSZ_P=1.0
}

-- Preset B: Lower gain PID parameters optimized for heavy payload
-- Reduced gains for smoother, more stable control with increased inertia
local PRESET_B = {
  ATC_RAT_RLL_P=0.10, ATC_RAT_RLL_I=0.12, ATC_RAT_RLL_D=0.002,
  ATC_RAT_PIT_P=0.10, ATC_RAT_PIT_I=0.12, ATC_RAT_PIT_D=0.002,
  ATC_RAT_YAW_P=0.15, ATC_RAT_YAW_I=0.08, ATC_RAT_YAW_D=0.0,
  ATC_ANG_RLL_P=3.5, ATC_ANG_PIT_P=3.5, ATC_ANG_YAW_P=3.2,
  PSC_VELXY_P=1.4, PSC_VELXY_I=0.6, PSC_VELXY_D=0.15,
  PSC_VELZ_P=4.0,  PSC_VELZ_I=1.5,  PSC_VELZ_D=0.0,
  PSC_POSXY_P=0.8, PSC_POSZ_P=0.8
}

-- State machine for preset switching
local state={active=nil, phase="IDLE", t0=0}
local i_targets={}

-- Get human-readable label for current phase
local function phase_label()
  if state.phase=="HOLD_I" then return "I: HOLD"
  elseif state.phase=="RAMP_I" then return "I: RAMP"
  else return "I: ON" end
end

-- Generate HUD status text showing current preset and I-term phase
local function hud_text()
  local a = state.active or "-"
  return "PID PRESET: "..a.." ("..phase_label()..")"
end

-- Send HUD message to GCS only on state changes
local function hud_push()
  gcs:send_text(6, hud_text())
end

-- Extract I-term target values from preset for later ramping
local function build_targets(preset)
  i_targets={}
  for k,v in pairs(preset) do 
    if isp_i(k) and v ~= nil and type(v) == "number" then
      i_targets[k]=v
    end
  end
end

-- Set all I-terms to minimum value (freeze integrators to prevent windup)
local function freeze_i()
  for name,_ in pairs(i_targets) do 
    setp(name, I_MIN) 
  end
end

-- Apply all non-I parameters (P, D, etc.) immediately
local function apply_non_i(preset)
  for k,v in pairs(preset) do 
    if not isp_i(k) then 
      setp(k,v) 
    end 
  end
end

-- Apply I-terms with ramped scaling (alpha: 0.0 = minimum, 1.0 = full target)
local function apply_i_ramped(alpha)
  -- Ensure alpha is a valid number
  if alpha == nil or type(alpha) ~= "number" then
    alpha = tonumber(alpha) or 0.0
  end
  alpha = clamp((alpha + 0.0), 0.0, 1.0)  -- Force float and clamp
  
  for name,target in pairs(i_targets) do 
    if name ~= nil and target ~= nil and type(target) == "number" then
      target = (tonumber(target) or 0.0) + 0.0  -- Force float
      local value = I_MIN + (target - I_MIN) * alpha
      setp(name, value)
    end
  end
end

-- Switch to specified preset (A or B)
-- Process: freeze I-terms, apply non-I params, then ramp I-terms gradually
local function switch_to(which)
  local preset = (which=="A") and PRESET_A or PRESET_B
  build_targets(preset)
  freeze_i()
  apply_non_i(preset)
  state.active=which; state.phase="HOLD_I"; state.t0=millis()
  hud_push()
end

-- Main update loop: monitor RC switch and manage preset switching state machine
local function update()
  -- Read RC switch channel; if unavailable, return without messages
  local pwm=rc:get_pwm(SW_CH)
  if pwm==nil then 
    return update,LOOP_MS 
  end
  
  -- Determine desired preset based on switch position
  local want=(pwm>TH) and "A" or "B"
  
  -- Switch preset if changed or on first run
  if state.active==nil or want~=state.active then 
    switch_to(want) 
  end

  -- Manage I-term ramping state machine
  local t=millis()
  if state.phase=="HOLD_I" then
    -- Hold I-terms at minimum for I_HOLD_MS, then start ramping
    local dt = ((tonumber(t) or (t + 0.0)) - (tonumber(state.t0) or (state.t0 + 0.0))) + 0.0
    if dt >= I_HOLD_MS then 
      state.phase="RAMP_I"; 
      state.t0=t; 
      hud_push() 
    end
  elseif state.phase=="RAMP_I" then
    -- Gradually ramp I-terms from minimum to target over I_RAMP_MS
    local t_float = (tonumber(t) or (t + 0.0)) + 0.0
    local t0_float = (tonumber(state.t0) or (state.t0 + 0.0)) + 0.0
    local dt = t_float - t0_float
    local alpha = (dt / I_RAMP_MS) + 0.0  -- Force float conversion
    
    if alpha >= 1.0 then 
      apply_i_ramped(1.0); 
      state.phase="IDLE"; 
      hud_push() 
    else 
      apply_i_ramped(alpha) 
    end
  end

  return update,LOOP_MS
end

-- Return update function and initial delay for ArduPilot Lua scheduler
return update,LOOP_MS
