extends Node3D

@export var ego_max_accel: float = 3.5
@export var ego_max_brake: float = 6.0

var ego_pos: float = 0.0
var ego_speed: float = 0.0
var lead_pos: float = 60.0  # Blocked road obstacle at Z=60 by default
var lead_speed: float = 0.0

var target_b: float = 100.0
var scenario_blocked_road: bool = true
var auto_drive: bool = true
var ego_length: float = 4.0
var lead_length: float = 4.0

var cpp_bridge = null

@onready var ego_car: Node3D = $EgoCar
@onready var lead_car: Node3D = $LeadCar
@onready var follow_camera: Camera3D = $FollowCamera
@onready var hud_label: Label = $CanvasLayer/TelemetryPanel/HUD

func _ready() -> void:
    if ClassDB.class_exists("EasyDriveBridge"):
        cpp_bridge = ClassDB.instantiate("EasyDriveBridge")
    reset_simulation()

func _exit_tree() -> void:
    if cpp_bridge:
        cpp_bridge.free()
        cpp_bridge = null

func reset_simulation() -> void:
    ego_pos = 0.0
    ego_speed = 0.0
    if scenario_blocked_road:
        lead_pos = 60.0
        lead_speed = 0.0
        if lead_car:
            lead_car.visible = true
    else:
        lead_pos = 999.0  # Far away / inactive for clear road
        lead_speed = 0.0
        if lead_car:
            lead_car.visible = false

func _process(delta: float) -> void:
    # --- 1. Keyboard Controls & Scenario Switching ---
    if Input.is_key_pressed(KEY_1):
        scenario_blocked_road = true
        reset_simulation()
    elif Input.is_key_pressed(KEY_2):
        scenario_blocked_road = false
        reset_simulation()

    if Input.is_action_just_pressed("ui_accept") or Input.is_key_pressed(KEY_SPACE):
        auto_drive = !auto_drive

    if Input.is_key_pressed(KEY_R):
        reset_simulation()

    var manual_input: float = 0.0
    if Input.is_action_pressed("ui_up") or Input.is_key_pressed(KEY_W):
        manual_input = 1.0
    elif Input.is_action_pressed("ui_down") or Input.is_key_pressed(KEY_S):
        manual_input = -1.0

    # --- 2. Autonomous Control Calculation (Easy Drive Test 001) ---
    var accel: float = 0.0
    var braking_state: String = "COASTING"
    var obstacle_state_str: String = "CLEAR"

    if auto_drive && manual_input == 0.0:
        if scenario_blocked_road:
            # Blocked Road Scenario: Obstacle at Z=60
            var lead_rear: float = lead_pos - lead_length / 2.0
            var ego_front: float = ego_pos + ego_length / 2.0
            var bumper_sep: float = lead_rear - ego_front

            # Braking distance: v^2 / (2 * max_brake) + safety margin (2.5m) + step buffer
            var braking_dist: float = (ego_speed * ego_speed) / (2.0 * ego_max_brake) + 2.5 + ego_speed * delta

            if bumper_sep <= braking_dist || ego_pos >= lead_pos - 12.0:
                accel = -ego_max_brake
                braking_state = "AEB / PLANNED BRAKING"
                obstacle_state_str = "OBSTACLE DETECTED (APPROACHING)"
            elif ego_speed < 10.0:
                accel = ego_max_accel
                braking_state = "ACCELERATING"
                obstacle_state_str = "OBSTACLE AHEAD (Z=60)"
            else:
                accel = 0.0
                braking_state = "CRUISING"
                obstacle_state_str = "OBSTACLE AHEAD (Z=60)"
        else:
            # Clear Road Scenario: Stop at Point B (Z=100)
            var dist_to_b: float = target_b - ego_pos
            var braking_dist: float = (ego_speed * ego_speed) / (2.0 * ego_max_brake) + ego_speed * delta

            if dist_to_b <= braking_dist + 1.0 || ego_pos >= target_b:
                accel = -ego_max_brake
                braking_state = "STOPPING AT DESTINATION B"
                obstacle_state_str = "CLEAR ROAD"
            elif ego_speed < 10.0:
                accel = ego_max_accel
                braking_state = "ACCELERATING TO B"
                obstacle_state_str = "CLEAR ROAD"
            else:
                accel = 0.0
                braking_state = "CRUISING TO B"
                obstacle_state_str = "CLEAR ROAD"
    else:
        # Manual Input Override
        if manual_input > 0.0:
            accel = manual_input * ego_max_accel
            braking_state = "MANUAL ACCEL"
        elif manual_input < 0.0:
            accel = manual_input * ego_max_brake
            braking_state = "MANUAL BRAKE"
        else:
            accel = -0.5 # drag/coasting
            braking_state = "COASTING"

    # --- 2.5 C++ Shadow Evaluation (Shadow Mode Only) ---
    var shadow_result = {}
    if cpp_bridge and cpp_bridge.has_method("evaluate_shadow_step"):
        var obs_detected = scenario_blocked_road
        var obs_pos = lead_pos if scenario_blocked_road else 999.0
        var obs_speed = lead_speed if scenario_blocked_road else 0.0
        shadow_result = cpp_bridge.evaluate_shadow_step(
            ego_pos, ego_speed, accel,
            obs_detected, obs_pos, obs_speed,
            auto_drive, manual_input, target_b
        )

    # --- 3. Kinematics & Position Update ---
    ego_speed = max(0.0, ego_speed + accel * delta)
    ego_pos += ego_speed * delta

    if scenario_blocked_road:
        lead_pos += lead_speed * delta

    # --- 4. Update 3D Transforms ---
    if ego_car:
        ego_car.position = Vector3(0.0, 0.4, ego_pos)
    if lead_car and scenario_blocked_road:
        lead_car.position = Vector3(0.0, 0.4, lead_pos)

    # Follow camera
    if follow_camera:
        follow_camera.position = Vector3(0.0, 4.0, ego_pos - 10.0)
        follow_camera.look_at(Vector3(0.0, 1.0, ego_pos + 15.0), Vector3.UP)

    # --- 5. Bumper-to-Bumper Clearance Calculation ---
    var raw_bumper_sep: float = 0.0
    if scenario_blocked_road:
        var lead_rear: float = lead_pos - lead_length / 2.0
        var ego_front: float = ego_pos + ego_length / 2.0
        raw_bumper_sep = lead_rear - ego_front
    else:
        raw_bumper_sep = target_b - ego_pos

    var contact_status = "SAFE DISTANCE"
    if scenario_blocked_road:
        if raw_bumper_sep <= 0.0:
            contact_status = "!! CONTACT / COLLISION DETECTED !!"
        elif raw_bumper_sep < 2.0:
            contact_status = "! WARNING: CLEARANCE < 2.0m !"
        elif ego_speed == 0.0 && ego_pos > 10.0:
            contact_status = "SUCCESS: STOPPED SAFELY (CL >= 2.0m)"
    else:
        if ego_speed == 0.0 && ego_pos >= 98.0:
            contact_status = "SUCCESS: STOPPED AT POINT B (Z=100)"

    # --- 6. HUD Update (GDScript + C++ Shadow Telemetry) ---
    var scenario_name = "BLOCKED-ROAD (Obstacle at Z=60)" if scenario_blocked_road else "CLEAR-ROAD (Destination B at Z=100)"
    
    var s_control = shadow_result.get("control_input", 0.0)
    # --- Stage 3: Controller Comparison ---
    var cpp_equivalent_accel: float = 0.0

    if s_control >= 0.0:
        cpp_equivalent_accel = s_control * ego_max_accel
    else:
        cpp_equivalent_accel = s_control * ego_max_brake

    var accel_difference: float = abs(accel - cpp_equivalent_accel)
    var comparison_status: String = "MATCH"
    var comparison_detail: String = ""

    if shadow_result.is_empty():
        comparison_status = "C++ SHADOW UNAVAILABLE"
    elif ego_speed < 0.05 and accel <= 0.0 and cpp_equivalent_accel <= 0.0:
        comparison_status = "STATIONARY - BRAKING / HOLDING"
    elif accel < -0.05 and cpp_equivalent_accel < -0.05:
        comparison_status = "BOTH BRAKING"
    elif accel > 0.05 and cpp_equivalent_accel > 0.05:
        comparison_status = "BOTH ACCELERATING"
    elif accel > 0.05 and cpp_equivalent_accel < -0.05:
        comparison_status = "OPPOSITE DECISIONS"
    elif accel < -0.05 and cpp_equivalent_accel > 0.05:
        comparison_status = "OPPOSITE DECISIONS"
    elif accel_difference > 0.5:
        comparison_status = "DIFFERENT DECISIONS"

    if not shadow_result.is_empty():
        comparison_detail = "Intensity difference: %.2f m/sÂ²" % accel_difference
    var s_aeb = shadow_result.get("aeb_triggered", false)
    var s_acc = shadow_result.get("acc_active", false)
    var s_clearance = shadow_result.get("bumper_clearance", 0.0)
    var s_stop_dist = shadow_result.get("estimated_stopping_distance", 0.0)
    var s_warning_idx = shadow_result.get("warning_level", 0)
    var s_braking_idx = shadow_result.get("braking_state", 0)

    var shadow_braking_strs = ["ACCELERATING", "CRUISING", "PLANNED_DECEL", "EMERGENCY_AEB", "STOPPING_AT_DESTINATION", "COASTING"]
    var shadow_warning_strs = ["BLUE (Normal)", "YELLOW (Caution)", "RED (AEB/Emergency)"]
    var s_braking_str = shadow_braking_strs[s_braking_idx] if s_braking_idx >= 0 and s_braking_idx < shadow_braking_strs.size() else "UNKNOWN"
    var s_warning_str = shadow_warning_strs[s_warning_idx] if s_warning_idx >= 0 and s_warning_idx < shadow_warning_strs.size() else "BLUE"

    if hud_label:
        hud_label.text = "[EASY DRIVE 3D SIMULATION - TEST 001]\n" + \
            "Scenario: %s\n" % [scenario_name] + \
            "Ego Position Z: %.2f m | Target B: %.1f m\n" % [ego_pos, target_b] + \
            "Ego Speed: %.1f m/s (%.1f km/h)\n" % [ego_speed, ego_speed * 3.6] + \
            "Signed Clearance / Distance: %.2f m\n" % [raw_bumper_sep] + \
            "Obstacle State: %s\n" % [obstacle_state_str] + \
            "Braking State (GDScript): %s\n" % [braking_state] + \
            "Status: %s\n" % [contact_status] + \
            "----------------------------------------\n" + \
            "C++ SHADOW TELEMETRY (Shadow Mode):\n" + \
            "  - Braking State: %s\n" % [s_braking_str] + \
            "  - Warning Level: %s\n" % [s_warning_str] + \
            "  - Control Input: %.2f | ACC Active: %s\n" % [s_control, s_acc] + \
            "  - AEB Triggered: %s\n" % [s_aeb] + \
            "  - Bumper Clearance: %.2f m\n" % [s_clearance] + \
            "  - Est. Stopping Dist: %.2f m\n" % [s_stop_dist] + \
            "----------------------------------------\n" + \
            "CONTROLLER COMPARISON:\n" + \
            "  GDScript Accel: %.2f m/sÃ‚Â²\n" % [accel] + \
            "  C++ Equivalent Accel: %.2f m/sÃ‚Â²\n" % [cpp_equivalent_accel] + \
            "  Difference: %.2f m/sÃ‚Â²\n" % [accel_difference] + \
            "  Comparison: %s\n" % [comparison_status] + \
            "  Detail: %s\n" % [comparison_detail] + \
            "----------------------------------------\n" + \
            "Controls: [1] Blocked-Road | [2] Clear-Road | [SPACE] Auto/Manual | [R] Reset"
