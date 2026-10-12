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

@onready var ego_car: Node3D = $EgoCar
@onready var lead_car: Node3D = $LeadCar
@onready var follow_camera: Camera3D = $FollowCamera
@onready var hud_label: Label = $CanvasLayer/HUD

func _ready() -> void:
    reset_simulation()

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

    # --- 6. HUD Update ---
    var scenario_name = "BLOCKED-ROAD (Obstacle at Z=60)" if scenario_blocked_road else "CLEAR-ROAD (Destination B at Z=100)"
    if hud_label:
        hud_label.text = "[EASY DRIVE 3D SIMULATION - TEST 001]\n" + \
            "Scenario: %s\n" % [scenario_name] + \
            "Ego Position Z: %.2f m | Target B: %.1f m\n" % [ego_pos, target_b] + \
            "Ego Speed: %.1f m/s (%.1f km/h)\n" % [ego_speed, ego_speed * 3.6] + \
            "Signed Clearance / Distance: %.2f m\n" % [raw_bumper_sep] + \
            "Obstacle State: %s\n" % [obstacle_state_str] + \
            "Braking State: %s\n" % [braking_state] + \
            "Status: %s\n" % [contact_status] + \
            "Controls: [1] Blocked-Road | [2] Clear-Road | [SPACE] Auto/Manual | [R] Reset"
