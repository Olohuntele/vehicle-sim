extends SceneTree

func _initialize() -> void:
    print("--- EASY DRIVE GDEXTENSION SMOKE TEST START ---")
    var bridge = ClassDB.instantiate("EasyDriveBridge")
    if not bridge:
        print("SMOKE TEST FAILED: Could not instantiate EasyDriveBridge class.")
        print("--- EASY DRIVE GDEXTENSION SMOKE TEST END ---")
        quit(1)
        return

    print("Successfully instantiated EasyDriveBridge!")

    # 1. Test get_bridge_status()
    if bridge.has_method("get_bridge_status"):
        var status = bridge.get_bridge_status()
        print("get_bridge_status() returned: ", status)
        if status != "C++ GDExtension connected":
            print("SMOKE TEST FAILED: Unexpected status string: ", status)
            bridge.free()
            quit(1)
            return
    else:
        print("SMOKE TEST FAILED: get_bridge_status method not found")
        bridge.free()
        quit(1)
        return

    # 2. Test evaluate_shadow_step method existence
    if not bridge.has_method("evaluate_shadow_step"):
        print("SMOKE TEST FAILED: evaluate_shadow_step method not found on EasyDriveBridge")
        bridge.free()
        quit(1)
        return

    # 3. Test Clear-Road Driving Scenario
    var clear_result = bridge.evaluate_shadow_step(0.0, 5.0, 0.0, false, 999.0, 0.0, true, 0.0, 100.0)
    print("Clear-road evaluation result: ", clear_result)
    if typeof(clear_result) != TYPE_DICTIONARY:
        print("SMOKE TEST FAILED: evaluate_shadow_step did not return a Dictionary (clear road)")
        bridge.free()
        quit(1)
        return

    var expected_keys = [
        "control_input", "aeb_triggered", "acc_active", "relative_distance",
        "relative_speed", "ttc", "bumper_clearance", "estimated_stopping_distance",
        "warning_level", "braking_state", "collision_detected"
    ]
    for key in expected_keys:
        if not clear_result.has(key):
            print("SMOKE TEST FAILED: Missing expected Dictionary key: ", key)
            bridge.free()
            quit(1)
            return

    print("Clear-road Dictionary keys validated successfully.")

    # 4. Test Stationary Obstacle Ahead Scenario (Obstacle at Z=60)
    var obstacle_result = bridge.evaluate_shadow_step(0.0, 10.0, 0.0, true, 60.0, 0.0, true, 0.0, 100.0)
    print("Stationary obstacle evaluation result: ", obstacle_result)
    if typeof(obstacle_result) != TYPE_DICTIONARY:
        print("SMOKE TEST FAILED: evaluate_shadow_step did not return a Dictionary (obstacle ahead)")
        bridge.free()
        quit(1)
        return

    for key in expected_keys:
        if not obstacle_result.has(key):
            print("SMOKE TEST FAILED: Missing expected Dictionary key in obstacle test: ", key)
            bridge.free()
            quit(1)
            return

    print("Obstacle test Dictionary keys validated successfully.")

    # 5. Test Imminent Collision / AEB Scenario (Obstacle at Z=12)
    var aeb_result = bridge.evaluate_shadow_step(10.0, 10.0, 0.0, true, 12.0, 0.0, true, 0.0, 100.0)
    print("AEB obstacle evaluation result: ", aeb_result)
    if typeof(aeb_result) != TYPE_DICTIONARY:
        print("SMOKE TEST FAILED: evaluate_shadow_step did not return a Dictionary (AEB test)")
        bridge.free()
        quit(1)
        return

    bridge.free()
    print("SMOKE TEST PASSED: All native C++ shadow evaluation and dictionary validation tests passed successfully!")
    print("--- EASY DRIVE GDEXTENSION SMOKE TEST END ---")
    quit(0)
