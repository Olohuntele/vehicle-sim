extends SceneTree

func _initialize() -> void:
    print("--- EASY DRIVE GDEXTENSION SMOKE TEST START ---")
    var bridge = ClassDB.instantiate("EasyDriveBridge")
    if bridge:
        print("Successfully instantiated EasyDriveBridge!")
        if bridge.has_method("get_bridge_status"):
            var status = bridge.get_bridge_status()
            print("get_bridge_status() returned: ", status)
            if status == "C++ GDExtension connected":
                print("SMOKE TEST PASSED: Native C++ method invoked successfully!")
                print("--- EASY DRIVE GDEXTENSION SMOKE TEST END ---")
                quit(0)
            else:
                print("SMOKE TEST FAILED: Unexpected status string: ", status)
                print("--- EASY DRIVE GDEXTENSION SMOKE TEST END ---")
                quit(1)
        else:
            print("SMOKE TEST FAILED: get_bridge_status method not found on EasyDriveBridge")
            print("--- EASY DRIVE GDEXTENSION SMOKE TEST END ---")
            quit(1)
        bridge.free()
    else:
        print("SMOKE TEST FAILED: Could not instantiate EasyDriveBridge class.")
        print("--- EASY DRIVE GDEXTENSION SMOKE TEST END ---")
        quit(1)
