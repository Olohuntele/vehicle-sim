#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/binder_common.hpp>

namespace godot {

class EasyDriveBridge : public Node {
    GDCLASS(EasyDriveBridge, Node);

protected:
    static void _bind_methods();

public:
    EasyDriveBridge();
    ~EasyDriveBridge();

    String get_bridge_status() const;
};

} // namespace godot
