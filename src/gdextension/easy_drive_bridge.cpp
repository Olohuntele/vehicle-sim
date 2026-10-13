#include "easy_drive_bridge.h"

namespace godot {

EasyDriveBridge::EasyDriveBridge() {
}

EasyDriveBridge::~EasyDriveBridge() {
}

String EasyDriveBridge::get_bridge_status() const {
    return "C++ GDExtension connected";
}

void EasyDriveBridge::_bind_methods() {
    ClassDB::bind_method(D_METHOD("get_bridge_status"), &EasyDriveBridge::get_bridge_status);
}

} // namespace godot
