// SPDX-License-Identifier: Apache-2.0
#include "nomad/vehicle/operation.hpp"

namespace nomad::vehicle {

std::string_view operation_name(VehicleOperation operation) {
    switch (operation) {
    case VehicleOperation::Arm:
        return "arm";
    case VehicleOperation::Disarm:
        return "disarm";
    case VehicleOperation::SetMode:
        return "set mode";
    case VehicleOperation::SetGuidedMode:
        return "set guided mode";
    case VehicleOperation::Takeoff:
        return "takeoff";
    case VehicleOperation::VtolTakeoff:
        return "vtol takeoff";
    case VehicleOperation::GotoLocation:
        return "goto location";
    case VehicleOperation::Land:
        return "land";
    case VehicleOperation::ReturnToLaunch:
        return "return to launch";
    case VehicleOperation::BodyVelocity:
        return "body-frame velocity";
    case VehicleOperation::SetServo:
        return "set servo";
    case VehicleOperation::SetRelay:
        return "set relay";
    case VehicleOperation::MotorTest:
        return "motor test";
    case VehicleOperation::ConfigureGimbal:
        return "configure gimbal";
    case VehicleOperation::SendUserCommand:
        return "send user command";
    case VehicleOperation::ReleasePayload:
        return "release payload";
    case VehicleOperation::FenceConfiguration:
        return "fence configuration";
    }
    return "unknown operation";
}

bool supports_operation(telemetry::AircraftClass aircraft_class, VehicleOperation operation) {
    switch (aircraft_class) {
    case telemetry::AircraftClass::Copter:
        switch (operation) {
        case VehicleOperation::Arm:
        case VehicleOperation::Disarm:
        case VehicleOperation::SetMode:
        case VehicleOperation::SetGuidedMode:
        case VehicleOperation::Takeoff:
        case VehicleOperation::GotoLocation:
        case VehicleOperation::Land:
        case VehicleOperation::ReturnToLaunch:
        case VehicleOperation::BodyVelocity:
        case VehicleOperation::SetServo:
        case VehicleOperation::SetRelay:
        case VehicleOperation::MotorTest:
        case VehicleOperation::ConfigureGimbal:
        case VehicleOperation::SendUserCommand:
        case VehicleOperation::ReleasePayload:
        case VehicleOperation::FenceConfiguration:
            return true;
        }
        return false;
    case telemetry::AircraftClass::Plane:
        return false;
    case telemetry::AircraftClass::QuadPlane:
        switch (operation) {
        case VehicleOperation::Arm:
        case VehicleOperation::SetGuidedMode:
        case VehicleOperation::VtolTakeoff:
            return true;
        case VehicleOperation::Disarm:
        case VehicleOperation::SetMode:
        case VehicleOperation::Takeoff:
        case VehicleOperation::GotoLocation:
        case VehicleOperation::Land:
        case VehicleOperation::ReturnToLaunch:
        case VehicleOperation::BodyVelocity:
        case VehicleOperation::SetServo:
        case VehicleOperation::SetRelay:
        case VehicleOperation::MotorTest:
        case VehicleOperation::ConfigureGimbal:
        case VehicleOperation::SendUserCommand:
        case VehicleOperation::ReleasePayload:
        case VehicleOperation::FenceConfiguration:
            return false;
        }
        return false;
    case telemetry::AircraftClass::Unknown:
        return false;
    }
    return false;
}

} // namespace nomad::vehicle
