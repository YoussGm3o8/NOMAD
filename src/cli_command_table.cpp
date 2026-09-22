// SPDX-License-Identifier: Apache-2.0
#include "cli_command_table.hpp"

#include <cstddef>
#include <iostream>

namespace {

// Order matches the historical usage line, so the client-facing verb list stays
// recognizable to anything that scrapes it.
constexpr CliCommand kCommands[] = {
    {"connect", false, ""},
    {"status", false, ""},
    {"arm", true, ""},
    {"disarm", true, ""},
    {"mode", true, "<custom_mode>"},
    {"takeoff", true, "<altitude_m>"},
    {"vtol-takeoff", true, "<altitude_m>"},
    {"goto", true, "<latitude> <longitude> <altitude_m>"},
    {"land", true, ""},
    {"rtl", true, ""},
    {"servo", true, "<channel> <pwm_us>"},
    {"relay", true, "<number> <0|1>"},
    {"motor-test", true, "<instance> <pwm_us> <timeout_s>"},
    {"gimbal-config", true, "<mount_mode>"},
    {"user-command", true, "<p1> <p2> <p3> <p4> <p5> <p6> <p7>"},
    {"mission-demo", true, ""},
    {"velocity", true, "--vx <m_s> [--vy --vz --yaw-rate] --duration <seconds>"},
    {"velocity-demo", true, ""},
    {"fence-demo", true, ""},
    {"payload-demo", true, "<relay_number> <duration_s>"},
};

} // namespace

std::span<const CliCommand> cli_commands() {
    return kCommands;
}

bool is_supported_command(std::string_view command) {
    for (const auto &entry : cli_commands()) {
        if (entry.name == command) {
            return true;
        }
    }
    return false;
}

bool is_actuation_command(std::string_view command) {
    for (const auto &entry : cli_commands()) {
        if (entry.name == command) {
            return entry.actuation;
        }
    }
    // parse_arguments rejects an unknown verb before this is consulted; an
    // unknown verb is never treated as actuation.
    return false;
}

void print_usage() {
    const auto commands = cli_commands();
    std::cout << "Usage: nomad <";
    for (std::size_t index = 0; index < commands.size(); ++index) {
        std::cout << (index == 0 ? "" : "|") << commands[index].name;
    }
    std::cout << "> [value] [--endpoint udpin:host:port]\n";
    for (const auto &entry : commands) {
        if (!entry.arguments.empty()) {
            std::cout << entry.name << " requires: " << entry.arguments << '\n';
        }
    }
    std::cout << "Actuation commands require the NOMAD_API_KEY environment variable.\n";
}
