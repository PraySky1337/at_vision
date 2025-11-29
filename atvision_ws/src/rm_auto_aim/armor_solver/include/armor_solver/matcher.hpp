#pragma once
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/target.hpp>

namespace armor_tracker {
struct Matcher {
    Matcher() = default;
    void filter(rm_interfaces::msg::Armors& armors, const rm_interfaces::msg::Target& target);
private:

};
} // namespace armor_tracker