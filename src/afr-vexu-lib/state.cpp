#include <afr-vexu-lib/state.h>
#include <iostream>

AFR::VexU::state::state(const std::unordered_map<std::string, AFR::VexU::action&>& action_map,
                        const std::vector<AFR::VexU::transition>& transitions,
                        const std::function<void(const state&)>& on_state_entry) : action_map_(action_map),
                                                                                   transitions_(transitions),
                                                                                   on_state_entry_(on_state_entry){}

void AFR::VexU::state::update_actions(){
    for(auto action : action_map_){
        action.second.update();
    }
}

void AFR::VexU::state::on_state_entry(const state& previous){
    on_state_entry_(previous);
}

AFR::VexU::action& AFR::VexU::state::get_action(const std::string& identifier){
    return action_map_.at(identifier);
}

const std::vector<AFR::VexU::transition>& AFR::VexU::state::get_transitions(){
    return transitions_;
}

AFR::VexU::transition::transition(const std::function<bool()>& condition_function, const std::string& next_state)
        : condition_function_(condition_function), next_state_(next_state){}

bool AFR::VexU::transition::should_change_state() const{
    return condition_function_();
}

std::string AFR::VexU::transition::get_next_state() const{
    std::cout << "Moving to " << next_state_ << std::endl;
    return next_state_;
}