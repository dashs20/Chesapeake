#pragma once
#include <functional>
#include <vector>
#include <unordered_map>

template <typename StateEnum, typename Context>
struct ExitPath {
    StateEnum destination;
    std::function<bool(const Context&)> condition;
};

template <typename StateEnum, typename Context>
struct State {
    StateEnum id;
    std::vector<ExitPath<StateEnum, Context>> exit_paths;
};

template <typename StateEnum, typename Context>
class StateMachine {
public:
    StateMachine() = default;

    void add_state(State<StateEnum, Context> state) {
        states[state.id] = state;
    }

    StateEnum update(StateEnum current_state_id, const Context& context) {
        auto it = states.find(current_state_id);
        if (it != states.end()) {
            for (const auto& path : it->second.exit_paths) {
                if (path.condition(context)) {
                    return path.destination;
                }
            }
        }
        return current_state_id;
    }

private:
    std::unordered_map<StateEnum, State<StateEnum, Context>> states;
};
