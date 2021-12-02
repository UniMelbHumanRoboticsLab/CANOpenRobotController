#include "StateMachine.h"


StateMachine::StateMachine(): _running(false) {
}

void StateMachine::setRobot(std::shared_ptr<Robot> r) {
    spdlog::debug("StateMachine::setRobot()");
    if(!_robot) {
        _robot = r;
    }
    else {
        spdlog::error("Robot already set to state machine. Can't re-set.");
    }
}

bool StateMachine::configureMasterPDOs() {
    spdlog::debug("StateMachine::configureMasterPDOs()");
    if(_robot) {
        return _robot->configureMasterPDOs();
    }
    else {
        return false;
    }
}

void StateMachine::setInitState(std::string state_name) {
    _currentState = state_name;
    if(_states.count(_currentState)<1) {
        spdlog::error("Requested initial state {} does not exists. Left to default.", _currentState);
    }
}

void StateMachine::addState(std::string state_name, std::shared_ptr<State> s_ptr) {
   spdlog::debug("StateMachine::addState({})", state_name);
   _states[state_name]=s_ptr;
   //Set first added state as default first for execution
   if(_states.size()==1) {
      _currentState=state_name;
   }
}

void StateMachine::addTransition(std::string from, TransitionCb_t t_cb, std::string to) {
    spdlog::debug("StateMachine::addTransition({} -> {})", from, to);
    if(_states.count(from)>0 && _states.count(to)>0) {
        _transitions[from].push_back(Transition_t(t_cb, to));
    }
    else {
        spdlog::error("State {} or {} do not exist. Cannot create requested transition.", from, to);
    }
}

void StateMachine::addTransitionFromAny(TransitionCb_t t_cb, std::string to) {
    spdlog::debug("StateMachine::addTransition(ANY -> {})", to);
    if(_states.count(to)>0) {
        //Add transitions to all states (but target)
        for(const auto& [key, s]: _states) {
            if(key!=to) {
                _transitions[key].push_back(Transition_t(t_cb, to));
            }
        }
    }
    else {
        spdlog::error("State {} do not exist. Cannot create requested transitions.", to);
    }
}

void StateMachine::activate() {
    spdlog::debug("StateMachine::activate()");
    if(_states.count(_currentState)>0) {
        _running = true;
        _time_init = std::chrono::steady_clock::now();
        _time_running = 0;
        _states[_currentState]->doEntry();
    }
    else {
        spdlog::critical("StateMachine activation state ({}) does not exist. Exiting...", _currentState);
        std::raise(SIGTERM); //Clean exit
    }
}

void StateMachine::update() {
    spdlog::trace("StateMachine::update()");

    //Keep running time
    _time_running = (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - _time_init).count()) / 1e6;

    //Call state machine hardware update method (specialised)
    hwStateUpdate();

    //Manage possible transition
    bool transitioned = false;
    for (auto& tr : _transitions[_currentState]) {
        //Transition is active?
        if(tr.first(*this)) {
            _states[_currentState]->doExit();
            _currentState=tr.second;
            _states[_currentState]->doEntry();
            transitioned=true;
            break;
        }
    }

    //Execute (if not just transitioned)
    if(!transitioned) {
        _states[_currentState]->doDuring();
    }

    //Logging
    if(logHelper.isStarted() && logHelper.isInitialised())
        logHelper.recordLogData();
}

void StateMachine::hwStateUpdate() {
    spdlog::trace("StateMachine::hwStateUpdate()");
    if(_robot) {
        _robot->updateRobot();
    }
}
