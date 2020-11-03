#include "ExoTestState.h"

ExoTestState::ExoTestState(StateMachine *m, X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name) : State(m, name), robot(exo), trajectoryGenerator(tg){};