#include "ExoTestState.h"

ExoTestState::ExoTestState(X2Robot *exo, DummyTrajectoryGenerator *tg, const char *name) : State(name), robot(exo), trajectoryGenerator(tg){};