/*
===========================================================================
fpvq3 - FPV quadcopter flight for ioquake3

This file is part of a Quake III Arena modification and is distributed
under the GNU General Public License version 2 or later, like the rest of
the ioquake3 game source it builds on.
===========================================================================
*/
//
// ai_drone.h -- bot autopilot for drones
//

void BotDroneSetupMovement( bot_state_t *bs, bot_initmove_t *initmove );
void BotMoveToGoalDrone( bot_state_t *bs, bot_moveresult_t *result, bot_goal_t *goal, int travelflags );
void BotDroneInput( bot_state_t *bs, bot_input_t *bi, usercmd_t *ucmd );
