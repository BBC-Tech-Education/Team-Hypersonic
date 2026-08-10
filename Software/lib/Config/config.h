#ifndef CONFIG_H
#define CONFIG_H

/* COMPETITION MODE */
#define COMP false

/* ROBOT */
#define ROBOT_1 false
#define ROBOT_2 (!ROBOT_1)
#define ATTACK true

/* ATTACK DIRECTION */
#define BLUE_GOAL_ATTACK true
// True: Attacking Blue Goal (Defending Yellow Goal)
// False: Attacking Yellow Goal (Defending Blue Goal)

#endif