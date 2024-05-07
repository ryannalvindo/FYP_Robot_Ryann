#ifndef PID_ROUTINE_H
#define PID_ROUTINE_H

void goToTargetPos(Encoder &encoder_left, long target_pos_left, Encoder &encoder_right, long target_pos_right, uint8_t motorLeft, uint8_t speedLeft, uint8_t motorRight, uint8_t speedRight);

#endif