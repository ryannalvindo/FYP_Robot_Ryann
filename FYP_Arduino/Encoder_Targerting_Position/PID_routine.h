#ifndef PID_ROUTINE_H
#define PID_ROUTINE_H

void goToTargetPos (Encoder &encoder1, long targetPos, uint8_t motor_pin_1, uint8_t motor_pin_2, uint8_t enable_pin);

#endif