#ifndef PID_ROUTINE_H
#define PID_ROUTINE_H

void goToTargetPos (Encoder &encoder1, long targetPos1, Encoder &encoder2, long targetPos2, uint8_t motor1_pin_1, uint8_t motor1_pin_2, uint8_t enable_pin1, uint8_t motor2_pin_1, uint8_t motor2_pin_2, uint8_t enable_pin2);

#endif