#ifndef PID_h
#define PID_h

class PID {
  public:
    PID(float kp, float ki, float kd);
    void setConstants(float kp, float ki, float kd);
    float compute(float target, float current);
    void reset();
    
  private:
    float _kp;
    float _ki;
    float _kd;
    float _integral;
    float _lastError;
};

#endif
