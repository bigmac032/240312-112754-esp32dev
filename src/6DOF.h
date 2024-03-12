#ifndef _6DOF_H

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LSM6DS.h"
class 6DOF :  public Adafruit_LSM6DSOX {
  public:
    6DOF();

  void enableI2CMasterPullups(bool enable_pullups);
  void disableSPIMasterPullups(bool disable_pullups);

private:
  bool _init(int32_t sensor_id);
}
#endif