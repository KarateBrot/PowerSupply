#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "Arduino.h"
#include <algorithm>
#include <vector>

// #pragma GCC optimize ("O3")

typedef void(*fptr_t)(void);


struct Task {

  fptr_t callback;
  bool   toBeDeleted;
  
  uint32_t 
    lastCallback, 
    deltaTime, 
    offset, 
    numCalls, numCallsMax, 
    birthtime, lifetime;

  Task(fptr_t);
};


class Scheduler {

private:
  std::vector<Task> _tasks;
  
  bool     _running;
  uint32_t _tickrate, _lastTick, _lastWait, _lastWaitUntil;

  static bool _isMarkedToDelete(Task& task) { return task.toBeDeleted; }
  
public:
  Scheduler(void);

  Scheduler& add      (fptr_t);
  Scheduler& period   (uint32_t);
  Scheduler& frequency(float);
  Scheduler& numCalls (uint32_t);
  Scheduler& lifetime (uint32_t);
  Scheduler& offset   (uint32_t);

  void run  (void);
  void stop (void) { _running = false; }
  void clear(void) { _tasks.clear(); }

  void wait         (uint32_t);
  void waitUntil    (uint32_t);
  void forceTickRate(float);

  uint32_t getTickRate(void) const { return _tickrate; }
};


#endif // SCHEDULER_H
