#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <algorithm>
#include <functional>
#include <vector>

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

friend class Task;

private:
  std::vector<Task> _tasks;

  static std::function<uint32_t(void)> _timeFn;
  static std::function<void(void)>     _yield;
  
  bool     _running;
  uint32_t _tickrate, _lastTick, _lastWait, _lastWaitUntil;

  static bool _isMarkedToDelete(Task& task) { return task.toBeDeleted; }

public:
  Scheduler(std::function<uint32_t(void)>, std::function<void(void)> = NULL);

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
