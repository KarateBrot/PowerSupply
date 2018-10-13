#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>
#include <vector>

typedef void(*fptr_t)(void);


struct Task {

public:
  String   name;
  fptr_t   callback;
  uint32_t lastCallback, deltaTime;

  Task(String, fptr_t, float);
};


class Scheduler {

private:

  std::vector<Task> _tasks;

  bool     _running;
  float    _tickrate;
  uint32_t _lastTick, _lastWait, _lastWaitUntil;

public:

  Scheduler(void);

  void add   (String, fptr_t &, float);
  void add   (fptr_t, float);
  void remove(String);

  void run   (void);
  void stop  (void) { _running = false; }
  void clear (void) { _tasks.clear(); }

  void wait         (uint32_t);
  void waitUntil    (uint32_t);
  void forceTickRate(float);

  uint32_t getTickRate(void) { return _tickrate; }
};


#endif // SCHEDULER_H
