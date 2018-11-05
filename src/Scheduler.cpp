#include "Scheduler.h"




Task::Task(fptr_t f) {

  lastCallback = micros();
  
  callback = f;
  
  toBeDeleted = false;
  deltaTime   = 0;
  offset      = 0;
  numCalls    = 0;
  numCallsMax = 0;
  birthtime   = lastCallback;
  lifetime    = 0;
}


Scheduler::Scheduler() {

  uint32_t t = micros();

  _lastTick      = t;
  _lastWait      = t;
  _lastWaitUntil = t;
}


Scheduler& Scheduler::add(fptr_t f) {

  if (_tasks.size() <= UINT8_MAX) { _tasks.push_back(Task(f)); }

  return *this;
}


Scheduler& Scheduler::period(uint32_t val) {

  _tasks.back().deltaTime = val;

  return *this;
}


Scheduler& Scheduler::frequency(float cps) {

  cps != 0
    ? _tasks.back().deltaTime = (uint32_t)(1000000.0f/cps)
    : _tasks.back().deltaTime = 0.0f;

  return *this;
}


Scheduler& Scheduler::numCalls(uint32_t num) {

  _tasks.back().numCallsMax = num;

  return *this;
}


Scheduler& Scheduler::lifetime(uint32_t val) {

  _tasks.back().lifetime = val;

  return *this;
}


Scheduler& Scheduler::offset(uint32_t val) {

  _tasks.back().offset = val;

  return *this;
}


void Scheduler::run() {

  // Initialization
  uint32_t timer = micros();
  uint8_t  size  = _tasks.size();
  
  _lastTick      = timer;
  _lastWait      = timer;
  _lastWaitUntil = timer;
  _running       = true;

  // Infinite loop until Scheduler::stop() is called
  while (_running) {

    // Iterate through every task in _tasks
    size = _tasks.size();
    for (size_t n = 0; n < size; n++) {

      Task& task = _tasks[n]; // Reference for better readability
      timer      = micros();  // Snapshot of current point in time
      
      // Check if current task is determined to be executed
      uint32_t delta = (uint32_t)(timer - task.lastCallback);
      if (delta >= task.deltaTime && delta >= task.offset) {
        
        // Callback task and increment counter
        task.callback();
        task.numCalls++;
        
        // Determine point in time of next task callback
        if (task.offset == 0) { 
          task.lastCallback += task.deltaTime;
        } 
        else {
          task.offset       = 0;
          task.birthtime    = timer; 
          task.lastCallback = timer;
        }

        // References for better readability
        uint32_t& calls = task.numCalls;
        uint32_t& max   = task.numCallsMax;
        uint32_t& life  = task.lifetime;

        // Check for deletion of task
        delta = (uint32_t)(timer - task.birthtime);
        if ( (calls >= max && max != 0) || (delta >= life && life != 0) ) {
          task.toBeDeleted = true;
        }
      }
    }

    // Remove all tasks marked for deletion
    _tasks.erase( 
      std::remove_if(_tasks.begin(), _tasks.end(), _isMarkedToDelete), 
      _tasks.end()
    );

    // Calculate tickrate of scheduler
    timer     = micros();
    _tickrate = 1000000L/(uint32_t)(timer - _lastTick);
    _lastTick = timer;

    yield();
  }
}


void Scheduler::wait(uint32_t val) {

  _lastWait = micros();
  while ((uint32_t)(micros() - _lastWait) < val) { yield(); }
}


void Scheduler::waitUntil(uint32_t val) {

  while ((uint32_t)(micros() - _lastWaitUntil) < val) { yield(); }
  _lastWaitUntil = micros();
}


void Scheduler::forceTickRate(float tps) {

  waitUntil( (uint32_t)(1000000.0f/tps + 0.5f) );
}
