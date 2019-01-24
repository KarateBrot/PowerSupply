#include "Scheduler.h"



Task::Task(fptr_t f) :

  lastCallback(Scheduler::_timeFn()),
  birthtime(lastCallback) {

  callback = f;
}



std::function<uint32_t(void)> Scheduler::_timeFn;
std::function<void(void)>     Scheduler::_yield;


Scheduler::Scheduler(std::function<uint32_t(void)> timeFn, std::function<void(void)> yield) {

  _timeFn = timeFn;
  _yield  = yield;

  uint32_t t = _timeFn();

  _lastTick      = t;
  _lastWait      = t;
  _lastWaitUntil = t;
}


Scheduler& Scheduler::add(fptr_t f) {

  if (_tasks.size() <= UINT8_MAX) { _tasks.emplace_back(f); }

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
  uint32_t timer = _timeFn();
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
      timer      = _timeFn();  // Snapshot of current point in time
      
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
    timer     = _timeFn();
    _tickrate = 1000000L/(uint32_t)(timer - _lastTick);
    _lastTick = timer;

    _yield();
  }
}


void Scheduler::wait(uint32_t val) {

  _lastWait = _timeFn();
  while ((uint32_t)(_timeFn() - _lastWait) < val) { _yield(); }
}


void Scheduler::waitUntil(uint32_t val) {

  while ((uint32_t)(_timeFn() - _lastWaitUntil) < val) { _yield(); }
  _lastWaitUntil = _timeFn();
}


void Scheduler::forceTickRate(float tps) {

  waitUntil( (uint32_t)(1000000.0f/tps + 0.5f) );
}
