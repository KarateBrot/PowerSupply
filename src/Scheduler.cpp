#include "Scheduler.h"




Task::Task(String taskName, fptr_t f, float tps) {

  name     = taskName;
  callback = f;

  tps != 0
    ? deltaTime = (uint32_t)(1000000.0f/tps)
    : deltaTime = 0.0f;
  lastCallback  = micros();
}


Scheduler::Scheduler() {

  uint32_t t = micros();

  _lastTick      = t;
  _lastWait      = t;
  _lastWaitUntil = t;
}


void Scheduler::add(String taskName, fptr_t &f, float tps) {

  if (_tasks.size() <= UINT8_MAX) { 
    
    _tasks.push_back( Task(taskName, f, tps) ); 
  }
}


void Scheduler::add(fptr_t f, float tps) {

  add("", f, tps);
}


void Scheduler::remove(String taskName) {

  std::vector<Task>::iterator front = _tasks.begin();
  uint8_t                     size  = _tasks.size();

  for (size_t n = 0; n < size; n++) {

    if (_tasks[n].name == taskName) { _tasks.erase(front + n); }
  }
}


void Scheduler::run() {

  // Initialization
  _running       = true;
  uint8_t  size  = _tasks.size();
  uint32_t timer = micros();
  for (size_t n  = 0; n < size; n++) { _tasks[n].lastCallback = timer; }
  _lastTick      = timer;
  _lastWait      = timer;
  _lastWaitUntil = timer;

  // Infinite loop until Scheduler::stop() is called
  while (_running) {

    size = _tasks.size();

    for (size_t n = 0; n < size; n++) {

      Task &task = _tasks[n];
      timer      = micros();

      if ((uint32_t)(timer - task.lastCallback) >= task.deltaTime) {
        task.callback();
        task.lastCallback += task.deltaTime;
      }
    }

    timer     = micros();
    _tickrate = 1000000.0f/(float)(uint32_t)(timer - _lastTick);
    _lastTick = timer;

    yield();
  }
}


void Scheduler::wait(uint32_t timer) {

  _lastWait = micros();
  while ((uint32_t)(micros() - _lastWait) < timer) { yield(); }
}


void Scheduler::waitUntil(uint32_t timer) {

  while ((uint32_t)(micros() - _lastWaitUntil) < timer) { yield(); }
  _lastWaitUntil = micros();
}


void Scheduler::forceTickRate(float tps) {

  waitUntil( (uint32_t)(1000000.0f/tps + 0.5f) );
}
