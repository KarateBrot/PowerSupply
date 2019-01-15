#ifndef COMMAND_LINE_INTERFACE_H
#define COMMAND_LINE_INTERFACE_H



#include "Arduino.h"
#include <cstdlib>
#include <string>
#include <vector>

typedef void(*fptr_t)(void);

struct Command;
typedef std::vector<Command> CommandList;
typedef std::vector<int32_t> ArgList;


struct Command {

  std::string name, help;
  uint8_t     nArgs;
  fptr_t      run;
};



class CLI {

private:
  std::string _prompt;
  CommandList _commands;
  Print      *_print_ptr;

  struct Buffer {
    std::string line, command;
    ArgList     args;

    void clear(void);
  }
  _buffer;

  void _parse  (std::string);
  bool _execute(Command);

public:
  CLI(const std::string&, Print*);
  CLI(const std::string&, Print*, CommandList);
  
  void fetch(char);
  void help (void);

  void operator<<(char c) { fetch(c); }
  
  CLI& add(const Command &c) { _commands.emplace_back(c); return *this; };

  ArgList getArgs(void)    const { return _buffer.args; }
  int32_t getArg (uint8_t) const;
};



#endif // COMMAND_LINE_INTERFACE_H