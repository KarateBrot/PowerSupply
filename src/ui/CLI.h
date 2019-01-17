#ifndef COMMAND_LINE_INTERFACE_H
#define COMMAND_LINE_INTERFACE_H



#include "Arduino.h"
#include <cstdlib>
#include <string>
#include <vector>

typedef void(*fptr_t)(void);

struct Command;
typedef std::vector<Command>     CmdList;
typedef std::vector<std::string> StrList;
typedef std::vector<int32_t>     ArgList;



struct Command {

  std::string name, help;
  uint8_t     nArgs;
  fptr_t      run;
};



class CLI {

private:
  std::string _prompt;
  CmdList     _commands;
  Print      *_pr_ptr;

  struct Buffer {
    std::string line, cmd;
    ArgList     args;

    void clear(void) { line.clear(); cmd.clear(); args.clear(); }
  }
  _buffer;

  StrList _split  (std::string, const char&);
  void    _parse  (const std::string&);
  bool    _execute(const Command&);

public:
  CLI(const std::string&, Print*);
  CLI(const std::string&, Print*, CmdList);
  
  void fetch(const char&);
  void help (void);
  
  CLI& add(const Command &c) { _commands.emplace_back(c); return *this; }

  void operator<<(const char &c) { fetch(c); }

  ArgList getArgs(void)    const { return _buffer.args; }
  int32_t getArg (uint8_t) const;
};



#endif // COMMAND_LINE_INTERFACE_H