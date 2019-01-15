#include "CLI.h"


CLI::CLI(const std::string &prompt, Print *pf) : 
  
  _prompt(prompt),
  _print_ptr(pf) {

  _print_ptr->print(_prompt.c_str());
}

CLI::CLI(const std::string &prompt, Print *pf, CommandList l) :

  CLI(prompt, pf) {
  
  _commands = l;
}


void CLI::Buffer::clear() {

  line.clear();
  command.clear();
  args.clear();
}


void CLI::_parse(std::string str) {

  if (str.empty()) {
    _buffer.clear();
    _print_ptr->print(_prompt.c_str());
    return; 
  }
  
  if (str == "?") { 
    help();
    return; 
  }

  size_t cursor = str.find(" ");

  _buffer.command = str.substr(0, cursor);
  str.erase(0, cursor);

  while (!str.empty()) {

    while (str[0] == ' ') str.erase(0, 1);
    if (str.empty()) break;
    cursor = str.find(" ");
    int32_t arg = atoi(str.c_str());
    _buffer.args.emplace_back(arg);
    str.erase(0, cursor);
    yield();
  }

  for (Command c : _commands) { 
    if (_execute(c)) { return; }
  }

  _buffer.clear();
  _print_ptr->println("Unknown command. '?' for help.");
  _print_ptr->print(_prompt.c_str()); 
}


bool CLI::_execute(Command c) {

  if (_buffer.command == c.name) {
      
    if (_buffer.args.size() >= c.nArgs) { c.run(); } else {

      _print_ptr->print("Command '");
      _print_ptr->print(c.name.c_str());
      _print_ptr->print("' requires ");
      _print_ptr->print(c.nArgs);
      _print_ptr->println(c.nArgs == 1 ? " argument." : " arguments.");
    }

    _buffer.clear();
    _print_ptr->print(_prompt.c_str()); 

    return 1;
  }

  return 0;
}


void CLI::fetch(char c) {

  if (c == 0xD) { // Carriage return
    _print_ptr->println();
    _parse(_buffer.line);
    return;
  }

  if ((c == 0x8 || c == 0x7F)) {
    if (!_buffer.line.empty()) {
      _buffer.line.pop_back();
      _print_ptr->print("\10");
    }
    return;
  }

  _buffer.line += c;
  _print_ptr->print(c);
}


void CLI::help() {

  _print_ptr->println("Available commands:");

  for (Command c : _commands) {

    _print_ptr->print(" ");
    _print_ptr->print(c.name.c_str());
    _print_ptr->print(" - ");
    _print_ptr->println(c.help.c_str());
  }

  _buffer.clear();
  _print_ptr->print(_prompt.c_str());
}


int32_t CLI::getArg(uint8_t n) const {

  uint8_t nArgs = _buffer.args.size();

  if (n < 1)     { n = 1;     }
  if (n > nArgs) { n = nArgs; }

  return _buffer.args[n-1];
}