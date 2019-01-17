#include "CLI.h"


CLI::CLI(const std::string &prompt, Print *pf) : 
  
  _prompt(prompt),
  _pr_ptr(pf) {

  _pr_ptr->print(_prompt.c_str());
}


CLI::CLI(const std::string &prompt, Print *pf, CmdList l) :

  CLI(prompt, pf) {
  
  _commands = l;
}


StrList CLI::_split(std::string str, const char &delimiter) {

  StrList list;
  size_t  cursor;

  while (!str.empty()) {

    while (str[0] == delimiter) str.erase(0, 1);
    if (str.empty()) break;
    
    cursor = str.find(delimiter);
    list.emplace_back(str.substr(0, cursor));
    str.erase(0, cursor);
    
    yield();
  }

  return list;
}


void CLI::_parse() {

  StrList list = _split(_buffer.line, ' ');

  if (list.empty()) return;

  _buffer.cmd = list[0];
  
  for (size_t i = 1; i < list.size(); i++) {
    _buffer.args.emplace_back(list[i]); 
  }
}


void CLI::_execute() {

  if (_buffer.cmd.empty()) {
    _pr_ptr->print(_prompt.c_str()); 
    _buffer.clear();
    return;
  }

  if (_buffer.cmd == "?") {
    help(); 
    return; 
  }

  for (Command c : _commands) { 

    if (_buffer.cmd == c.name) {
        
      if (_buffer.args.size() >= c.nArgs) { c.run(); } else {

        _pr_ptr->print("Command '");
        _pr_ptr->print(c.name.c_str());
        _pr_ptr->print("' requires ");
        _pr_ptr->print(c.nArgs);
        _pr_ptr->println(c.nArgs == 1 ? " argument." : " arguments.");
      }

      _pr_ptr->print(_prompt.c_str()); 
      _buffer.clear();

      return;
    }
  }

  _pr_ptr->println("Unknown command. '?' for available commands.");
  _pr_ptr->print(_prompt.c_str()); 
  _buffer.clear();
}


void CLI::fetch(const char &c) {

  if (c == CHAR_CR) {

    _pr_ptr->println();

    _parse();
    _execute();

    return;
  }

  if ((c == CHAR_BS || c == CHAR_DEL)) {

    if (!_buffer.line.empty()) {

      _buffer.line.pop_back();
      _pr_ptr->print('\10');
    }

    return;
  }

  _buffer.line += c;
  _pr_ptr->print(c);
}


void CLI::help() {

  _pr_ptr->println("Available commands:");

  uint8_t maxLength = 0;
  for (Command c : _commands) {
    uint8_t length = c.name.size();
    if (length > maxLength) { maxLength = length; }
  }

  for (Command c : _commands) {

    _pr_ptr->print(" ");
    _pr_ptr->print(c.name.c_str());
    for (size_t i = 0; i < (maxLength - c.name.size()); i++) {
      _pr_ptr->print(" ");
    }
    _pr_ptr->print(" - ");
    _pr_ptr->println(c.help.c_str());
  }

  _pr_ptr->print(_prompt.c_str());
  _buffer.clear();
}


std::string CLI::getArg(uint8_t n) const {

  uint8_t nArgs = _buffer.args.size();

  if (n < 1)     { n = 1;     }
  if (n > nArgs) { n = nArgs; }

  return _buffer.args[n-1];
}


int32_t CLI::getArg_i(const uint8_t &n) const {

  return atoi(getArg(n).c_str());
}


double CLI::getArg_d(const uint8_t &n) const {

  return atof(getArg(n).c_str());
}