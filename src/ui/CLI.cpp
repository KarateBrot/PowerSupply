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


void CLI::_parse(const std::string &str) {

  StrList list = _split(str, ' ');

  if (list.empty()) {
    _pr_ptr->print(_prompt.c_str()); 
    _buffer.clear();
    return;
  }

  _buffer.cmd = list[0];

  if (_buffer.cmd == "?") { 
    help();
    return; 
  }

  for (size_t i = 1; i < list.size(); i++) {
    _buffer.args.emplace_back(list[i]); 
  }

  for (Command c : _commands) { 
    if (_execute(c)) return; 
  }

  _pr_ptr->println("Unknown command. '?' for available commands.");
  _pr_ptr->print(_prompt.c_str()); 
  _buffer.clear();
}


bool CLI::_execute(const Command &c) {

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

    return 1;
  }

  return 0;
}


void CLI::fetch(const char &c) {

  if (c == 0xD) {
    _pr_ptr->println();
    _parse(_buffer.line);
    return;
  }

  if ((c == 0x8 || c == 0x7F)) {
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

  uint8_t strMaxLength = 0;
  for (Command c : _commands) {
    uint8_t strLength = c.name.size();
    if (strLength > strMaxLength) { strMaxLength = strLength; }
  }

  for (Command c : _commands) {

    _pr_ptr->print(" ");
    _pr_ptr->print(c.name.c_str());
    for (size_t i = 0; i < (strMaxLength - c.name.size()); i++) {
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