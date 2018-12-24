#ifndef GRAPHICAL_USER_INTERFACE_H
#define GRAPHICAL_USER_INTERFACE_H

#include <cstdint>
#include <memory>
#include <vector>


class Page {

};


class ListPage : public Page {

};


class StatsPage : public Page {

};


class GUI {

private:
  std::vector<std::unique_ptr<Page>> _pageBuffer;
};


#endif // GRAPHICAL_USER_INTERFACE_H