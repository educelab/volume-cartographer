#include <iostream>

#include "vc/core/Version.hpp"

using namespace volcart;

auto main() -> int
{
    std::cout << ProjectInfo::NameAndVersion();
    std::cout << " (" << ProjectInfo::RepositoryShortHash() << ")\n";
}