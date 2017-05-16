#include "vc/core/types/Rendering.hpp"

using namespace volcart;

///// Constructors/Destructors /////
Rendering::Rendering()
{
    metadata_.set<std::string>("type", "rendering");
    metadata_.set<std::string>("id", DateTime());
}
