//
// Created by Seth Parker on 6/9/16.
//
#include "core/types/Rendering.h"

using namespace volcart;

///// Constructors/Destructors /////
Rendering::Rendering()
{
    _metadata.set<std::string>("type", "rendering");
    _metadata.set<std::string>("id", DATE_TIME());
}
