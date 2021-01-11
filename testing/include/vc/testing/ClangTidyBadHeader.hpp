#pragma once

/** @file */

#include <iostream>

#define BUFLEN 42

template <typename T>
void func(T t)
{
    char buf[BUFLEN];
    std::cout << sizeof(BUFLEN) << std::endl;
    std::cout << buf << std::endl;
    auto otherT = T{};
    otherT += T{};
    otherT += t;
    return;
}
