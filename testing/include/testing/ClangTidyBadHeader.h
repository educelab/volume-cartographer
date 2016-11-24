#pragma once

#include <iostream>

#define BUFLEN 42

template <typename T>
void func(T t)
{
    char buf[BUFLEN];
    std::cout << sizeof(BUFLEN) << std::endl;
    T other_t = T{};
    other_t += T{};
    return;
}
