#include <iostream>

#include "vc/testing/ClangTidyBadHeader.hpp"

int main()
{
    std::cout << "hello, world!" << std::endl;
    bool a = true;
    if (a)
        std::cout << "a is true" << std::endl;
    func(int(1));
    return 0;
}
