#include <iostream>
#include <string>

#include "vc/core/util/Signals.hpp"

inline void NoParameter() { std::cout << "Hello, World!" << std::endl; }

inline void SingleParameter(int i) { std::cout << i << std::endl; }

inline void FloatParameter(float f) { std::cout << f << std::endl; }

inline void MultiParameter(int i, float f, const std::string& s)
{
    std::cout << i << " " << f << " " << s << std::endl;
}

using namespace volcart;

auto main() -> int
{
    // No parameter
    Signal<> event;
    event.connect(NoParameter);
    event.send();

    // Single parameter
    Signal<int> oneParam;
    oneParam.connect(SingleParameter);
    oneParam.send(1);

    // Implicit conversion
    // Compiler will still warn about this
    Signal<float> floatParam;
    floatParam.connect(FloatParameter);
    floatParam.connect(SingleParameter);
    floatParam.send(1.5);

    // Multiple parameters
    Signal<int, float, std::string> multiParam;
    multiParam.connect(MultiParameter);
    multiParam.send(1, 2.0, "3");

    // Clear all connections
    oneParam.disconnect();
    floatParam.disconnect();
    multiParam.disconnect();

    // Any signal can connect to a no parameter function
    oneParam.connect(NoParameter);
    floatParam.connect(NoParameter);
    multiParam.connect(NoParameter);

    oneParam.send(0);
    floatParam.send(0);
    multiParam.send(0, 0, "");
}