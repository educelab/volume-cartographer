# Graph processing

[TOC]

Volume Cartographer is integrated with the 
[Smeagol](https://gitlab.com/educelab/smeagol) graph processing library. All 
nodes can be found in the `VC::graph` library.

## Initializing the graph processing system

Include `vc/graph.hpp` and call volcart::RegisterNodes() before doing anything:

```{.cpp}
#include <vc/graph.hpp>

int main()
{
    volcart::RegisterNodes();
}
```

## Building a graph

See 
[RenderGraphsExample.cpp](https://gitlab.com/educelab/volume-cartographer/-/tree/develop/examples/src/RenderGraphsExample.cpp)
for an example of how to build a render graph.