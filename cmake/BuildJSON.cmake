option(VC_BUILD_JSON "Build in-source JSON library" on)
if(VC_BUILD_JSON)
    FetchContent_Declare(
        json
        URL https://github.com/nlohmann/json/archive/v3.11.3.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP ON
        EXCLUDE_FROM_ALL
    )
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    set(JSON_Install ON CACHE INTERNAL "")
    FetchContent_MakeAvailable(json)
else()
    find_package(nlohmann_json 3.9.1 REQUIRED)
endif()