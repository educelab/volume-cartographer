### Check platform support for memory mapping
### 08/2024 - Add support for systems providing mmap

# Check if minimal mmap code compiles
set(mmap_code [[
    #include <cerrno>
    #include <cstring>
    #include <fcntl.h>
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <unistd.h>

    int main() {
        const char* path = "foo.txt";
        const int fd = open(path, O_RDONLY);

        struct stat sb {};
        fstat(fd, &sb);

        void* data = mmap(nullptr, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);
        close(fd);

        munmap(data, sb.st_size);
        return 0;
    }
]])
check_cxx_source_compiles("${mmap_code}" VC_MEMMAP_MMAP)

# Check platform support
set(VC_MEMMAP_SUPPORTED FALSE)
if(VC_MEMMAP_MMAP)
    message(STATUS "Using MemMap support: mmap")
    set(VC_MEMMAP_SUPPORTED TRUE)
    add_compile_definitions(VC_MEMMAP_MMAP)
else()
    message(STATUS "MemMap support: none")
endif()

set(VC_MEMMAP_SUPPORTED VC_MEMMAP_SUPPORTED CACHE BOOL "Memory mapping is supported on this platform")
if(VC_MEMMAP_SUPPORTED)
    add_compile_definitions(VC_MEMMAP_SUPPORTED=true)
else()
    add_compile_definitions(VC_MEMMAP_SUPPORTED=false)
endif()