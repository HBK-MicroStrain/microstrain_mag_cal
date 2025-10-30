#include <iostream>

#include "mio/mmap.hpp"


int main()
{
    std::error_code error;
    const mio::mmap_source map_source = mio::make_mmap_source("test_file_for_offline_mag.bin", error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
        return 1;
    }

    const char *data = map_source.data();

    return 0;
}