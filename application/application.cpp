#include <iostream>

#include "mio/mmap.hpp"


/// Creates a read-only memory-mapped view of the given file.
mio::mmap_source map_file(const std::string &file_path, std::error_code &error)
{
    mio::mmap_source read_only_mapping = mio::make_mmap_source(file_path, error);

    if (error)
    {
        std::cerr << "Error opening file: " << error.message() << std::endl;
    }

    return read_only_mapping;
}


int main()
{
    std::error_code error;
    const mio::mmap_source file_mapping = map_file("test_file_for_offline_mag.bin", error);

    if (error)
    {
        return 1;
    }

    const char *data = file_mapping.data();
    const size_t data_size = file_mapping.size();

    return 0;
}