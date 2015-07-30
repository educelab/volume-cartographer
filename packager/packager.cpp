//
// Created by Seth Parker on 7/30/15.
//

#include "packager.h"

int main ( int argc, char* argv[]) {

    ///// Parse the command line options /////
    boost::filesystem::path slicesPath, volpkgPath;
    try {
        // All command line options
        boost::program_options::options_description options("Options");
        options.add_options()
                ("help,h", "Show this message")
                ("slices,s", boost::program_options::value<std::string>(), "Directory of input slice data")
                ("volpkg,vp", boost::program_options::value<std::string>(), "Path for the output volume package");

        // parsedOptions will hold the values of all parsed options as a Map
        boost::program_options::variables_map parsedOptions;
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(options).run(), parsedOptions);
        boost::program_options::notify(parsedOptions);

        // Show the help message
        if (parsedOptions.count("help") || argc < 2) {
            std::cout << options << std::endl;
            return EXIT_SUCCESS;
        }

        // Get the input slices path
        if (parsedOptions.count("slices")) {
            slicesPath = parsedOptions["slices"].as<std::string>();
        } else {
            std::cerr << "ERROR: Path to slices directory not supplied!" << std::endl;
            return EXIT_FAILURE;
        }

        // Get the output volpkg path
        if (parsedOptions.count("volpkg")) {
            volpkgPath = parsedOptions["volpkg"].as<std::string>();
        } else {
            std::cerr << "ERROR: Output Volume Package path not supplied!" << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch(std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }


    ///// Validate input options /////
    // Don't overwrite an existing volpkg
    if (boost::filesystem::exists(volpkgPath)) {
        std::cerr << "ERROR: Volume package already exists at path specified." << std::endl;
        std::cerr << "This program does not currently allow for modification of existing volume packages." << std::endl;
        return EXIT_FAILURE;
    }
    VolumePkg volpkg(volpkgPath.string(), VOLPKG_VERSION);

    // Filter the slice path directory
    std::vector<Slice> slices;
    if (boost::filesystem::exists(slicesPath) && boost::filesystem::is_directory(slicesPath)) {

        // Directory iterators
        boost::filesystem::directory_iterator dir_subfile(slicesPath);
        boost::filesystem::directory_iterator dir_end;

        // Filter out subfiles that aren't TIFs
        // To-Do: Handle other formats
        while (dir_subfile != dir_end) {
            std::string file_ext( boost::to_upper_copy<std::string>(dir_subfile->path().extension().string()) );
            if ( is_regular_file(dir_subfile->path()) && (file_ext == ".TIF" || file_ext == ".TIFF")) {
                Slice temp;
                temp.path = *dir_subfile;
                slices.push_back(temp);
            }
            ++dir_subfile;
        }
    } else {
        std::cerr << "ERROR: Slices directory does not exist/is not a directory." << std::endl;
        std::cerr << "Please provide a directory of slice images." << std::endl;
        return EXIT_FAILURE;
    }
    if (slices.empty()) {
        std::cerr << "ERROR: No TIF files found in slices directory." << std::endl;
        return EXIT_FAILURE;
    }
    // Sort the Slices by their filenames
    std::sort(slices.begin(), slices.end(), SliceLess);

    // Print the ordered paths
    for ( auto slice = slices.begin(); slice != slices.end(); ++slice ) {
        std::cout << slice->path.string() << std::endl;
    }

    return EXIT_SUCCESS;
}