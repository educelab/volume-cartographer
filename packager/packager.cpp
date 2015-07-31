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
                ("volpkg,v", boost::program_options::value<std::string>(), "Path for the output volume package");

        // parsedOptions will hold the values of all parsed options as a Map
        boost::program_options::variables_map parsedOptions;
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(options).run(), parsedOptions);
        boost::program_options::notify(parsedOptions);

        // Show the help message
        if (parsedOptions.count("help") || argc < 2) {
            std::cout << options << std::endl;
            return EXIT_SUCCESS;
        }

        // Get the input slices dir
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

    ///// Setup /////
    // Check the extension and make sure the pkg doesn't already exist
    if ( volpkgPath.extension().string() != ".volpkg" ) volpkgPath.replace_extension(".volpkg");
    if (boost::filesystem::exists(volpkgPath)) {
        std::cerr << "ERROR: Volume package already exists at path specified." << std::endl;
        std::cerr << "This program does not currently allow for modification of existing volume packages." << std::endl;
        return EXIT_FAILURE;
    }

    // Generate an empty volpkg and save it to disk
    VolumePkg volpkg(volpkgPath.string(), VOLPKG_VERSION);
    volpkg.readOnly(false);
    volpkg.setMetadata("volumepkg name", volpkgPath.stem().string());
    volpkg.initialize();

    // Filter the slice path directory by extension and sort the vector of files
    std::vector<volcart::Slice> slices;
    if (boost::filesystem::exists(slicesPath) && boost::filesystem::is_directory(slicesPath)) {

        // Directory iterators
        boost::filesystem::directory_iterator dir_subfile(slicesPath);
        boost::filesystem::directory_iterator dir_end;

        // Filter out subfiles that aren't TIFs
        // To-Do: Handle other formats
        while (dir_subfile != dir_end) {
            std::string file_ext( boost::to_upper_copy<std::string>(dir_subfile->path().extension().string()) );
            if (is_regular_file(dir_subfile->path()) && (file_ext == ".TIF" || file_ext == ".TIFF")) {
                volcart::Slice temp;
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
        std::cerr << "ERROR: No supported image files found in provided slices directory." << std::endl;
        return EXIT_FAILURE;
    }
    // Sort the Slices by their filenames
    std::sort(slices.begin(), slices.end(), SliceLess);


    ///// Analyze the slices /////
    bool vol_consistent = true;
    double vol_min, vol_max;
    for ( auto slice = slices.begin(); slice != slices.end(); ++slice ) {
        if (!slice->analyze()) continue; // skip if we can't analyze

        // Compare all slices to the properties of the first slice
        if (slice == slices.begin()) {
            vol_min = slice->min();
            vol_max = slice->max();
        }
        else {
            // Check for consistency first
            if (*slice != *slices.begin()) {
                vol_consistent = false;
                std::cerr << slice->path.filename() << " does not match the initial slice of the volume." << std::endl;
                continue;
            }

            // Update the volume's min and max
            if ( slice->min() < vol_min ) vol_min = slice->min();
            if ( slice->max() > vol_max ) vol_max = slice->max();
        }

    }
    if (!vol_consistent) {
        std::cerr << "ERROR: Slices in slice directory do not have matching properties (width/height/depth)." << std::endl;
        return EXIT_FAILURE;
    }

    ///// Add data to the volume package /////
    // Metadata
    volpkg.setMetadata("number of slices", slices.size());
    volpkg.setMetadata("width", slices.begin()->width());
    volpkg.setMetadata("height", slices.begin()->height());

    // Scale 8-bit min/max values
    // To-Do: Handle other bit depths
    if ( slices.begin()->depth() == 0 ) {
        vol_min = vol_min * 65535.00/255.00;
        vol_max = vol_max * 65535.00/255.00;
    }
    volpkg.setMetadata("min", vol_min);
    volpkg.setMetadata("max", vol_max);

    std::cout << "Saving slice images to " << volpkg.getPkgName() << "..." << std::endl;
    unsigned long count = 0;
    for ( auto slice = slices.begin(); slice != slices.end(); ++slice ) {
        volpkg.setSliceData(count, slice->image());
        ++count;
    }

    // Save final metadata changes to disk
    volpkg.saveMetadata();
    return EXIT_SUCCESS;
}