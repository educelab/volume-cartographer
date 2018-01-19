#include "vc/core/types/VolumePkg.hpp"

#include <boost/range/iterator_range.hpp>

#include "vc/core/util/DateTime.hpp"

using namespace volcart;

namespace fs = boost::filesystem;

static const fs::path SUBPATH_META{"config.json"};
static const fs::path SUBPATH_REND{"renders"};
static const fs::path SUBPATH_SEGS{"paths"};
static const fs::path SUBPATH_VOLS{"volumes"};

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(const fs::path& fileLocation, int version)
    : rootDir_{fileLocation}
    , volsDir_{fileLocation / SUBPATH_VOLS}
    , segsDir_{fileLocation / SUBPATH_SEGS}
    , rendDir_{fileLocation / SUBPATH_REND}
{
    // Lookup the metadata template from our library of versions
    auto findDict = volcart::VERSION_LIBRARY.find(version);
    if (findDict == std::end(volcart::VERSION_LIBRARY)) {
        throw std::runtime_error("No dictionary found for volpkg");
    }

    // Create the directories with the default values
    config_ = VolumePkg::InitConfig(findDict->second, version);
    config_.setPath(rootDir_ / "config.json");

    // Make directories
    for (const auto& d : {rootDir_, volsDir_, segsDir_, rendDir_}) {
        if (!fs::exists(d)) {
            fs::create_directory(d);
        }
    }

    // Do initial save
    config_.save();
}

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(const fs::path& fileLocation)
    : rootDir_{fileLocation}
    , volsDir_{fileLocation / SUBPATH_VOLS}
    , segsDir_{fileLocation / SUBPATH_SEGS}
    , rendDir_{fileLocation / SUBPATH_REND}
{
    // Check directory structure
    if (!(fs::exists(rootDir_) && fs::exists(segsDir_) &&
          fs::exists(volsDir_) && fs::exists(rendDir_))) {
        throw std::runtime_error("invalid volumepkg structure");
    }

    // Loads the metadata
    config_ = volcart::Metadata(fileLocation / SUBPATH_META);

    // Load volumes into volumes_ vector
    auto range =
        boost::make_iterator_range(fs::directory_iterator(volsDir_), {});
    for (const auto& entry : range) {
        if (fs::is_directory(entry)) {
            auto v = Volume::New(entry);
            volumes_.emplace(v->id(), v);
        }
    }

    // Load segmentations into the segmentations_ vector
    range = boost::make_iterator_range(fs::directory_iterator(segsDir_), {});
    for (const auto& entry : range) {
        if (fs::is_directory(entry)) {
            auto s = Segmentation::New(entry);
            segmentations_.emplace(s->id(), s);
        }
    }

    // Load Renders into the renders_ vector
    range = boost::make_iterator_range(fs::directory_iterator(rendDir_), {});
    for (const auto& entry : range) {
        if (fs::is_directory(entry)) {
            auto r = Render::New(entry);
            renders_.emplace(r->id(), r);
        }
    }
}

VolumePkg::Pointer VolumePkg::New(fs::path fileLocation, int version)
{
    return std::make_shared<VolumePkg>(fileLocation, version);
}

// Shared pointer volumepkg construction
VolumePkg::Pointer VolumePkg::New(fs::path fileLocation)
{
    return std::make_shared<VolumePkg>(fileLocation);
}

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
std::string VolumePkg::name() const
{
    // Gets the Volume name from the configuration file
    auto name = config_.get<std::string>("name");
    if (name != "NULL") {
        return name;
    }

    return "UnnamedVolume";
}

int VolumePkg::version() const { return config_.get<int>("version"); }

double VolumePkg::materialThickness() const
{
    return config_.get<double>("materialthickness");
}

// VOLUME FUNCTIONS //
std::vector<Volume::Identifier> VolumePkg::volumeIDs() const
{
    std::vector<Volume::Identifier> ids;
    for (auto& v : volumes_) {
        ids.emplace_back(v.first);
    }
    return ids;
}

std::vector<std::string> VolumePkg::volumeNames() const
{
    std::vector<Volume::Identifier> names;
    for (auto& v : volumes_) {
        names.emplace_back(v.second->name());
    }
    return names;
}

Volume::Pointer VolumePkg::newVolume(std::string name)
{
    // Generate a uuid
    auto uuid = DateTime();

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Make the volume directory
    auto volDir = volsDir_ / uuid;
    if (!fs::exists(volDir)) {
        fs::create_directory(volDir);
    } else {
        throw std::runtime_error("Volume directory already exists");
    }

    // Make the volume
    auto r = volumes_.emplace(uuid, Volume::New(volDir, uuid, name));
    if (!r.second) {
        auto msg = "Volume already exists with id " + uuid;
        throw std::runtime_error(msg);
    }

    // Return the Volume Pointer
    return r.first->second;
}

// SEGMENTATION FUNCTIONS //
std::vector<Segmentation::Identifier> VolumePkg::segmentationIDs() const
{
    std::vector<Segmentation::Identifier> ids;
    for (auto& s : segmentations_) {
        ids.emplace_back(s.first);
    }
    return ids;
}

std::vector<std::string> VolumePkg::segmentationNames() const
{
    std::vector<std::string> names;
    for (auto& s : segmentations_) {
        names.emplace_back(s.second->name());
    }
    return names;
}

// Make a new folder inside the volume package to house everything for this
// segmentation and push back the new segmentation into our vector of
// segmentations_
Segmentation::Pointer VolumePkg::newSegmentation(std::string name)
{
    // Generate a uuid
    auto uuid = DateTime();

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Make the volume directory
    auto segDir = segsDir_ / uuid;
    if (!fs::exists(segDir)) {
        fs::create_directory(segDir);
    } else {
        throw std::runtime_error("Segmentation directory already exists");
    }

    // Make the Segmentation
    auto r =
        segmentations_.emplace(uuid, Segmentation::New(segDir, uuid, name));
    if (!r.second) {
        auto msg = "Segmentation already exists with id " + uuid;
        throw std::runtime_error(msg);
    }

    // Return the Segmentation Pointer
    return r.first->second;
}

// RENDER FUNCTIONS //
std::vector<Render::Identifier> VolumePkg::renderIDs() const
{
    std::vector<Render::Identifier> ids;
    for (auto& r : renders_) {
        ids.emplace_back(r.first);
    }
    return ids;
}

std::vector<std::string> VolumePkg::renderNames() const
{
    std::vector<std::string> names;
    for (auto& r : renders_) {
        names.emplace_back(r.second->name());
    }
    return names;
}

// Make a new folder inside the volume package to house everything for this
// Render and push back the new render into our vector of renders_
Render::Pointer VolumePkg::newRender(std::string name)
{
    // Generate a uuid
    auto uuid = DateTime();

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Make the volume directory
    auto renDir = rendDir_ / uuid;
    if (!fs::exists(renDir)) {
        fs::create_directory(renDir);
    } else {
        throw std::runtime_error("Render directory already exists");
    }

    // Make the Render
    auto r = renders_.emplace(uuid, Render::New(renDir, uuid, name));
    if (!r.second) {
        auto msg = "Render already exists with id " + uuid;
        throw std::runtime_error(msg);
    }

    // Return the Render Pointer
    return r.first->second;
}

volcart::Metadata VolumePkg::InitConfig(
    const volcart::Dictionary& dict, int version)
{
    volcart::Metadata config;

    // Populate the config file with keys from the dictionary
    for (const auto& entry : dict) {
        if (entry.first == "version") {
            config.set("version", version);
            continue;
        }

        // Default values
        switch (entry.second) {
            case volcart::Type::INT:
                config.set(entry.first, int{});
                break;
            case volcart::Type::DOUBLE:
                config.set(entry.first, double{});
                break;
            case volcart::Type::STRING:
                config.set(entry.first, std::string{});
                break;
        }
    }

    return config;
}
