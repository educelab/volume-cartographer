#include "vc/core/types/VolumePkg.hpp"

#include "vc/core/util/DateTime.hpp"

using namespace volcart;

namespace fs = volcart::filesystem;

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
    auto findDict = VERSION_LIBRARY.find(version);
    if (findDict == std::end(VERSION_LIBRARY)) {
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
    config_ = Metadata(fileLocation / SUBPATH_META);

    // Load volumes into volumes_ vector
    for (const auto& entry : fs::directory_iterator(volsDir_)) {
        if (fs::is_directory(entry)) {
            auto v = Volume::New(entry);
            volumes_.emplace(v->id(), v);
        }
    }

    // Load segmentations into the segmentations_ vector
    for (const auto& entry : fs::directory_iterator(segsDir_)) {
        if (fs::is_directory(entry)) {
            auto s = Segmentation::New(entry);
            segmentations_.emplace(s->id(), s);
        }
    }

    // Load Renders into the renders_ vector
    for (const auto& entry : fs::directory_iterator(rendDir_)) {
        if (fs::is_directory(entry)) {
            auto r = Render::New(entry);
            renders_.emplace(r->id(), r);
        }
    }
}

auto VolumePkg::New(fs::path fileLocation, int version) -> VolumePkg::Pointer
{
    return std::make_shared<VolumePkg>(fileLocation, version);
}

// Shared pointer volumepkg construction
auto VolumePkg::New(fs::path fileLocation) -> VolumePkg::Pointer
{
    return std::make_shared<VolumePkg>(fileLocation);
}

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
auto VolumePkg::name() const -> std::string
{
    // Gets the Volume name from the configuration file
    auto name = config_.get<std::string>("name");
    if (name != "NULL") {
        return name;
    }

    return "UnnamedVolume";
}

auto VolumePkg::version() const -> int { return config_.get<int>("version"); }

auto VolumePkg::materialThickness() const -> double
{
    return config_.get<double>("materialthickness");
}

auto VolumePkg::metadata() const -> Metadata { return config_; }

void VolumePkg::saveMetadata() { config_.save(); }

void VolumePkg::saveMetadata(const fs::path& filePath)
{
    config_.save(filePath);
}

// VOLUME FUNCTIONS //
auto VolumePkg::hasVolumes() -> bool { return !volumes_.empty(); }

auto VolumePkg::hasVolume(const Volume::Identifier& id) const -> bool
{
    return volumes_.count(id) > 0;
}

auto VolumePkg::numberOfVolumes() -> size_t { return volumes_.size(); }

auto VolumePkg::volumeIDs() const -> std::vector<Volume::Identifier>
{
    std::vector<Volume::Identifier> ids;
    for (const auto& v : volumes_) {
        ids.emplace_back(v.first);
    }
    return ids;
}

auto VolumePkg::volumeNames() const -> std::vector<std::string>
{
    std::vector<Volume::Identifier> names;
    for (const auto& v : volumes_) {
        names.emplace_back(v.second->name());
    }
    return names;
}

auto VolumePkg::newVolume(std::string name) -> Volume::Pointer
{
    // Generate a uuid
    auto uuid = DateTime();

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Construct the volume directory path
    auto volDir = volsDir_ / uuid;
    if (!fs::exists(volDir)) {
        fs::create_directory(volDir);
    } else {
        throw std::runtime_error("Volume directory already exists");
    }

    // Make the volume
    auto r = volumes_.emplace(uuid, Volume::New(volDir, uuid, name));
    if (!r.second) {
        auto msg = "Volume already exists with ID " + uuid;
        throw std::runtime_error(msg);
    }

    // Return the Volume Pointer
    return r.first->second;
}

auto VolumePkg::volume() const -> const Volume::Pointer
{
    if (volumes_.empty()) {
        throw std::out_of_range("No volumes in VolPkg");
    }
    return volumes_.begin()->second;
}

auto VolumePkg::volume() -> Volume::Pointer
{
    if (volumes_.empty()) {
        throw std::out_of_range("No volumes in VolPkg");
    }
    return volumes_.begin()->second;
}

auto VolumePkg::volume(const Volume::Identifier& id) const
    -> const Volume::Pointer
{
    return volumes_.at(id);
}

auto VolumePkg::volume(const Volume::Identifier& id) -> Volume::Pointer
{
    return volumes_.at(id);
}

// SEGMENTATION FUNCTIONS //
auto VolumePkg::hasSegmentations() -> bool { return !segmentations_.empty(); }

auto VolumePkg::numberOfSegmentations() -> size_t
{
    return segmentations_.size();
}

auto VolumePkg::segmentation(const DiskBasedObjectBaseClass::Identifier& id)
    const -> const Segmentation::Pointer
{
    return segmentations_.at(id);
}

auto VolumePkg::segmentation(const DiskBasedObjectBaseClass::Identifier& id)
    -> Segmentation::Pointer
{
    return segmentations_.at(id);
}

auto VolumePkg::segmentationIDs() const -> std::vector<Segmentation::Identifier>
{
    std::vector<Segmentation::Identifier> ids;
    for (const auto& s : segmentations_) {
        ids.emplace_back(s.first);
    }
    return ids;
}

auto VolumePkg::segmentationNames() const -> std::vector<std::string>
{
    std::vector<std::string> names;
    for (const auto& s : segmentations_) {
        names.emplace_back(s.second->name());
    }
    return names;
}

// Make a new folder inside the volume package to house everything for this
// segmentation and push back the new segmentation into our vector of
// segmentations_
auto VolumePkg::newSegmentation(std::string name) -> Segmentation::Pointer
{
    // Generate a uuid
    auto uuid = DateTime();

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Construct the volume directory path
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
        auto msg = "Segmentation already exists with ID " + uuid;
        throw std::runtime_error(msg);
    }

    // Return the Segmentation Pointer
    return r.first->second;
}

auto VolumePkg::removeSegmentation(const Segmentation::Identifier& id) -> bool
{
    if(id.size() == 0)
        return false;

    // Construct the volume directory path
    auto segDir = segsDir_ / id;
    if (!fs::exists(segDir)) {
        throw std::runtime_error("Segmentation directory does not exist for ID " + id);
    } else {
        return fs::remove_all(segDir);
    }
}

// RENDER FUNCTIONS //
auto VolumePkg::hasRenders() -> bool { return !renders_.empty(); }
auto VolumePkg::numberOfRenders() -> size_t { return renders_.size(); }
auto VolumePkg::render(const DiskBasedObjectBaseClass::Identifier& id) const
    -> const Render::Pointer
{
    return renders_.at(id);
}
auto VolumePkg::render(const DiskBasedObjectBaseClass::Identifier& id)
    -> Render::Pointer
{
    return renders_.at(id);
}

auto VolumePkg::renderIDs() const -> std::vector<Render::Identifier>
{
    std::vector<Render::Identifier> ids;
    for (const auto& r : renders_) {
        ids.emplace_back(r.first);
    }
    return ids;
}

auto VolumePkg::renderNames() const -> std::vector<std::string>
{
    std::vector<std::string> names;
    for (const auto& r : renders_) {
        names.emplace_back(r.second->name());
    }
    return names;
}

// Make a new folder inside the volume package to house everything for this
// Render and push back the new render into our vector of renders_
auto VolumePkg::newRender(std::string name) -> Render::Pointer
{
    // Generate a uuid
    auto uuid = DateTime();

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Construct the volume directory path
    auto renDir = rendDir_ / uuid;
    if (!fs::exists(renDir)) {
        fs::create_directory(renDir);
    } else {
        throw std::runtime_error("Render directory already exists");
    }

    // Make the Render
    auto r = renders_.emplace(uuid, Render::New(renDir, uuid, name));
    if (!r.second) {
        auto msg = "Render already exists with ID " + uuid;
        throw std::runtime_error(msg);
    }

    // Return the Render Pointer
    return r.first->second;
}

auto VolumePkg::InitConfig(const Dictionary& dict, int version) -> Metadata
{
    Metadata config;

    // Populate the config file with keys from the dictionary
    for (const auto& entry : dict) {
        if (entry.first == "version") {
            config.set("version", version);
            continue;
        }

        // Default values
        switch (entry.second) {
            case DictionaryEntryType::Int:
                config.set(entry.first, int{});
                break;
            case DictionaryEntryType::Double:
                config.set(entry.first, double{});
                break;
            case DictionaryEntryType::String:
                config.set(entry.first, std::string{});
                break;
        }
    }

    return config;
}
