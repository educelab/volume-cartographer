#include "vc/core/types/VolumePkg.hpp"

#include <functional>
#include <utility>

#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;

namespace fs = volcart::filesystem;

namespace
{
constexpr auto CONFIG = "config.json";

auto VolpkgV3ToV4(const Metadata& meta) -> Metadata
{
    // Nothing to do
    if (meta.get<int>("version") != 3) {
        return meta;
    }
    Logger()->info("Performing v4 migrations");

    // VolumePkg path
    const auto path = meta.path().parent_path();

    // Write the new volpkg metadata
    Logger()->debug("- Creating primary metadata");
    Metadata newMeta;
    newMeta.set("version", 4);
    newMeta.set("name", meta.get<std::string>("volumepkg name"));
    newMeta.set("materialthickness", meta.get<double>("materialthickness"));
    newMeta.save(path / "config.json");

    // Make the "volumes" directory
    Logger()->debug("- Creating volumes directory");
    const fs::path volumesDir = path / "volumes";
    if (!fs::exists(volumesDir)) {
        fs::create_directory(volumesDir);
    }

    // Set up a new Volume name and make a new folder for it
    // Move the slices
    Logger()->debug("- Migrating v3 volume");
    const auto id = DateTime();
    const auto newVolDir = volumesDir / id;
    fs::rename(path / "slices", newVolDir);

    // Setup and save the metadata to the new Volume folder
    Metadata volMeta;
    volMeta.set("uuid", id);
    volMeta.set("name", id);
    volMeta.set("width", meta.get<int>("width"));
    volMeta.set("height", meta.get<int>("height"));
    volMeta.set("slices", meta.get<int>("number of slices"));
    volMeta.set("voxelsize", meta.get<double>("voxelsize"));
    volMeta.set("min", meta.get<double>("min"));
    volMeta.set("max", meta.get<double>("max"));
    volMeta.save(newVolDir / "meta.json");

    return newMeta;
}

auto VolpkgV4ToV5(const Metadata& meta) -> Metadata
{
    // Nothing to do check
    if (meta.get<int>("version") != 4) {
        return meta;
    }
    Logger()->info("Performing v5 migrations");

    // VolumePkg path
    const auto path = meta.path().parent_path();

    // Add metadata to all the segmentations
    Logger()->debug("- Initializing segmentation metadata");
    fs::path seg;
    const fs::path segsDir = path / "paths";
    for (const auto& entry : fs::directory_iterator(segsDir)) {
        if (fs::is_directory(entry)) {
            // Get the folder as a fs::path
            seg = entry;

            // Generate basic metadata
            Metadata segMeta;
            segMeta.set("uuid", seg.stem().string());
            segMeta.set("name", seg.stem().string());
            segMeta.set("type", "seg");

            // Link the metadata to the vcps file
            if (fs::exists(seg / "pointset.vcps")) {
                segMeta.set("vcps", "pointset.vcps");
            } else {
                segMeta.set("vcps", std::string{});
            }

            // Save the new metadata
            segMeta.save(seg / "meta.json");
        }
    }

    // Add renders folder
    Logger()->debug("- Adding renders directory");
    const fs::path rendersDir = path / "renders";
    if (!fs::exists(rendersDir)) {
        fs::create_directory(rendersDir);
    }

    // Update the version
    auto newMeta = meta;
    newMeta.set("version", 5);
    newMeta.save();

    return newMeta;
}

auto VolpkgV5ToV6(const Metadata& meta) -> Metadata
{
    // Nothing to do check
    if (meta.get<int>("version") != 5) {
        return meta;
    }
    Logger()->info("Performing v6 migrations");

    // VolumePkg path
    const auto path = meta.path().parent_path();

    // Add metadata to all the volumes
    Logger()->debug("- Updating volume metadata");
    fs::path vol;
    const fs::path volsDir = path / "volumes";
    for (const auto& entry : fs::directory_iterator(volsDir)) {
        if (fs::is_directory(entry)) {
            // Get the folder as a fs::path
            vol = entry;

            // Generate basic metadata
            Metadata volMeta(vol / "meta.json");
            if (!volMeta.hasKey("uuid")) {
                volMeta.set("uuid", vol.stem().string());
            }
            if (!volMeta.hasKey("name")) {
                volMeta.set("name", vol.stem().string());
            }
            if (!volMeta.hasKey("type")) {
                volMeta.set("type", "vol");
            }

            // Save the new metadata
            volMeta.save();
        }
    }

    // Update the version
    auto newMeta = meta;
    newMeta.set("version", 6);
    newMeta.save();

    return newMeta;
}

auto VolpkgV6ToV7(const Metadata& meta) -> Metadata
{
    // Nothing to do check
    if (meta.get<int>("version") != 6) {
        return meta;
    }
    Logger()->info("Performing v7 migrations");

    // VolumePkg path
    const auto path = meta.path().parent_path();

    // Add renders folder
    Logger()->debug("- Adding transforms directory");
    const fs::path tfmsDir = path / "transforms";
    if (not fs::exists(tfmsDir)) {
        fs::create_directory(tfmsDir);
    }

    // Add vc keep files
    Logger()->debug("- Adding keep files");
    for (const auto& d : {"paths", "renders", "volumes", "transforms"}) {
        if (not fs::exists(path / d / ".vckeep")) {
            std::ofstream(path / d / ".vckeep");
        }
    }

    // Update the version
    auto newMeta = meta;
    newMeta.set("version", 7);
    newMeta.save();

    return newMeta;
}

using UpgradeFn = std::function<Metadata(const Metadata&)>;
const std::vector<UpgradeFn> UPGRADE_FNS{
    VolpkgV3ToV4, VolpkgV4ToV5, VolpkgV5ToV6, VolpkgV6ToV7};
}  // namespace

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(fs::path fileLocation, int version)
    : rootDir_{std::move(fileLocation)}
{
    // Lookup the metadata template from our library of versions
    auto findDict = VERSION_LIBRARY.find(version);
    if (findDict == std::end(VERSION_LIBRARY)) {
        throw std::runtime_error("No dictionary found for volpkg");
    }

    // Create the directories with the default values
    config_ = VolumePkg::InitConfig(findDict->second, version);
    config_.setPath(rootDir_ / ::CONFIG);

    // Make directories
    for (const auto& d : required_dirs_()) {
        if (not fs::exists(d)) {
            fs::create_directory(d);
        }
        if (d != rootDir_ and not fs::exists(d / ".vckeep")) {
            std::ofstream(d / ".vckeep");
        }
    }

    // Do initial save
    config_.save();
}

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(const fs::path& fileLocation) : rootDir_{fileLocation}
{
    // Loads the metadata
    config_ = Metadata(fileLocation / ::CONFIG);

    // Auto-upgrade on load from v
    auto version = config_.get<int>("version");
    if (version >= 6 and version != VOLPKG_VERSION_LATEST) {
        Upgrade(fileLocation, VOLPKG_VERSION_LATEST);
        config_ = Metadata(fileLocation / ::CONFIG);
    }

    // Check directory structure
    for (const auto& d : required_dirs_()) {
        if (not fs::exists(d)) {
            Logger()->warn(
                "Creating missing VolumePkg directory: {}",
                d.filename().string());
            fs::create_directory(d);
        }
        if (d != rootDir_ and not fs::exists(d / ".vckeep")) {
            std::ofstream(d / ".vckeep");
        }
    }

    // Load volumes into volumes_
    for (const auto& entry : fs::directory_iterator(vols_dir_())) {
        if (fs::is_directory(entry)) {
            auto v = Volume::New(entry);
            volumes_.emplace(v->id(), v);
        }
    }

    // Load segmentations into the segmentations_
    for (const auto& entry : fs::directory_iterator(segs_dir_())) {
        if (fs::is_directory(entry)) {
            auto s = Segmentation::New(entry);
            segmentations_.emplace(s->id(), s);
        }
    }

    // Load Renders into the renders_
    for (const auto& entry : fs::directory_iterator(rend_dir_())) {
        if (fs::is_directory(entry)) {
            auto r = Render::New(entry);
            renders_.emplace(r->id(), r);
        }
    }

    // Load the transform files into transforms_
    for (const auto& entry : fs::directory_iterator(tfm_dir_())) {
        auto ep = entry.path();
        if (fs::is_regular_file(entry) and ep.extension() == ".json") {
            Transform3D::Pointer tfm;
            try {
                tfm = Transform3D::Load(ep);
            } catch (const std::exception& e) {
                Logger()->warn(
                    "Failed to load transform \"{}\". {}",
                    ep.filename().string(), e.what());
                continue;
            }
            transforms_.emplace(ep.stem(), tfm);
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

    // Make the volume directory
    auto volDir = vols_dir_() / uuid;
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

    // Make the volume directory
    auto segDir = segs_dir_() / uuid;
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
auto VolumePkg::hasRenders() -> bool { return !renders_.empty(); }
auto VolumePkg::numberOfRenders() -> size_t { return renders_.size(); }
auto VolumePkg::render(const Render::Identifier& id) const
    -> const Render::Pointer
{
    return renders_.at(id);
}
auto VolumePkg::render(const Render::Identifier& id) -> Render::Pointer
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
    while (renders_.count(uuid) > 0) {
        uuid = DateTime();
    }

    // Get dir name if not specified
    if (name.empty()) {
        name = uuid;
    }

    // Make the render directory
    auto renDir = rend_dir_() / uuid;
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

auto VolumePkg::addTransform(const Transform3D::Pointer& transform)
    -> Transform3D::Identifier
{
    // Make sure the transform has a source and target
    if (transform->source().empty()) {
        throw std::invalid_argument("Transform is missing source");
    }
    if (transform->target().empty()) {
        throw std::invalid_argument("Transform is missing target");
    }

    // Make sure we have a transforms directory
    if (not fs::exists(tfm_dir_())) {
        Logger()->debug("Creating transforms directory");
        fs::create_directory(tfm_dir_());
    }

    // Generate a uuid
    auto uuid = DateTime();
    auto tfmPath = tfm_dir_() / (uuid + ".json");
    while (fs::exists(tfmPath) or transforms_.count(uuid) > 0) {
        uuid = DateTime();
        tfmPath = tfmPath.replace_filename(uuid + ".json");
    }

    // Add to the internal ID map
    auto r = transforms_.insert({uuid, transform});
    if (!r.second) {
        auto msg = "Transform already exists with id " + uuid;
        throw std::runtime_error(msg);
    }

    // Write to disk
    Transform3D::Save(tfmPath, transform);

    return uuid;
}

void VolumePkg::setTransform(
    const Transform3D::Identifier& id, const Transform3D::Pointer& transform)
{
    if (transforms_.count(id) == 0) {
        throw std::range_error("Transform " + id + " does not exist");
    }

    // update the map
    transforms_[id] = transform;
    // update on disk
    auto tfmPath = tfm_dir_() / id;
    tfmPath = tfmPath.replace_extension("json");
    Transform3D::Save(tfmPath, transform);
}

auto VolumePkg::transform(const Transform3D::Identifier& id)
    -> Transform3D::Pointer
{
    return transforms_.at(id);
}

auto VolumePkg::transform(
    const Volume::Identifier& src, const Volume::Identifier& tgt)
    -> std::vector<std::pair<Transform3D::Identifier, Transform3D::Pointer>>
{
    std::vector<std::pair<Transform3D::Identifier, Transform3D::Pointer>> tfms;
    for (const auto& [id, tfm] : transforms_) {
        if (tfm->source() == src and tfm->target() == tgt) {
            tfms.emplace_back(id, tfm);
        } else if (
            tfm->invertible() and tfm->source() == tgt and
            tfm->target() == src) {
            auto idI = id + "*";
            tfms.emplace_back(idI, tfm->invert());
        }
    }

    return tfms;
}

auto VolumePkg::transformIDs() const -> std::vector<Transform3D::Identifier>
{
    std::vector<Transform3D::Identifier> keys;
    std::transform(
        transforms_.begin(), transforms_.end(), std::back_inserter(keys),
        [](const auto& p) { return p.first; });
    return keys;
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

auto VolumePkg::vols_dir_() const -> fs::path { return rootDir_ / "volumes"; }

auto VolumePkg::segs_dir_() const -> fs::path { return rootDir_ / "paths"; }

auto VolumePkg::rend_dir_() const -> fs::path { return rootDir_ / "renders"; }

auto VolumePkg::tfm_dir_() const -> fs::path { return rootDir_ / "transforms"; }

auto VolumePkg::required_dirs_() -> std::vector<filesystem::path>
{
    return {rootDir_, vols_dir_(), segs_dir_(), rend_dir_(), tfm_dir_()};
}

////////// Upgrade //////////
void VolumePkg::Upgrade(const fs::path& path, int version, bool force)
{
    // Copy the current metadata
    Metadata meta(path / "config.json");

    // Get current version
    const auto currentVersion = meta.get<int>("version");

    // Don't update for versions < 6 unless forced (those migrations are
    // expensive)
    if (currentVersion < 6 and not force) {
        throw std::runtime_error(
            "Volumepkg version " + std::to_string(currentVersion) +
            " should be upgraded with vc_volpkg_upgrade");
    }

    Logger()->info(
        "Upgrading volpkg version {} to {}", currentVersion, version);

    // Plot path to final version
    // UpgradeFns start at v3->v4
    auto startIdx = currentVersion - 3;
    auto endIdx = version - 3;
    for (auto idx = startIdx; idx < endIdx; idx++) {
        meta = ::UPGRADE_FNS[idx](meta);
    }
    // Save the final metadata
    meta.save();
}
