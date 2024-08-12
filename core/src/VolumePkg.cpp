#include "vc/core/types/VolumePkg.hpp"

#include <fstream>
#include <functional>
#include <set>
#include <utility>

#include <educelab/core/utils/String.hpp>

#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;

namespace el = educelab;
namespace fs = volcart::filesystem;

namespace
{
////// Convenience vars and fns for accessing VolumePkg sub-paths //////
constexpr auto CONFIG = "config.json";

auto VolsDir(const fs::path& baseDir) -> fs::path
{
    return baseDir / "volumes";
}

auto SegsDir(const fs::path& baseDir) -> fs::path { return baseDir / "paths"; }

auto RendDir(const fs::path& baseDir) -> fs::path
{
    return baseDir / "renders";
}

auto TfmDir(const fs::path& baseDir) -> fs::path
{
    return baseDir / "transforms";
}

auto ReqDirs(const fs::path& baseDir) -> std::vector<filesystem::path>
{
    return {
        baseDir, ::VolsDir(baseDir), ::SegsDir(baseDir), ::RendDir(baseDir),
        ::TfmDir(baseDir)};
}

void keep(const fs::path& dir)
{
    if (not fs::exists(dir / ".vckeep")) {
        std::ofstream(dir / ".vckeep", std::ostream::ate);
    }
}

////// Upgrade functions //////
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
    newMeta.set("name", meta.get<std::string>("volumepkg name").value());
    newMeta.set(
        "materialthickness", meta.get<double>("materialthickness").value());
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
    volMeta.set("width", meta.get<int>("width").value());
    volMeta.set("height", meta.get<int>("height").value());
    volMeta.set("slices", meta.get<int>("number of slices").value());
    volMeta.set("voxelsize", meta.get<double>("voxelsize").value());
    volMeta.set("min", meta.get<double>("min").value());
    volMeta.set("max", meta.get<double>("max").value());
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
        ::keep(path / d);
    }

    // Update the version
    auto newMeta = meta;
    newMeta.set("version", 7);
    newMeta.save();

    return newMeta;
}

auto VolpkgV7ToV8(const Metadata& meta) -> Metadata
{
    // Nothing to do check
    if (meta.get<int>("version") != 7) {
        return meta;
    }
    Logger()->info("Performing v8 migrations");

    // VolumePkg path
    const auto path = meta.path().parent_path();

    // Replace empty strings in Segmentation metadata
    Logger()->debug("- Updating segmentation metadata");
    fs::path seg;
    const auto segsDir = path / "paths";
    for (const auto& entry : fs::directory_iterator(segsDir)) {
        if (fs::is_directory(entry)) {
            // Get the folder as a fs::path
            seg = entry;

            // Load the metadata
            Metadata segMeta(seg / "meta.json");

            // Set null on the appropriate values
            for (auto key : {"vcps", "volume"}) {
                if (not segMeta.hasKey(key)) {
                    segMeta.set(key, nlohmann::json::value_t::null);
                } else {
                    const auto v = segMeta.get<std::string>(key);
                    if (v.has_value() and v.value().empty()) {
                        segMeta.set(key, nlohmann::json::value_t::null);
                    }
                }
            }

            // Save the new metadata
            segMeta.save();
        }
    }

    // Update the version
    auto newMeta = meta;
    newMeta.set("version", 8);
    newMeta.save();

    return newMeta;
}

using UpgradeFn = std::function<Metadata(const Metadata&)>;
const std::vector<UpgradeFn> UPGRADE_FNS{
    VolpkgV3ToV4, VolpkgV4ToV5, VolpkgV5ToV6, VolpkgV6ToV7, VolpkgV7ToV8};

/*
 * Runs BFS on tfms from src to tgt, returning the shortest paths first. This
 * implementation does not return every possible transform path, but prunes
 * cycles and paths with transforms which are already included in shorter paths.
 */
auto FindShortestPaths(
    const std::map<Transform3D::Identifier, Transform3D::Pointer>& tfms,
    const Volume::Identifier& src,
    const Volume::Identifier& tgt)
{
    // Local short hand for result type
    using NamedTransform =
        std::pair<Transform3D::Identifier, Transform3D::Pointer>;
    std::vector<NamedTransform> results;

    // BFS tree node
    struct Node {
        using Ptr = std::shared_ptr<Node>;
        Transform3D::Identifier id;
        Transform3D::Pointer tfm;
        std::shared_ptr<Node> parent{nullptr};
    };

    // Initialize the queue and visited list
    std::set<Transform3D::Identifier> visited;
    std::list<Node::Ptr> queue;
    for (const auto& [id, val] : tfms) {
        // Skip transforms without source and target IDs
        /* Not currently possible to have empty src/tgt in VolPkg
        if (val->source().empty() or val->target().empty()) {
            continue;
        }
        */

        // Queue transforms which start at our source
        if (val->source() == src) {
            visited.emplace(id);
            auto p = std::make_shared<Node>();
            p->id = id;
            p->tfm = val;
            queue.push_back(p);
        }

        // Queue invertible transforms which end at our source
        if (val->invertible() and val->target() == src) {
            visited.emplace(id + "*");
            auto p = std::make_shared<Node>();
            p->id = id + "*";
            p->tfm = val->invert();
            queue.push_back(p);
        }
    }

    // Iterate over the queue
    while (not queue.empty()) {
        // Pop next queue item
        auto n = queue.front();
        queue.pop_front();

        // If we've reached the target...
        if (n->tfm->target() == tgt) {
            // Node has no parent, so return original transform
            if (not n->parent) {
                results.emplace_back(n->id, n->tfm);
            }

            // Node has a parent, so return a composite transform
            else {
                Transform3D::Identifier id;
                auto c = CompositeTransform::New();
                c->source(src);
                c->target(tgt);
                // Iterate path from target to source
                while (n->parent) {
                    id = "->" + n->id + id;
                    c->push_front(n->tfm);
                    n = n->parent;
                }
                c->push_front(n->tfm);
                results.emplace_back(n->id + id, c);
            }
            // done with this node
            continue;
        }

        // Find the transforms which extend the path
        // TODO: Linear in # of transforms
        for (const auto& [id, val] : tfms) {
            // Skip visited nodes
            if (visited.count(id) > 0 or visited.count(id + "*") > 0) {
                continue;
            }

            // Queue transforms which start at our local source
            if (val->source() == n->tfm->target()) {
                visited.emplace(id);
                auto p = std::make_shared<Node>();
                p->id = id;
                p->tfm = val;
                p->parent = n;
                queue.push_back(p);
            }

            // Queue invertible transforms which end at our local source
            else if (val->invertible() and val->target() == n->tfm->target()) {
                visited.emplace(id + "*");
                auto p = std::make_shared<Node>();
                p->id = id + "*";
                p->tfm = val->invert();
                p->parent = n;
                queue.push_back(p);
            }
        }
    }

    return results;
}

}  // namespace

// CONSTRUCTORS //
// Make a volpkg of a particular version number
VolumePkg::VolumePkg(fs::path path, const int version)
    : rootDir_{std::move(path)}
{
    // Don't overwrite existing directories
    if (fs::exists(rootDir_)) {
        throw std::runtime_error("File exists at path: " + rootDir_.string());
    }

    // Lookup the metadata template from our library of versions
    const auto findDict = VERSION_LIBRARY.find(version);
    if (findDict == std::end(VERSION_LIBRARY)) {
        throw std::runtime_error("No dictionary found for volpkg");
    }

    // Create the directories with the default values
    config_ = VolumePkg::InitConfig(findDict->second, version);
    config_.setPath(rootDir_ / ::CONFIG);

    // Make directories
    for (const auto& d : ::ReqDirs(rootDir_)) {
        if (not fs::exists(d)) {
            fs::create_directory(d);
        }
        if (d != rootDir_) {
            ::keep(d);
        }
    }

    // Do initial save
    config_.save();
}

// Use this when reading a volpkg from a file
VolumePkg::VolumePkg(const fs::path& path) : rootDir_{path}
{
    // Loads the metadata
    config_ = Metadata(path / ::CONFIG);

    // Auto-upgrade on load from v
    const auto version = config_.get<int>("version");
    if (version >= 6 and version != VOLPKG_VERSION_LATEST) {
        Upgrade(path, VOLPKG_VERSION_LATEST);
        config_ = Metadata(path / ::CONFIG);
    }

    // Check directory structure
    for (const auto& d : ::ReqDirs(rootDir_)) {
        if (not fs::exists(d)) {
            Logger()->warn(
                "Creating missing VolumePkg directory: {}",
                d.filename().string());
            fs::create_directory(d);
        }
        if (d != rootDir_) {
            ::keep(d);
        }
    }

    // Load volumes into volumes_
    for (const auto& entry : fs::directory_iterator(::VolsDir(rootDir_))) {
        if (fs::is_directory(entry)) {
            if (not exists(entry.path() / "meta.json")) {
                Logger()->warn(
                    "Ignoring volume '{}': Does not contain metadata file",
                    entry.path().filename().string());
                continue;
            }
            try {
                auto v = Volume::New(entry);
                volumes_.emplace(v->id(), v);
            } catch (const std::exception& e) {
                Logger()->warn(
                    "Failed to load volume '{}': {}",
                    entry.path().filename().string(), e.what());
            }
        }
    }

    // Load segmentations into segmentations_
    for (const auto& entry : fs::directory_iterator(::SegsDir(rootDir_))) {
        if (fs::is_directory(entry)) {
            if (not exists(entry.path() / "meta.json")) {
                Logger()->warn(
                    "Ignoring segmentation '{}': Does not contain metadata "
                    "file",
                    entry.path().filename().string());
                continue;
            }
            try {
                auto s = Segmentation::New(entry);
                segmentations_.emplace(s->id(), s);
            } catch (const std::exception& e) {
                Logger()->warn(
                    "Failed to load segmentation '{}': {}",
                    entry.path().filename().string(), e.what());
            }
        }
    }

    // Load Renders into renders_
    for (const auto& entry : fs::directory_iterator(::RendDir(rootDir_))) {
        if (fs::is_directory(entry)) {
            if (not exists(entry.path() / "meta.json")) {
                Logger()->warn(
                    "Ignoring render '{}': Does not contain metadata file",
                    entry.path().filename().string());
                continue;
            }
            try {
                auto r = Render::New(entry);
                renders_.emplace(r->id(), r);
            } catch (const std::exception& e) {
                Logger()->warn(
                    "Failed to load render '{}': {}",
                    entry.path().filename().string(), e.what());
            }
        }
    }

    // Load the transform files into transforms_
    for (const auto& entry : fs::directory_iterator(::TfmDir(rootDir_))) {
        const auto ep = entry.path();
        if (fs::is_regular_file(entry) and ep.extension() == ".json") {
            try {
                auto tfm = Transform3D::Load(ep);
                transforms_.emplace(ep.stem(), tfm);
            } catch (const std::exception& e) {
                Logger()->warn(
                    "Failed to load transform '{}'. {}", ep.filename().string(),
                    e.what());
            }
        }
    }
}

auto VolumePkg::New(const fs::path& fileLocation, int version) -> Pointer
{
    return std::make_shared<VolumePkg>(fileLocation, version);
}

// Shared pointer volumepkg construction
auto VolumePkg::New(const fs::path& fileLocation) -> Pointer
{
    return std::make_shared<VolumePkg>(fileLocation);
}

// METADATA RETRIEVAL //
// Returns Volume Name from JSON config
auto VolumePkg::name() const -> std::string
{
    // Gets the Volume name from the configuration file
    if (const auto name = config_.get<std::string>("name"); name.has_value()) {
        return name.value();
    }

    return "UnnamedVolume";
}

auto VolumePkg::version() const -> int
{
    return config_.get<int>("version").value();
}

auto VolumePkg::materialThickness() const -> double
{
    return config_.get<double>("materialthickness").value();
}

auto VolumePkg::metadata() const -> Metadata { return config_; }

void VolumePkg::saveMetadata() const { config_.save(); }

void VolumePkg::saveMetadata(const fs::path& filePath) const
{
    config_.save(filePath);
}

// VOLUME FUNCTIONS //
auto VolumePkg::hasVolumes() const -> bool { return !volumes_.empty(); }

auto VolumePkg::hasVolume(const Volume::Identifier& id) const -> bool
{
    return volumes_.count(id) > 0;
}

auto VolumePkg::numberOfVolumes() const -> std::size_t
{
    return volumes_.size();
}

auto VolumePkg::volumeIDs() const -> std::vector<Volume::Identifier>
{
    std::vector<Volume::Identifier> ids;
    ids.reserve(volumes_.size());
    for (const auto& [id, _] : volumes_) {
        ids.emplace_back(id);
    }
    return ids;
}

auto VolumePkg::volumeNames() const -> std::vector<std::string>
{
    std::vector<Volume::Identifier> names;
    names.reserve(volumes_.size());
    for (const auto& [_, vol] : volumes_) {
        names.emplace_back(vol->name());
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
    auto volDir = ::VolsDir(rootDir_) / uuid;
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
auto VolumePkg::hasSegmentations() const -> bool
{
    return !segmentations_.empty();
}

auto VolumePkg::numberOfSegmentations() const -> std::size_t
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
    auto segDir = ::SegsDir(rootDir_) / uuid;
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

auto VolumePkg::removeSegmentation(const Segmentation::Identifier& id) -> bool
{
    if (id.size() == 0)
        return false;

    // Remove the volume directory
    auto segDir = ::SegsDir(rootDir_) / id;
    if (!fs::exists(segDir)) {
        throw std::runtime_error("Segmentation directory does not exist for ID " + id);
    } else {
        return fs::remove_all(segDir);
    }
}

// RENDER FUNCTIONS //
auto VolumePkg::hasRenders() const -> bool { return !renders_.empty(); }
auto VolumePkg::numberOfRenders() const -> std::size_t
{
    return renders_.size();
}
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
    auto renDir = ::RendDir(rootDir_) / uuid;
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

auto VolumePkg::hasTransforms() const -> bool
{
    return not transforms_.empty();
}

auto VolumePkg::hasTransform(const Transform3D::Identifier& id) const -> bool
{
    // Don't allow empty IDs
    if (id.empty()) {
        throw std::invalid_argument("Transform ID is empty");
    }

    // Split by ->
    const auto ids = el::split(id, "->");
    const bool isMulti = ids.size() > 1;

    // Iterate over the transform IDs
    for (auto i : ids) {
        // Remove the star for inverse transforms
        const bool findInverse = i.back() == '*';
        if (findInverse) {
            i.remove_suffix(1);
        }

        // Find the forward transform
        const auto iStr = std::string(i);
        auto found = transforms_.count(std::string(i)) > 0;

        // Invert if requested
        if (found and findInverse) {
            found = transforms_.at(iStr)->invertible();
        }

        // If ever not found or is single transform, return
        if (not found or not isMulti) {
            return found;
        }
    }

    // If we've made it here, we've found all parts
    return true;
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
    if (not fs::exists(::TfmDir(rootDir_))) {
        Logger()->debug("Creating transforms directory");
        fs::create_directory(::TfmDir(rootDir_));
    }

    // Generate a uuid
    auto uuid = DateTime();
    auto tfmPath = ::TfmDir(rootDir_) / (uuid + ".json");
    while (fs::exists(tfmPath) or transforms_.count(uuid) > 0) {
        uuid = DateTime();
        tfmPath = tfmPath.replace_filename(uuid + ".json");
    }

    // Add to the internal ID map
    auto r = transforms_.insert({uuid, transform});
    if (not r.second) {
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
    // Don't allow empty IDs
    if (id.empty()) {
        throw std::invalid_argument("Transform ID is empty");
    }

    // See if the
    if (transforms_.count(id) == 0) {
        throw std::range_error("Transform " + id + " does not exist");
    }

    // update the map
    transforms_[id] = transform;
    // update on disk
    auto tfmPath = ::TfmDir(rootDir_) / id;
    tfmPath = tfmPath.replace_extension("json");
    Transform3D::Save(tfmPath, transform);
}

auto VolumePkg::transform(Transform3D::Identifier id) const
    -> Transform3D::Pointer
{
    // Don't allow empty IDs
    if (id.empty()) {
        throw std::invalid_argument("Transform ID is empty");
    }

    // Split by ->
    const auto ids = el::split(id, "->");
    const bool isMulti = ids.size() > 1;

    // Result
    Transform3D::Pointer tfm;
    CompositeTransform::Pointer cmp;
    if (isMulti) {
        cmp = CompositeTransform::New();
        tfm = cmp;
    }

    // Iterate over the transform IDs
    for (auto i : ids) {
        // Remove the star for inverse transforms
        const bool getInverse = i.back() == '*';
        if (getInverse) {
            i.remove_suffix(1);
        }

        // Find the forward transform
        auto iStr = std::string(i);
        auto t = transforms_.at(iStr);

        // Invert if requested
        if (getInverse) {
            if (t->invertible()) {
                t = t->invert();
            } else {
                throw std::invalid_argument(
                    "Transform is not invertible: " + iStr);
            }
        }

        // Add to composite if needed
        if (isMulti) {
            cmp->push_back(t);
            // Set the source
            if (cmp->size() == 1) {
                cmp->source(t->source());
            }
            // Set the target
            if (cmp->size() == ids.size()) {
                cmp->target(t->target());
            }
        } else {
            tfm = t;
        }
    }

    return tfm;
}

auto VolumePkg::transform(
    const Volume::Identifier& src, const Volume::Identifier& tgt) const
    -> std::vector<std::pair<Transform3D::Identifier, Transform3D::Pointer>>
{
    return FindShortestPaths(transforms_, src, tgt);
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

////////// Upgrade //////////
void VolumePkg::Upgrade(const fs::path& path, int version, bool force)
{
    // Copy the current metadata
    Metadata meta(path / "config.json");

    // Get current version
    const auto currentVersion = meta.get<int>("version").value();

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
    const auto startIdx = currentVersion - 3;
    const auto endIdx = version - 3;
    for (auto idx = startIdx; idx < endIdx; idx++) {
        meta = ::UPGRADE_FNS[idx](meta);
    }
    // Save the final metadata
    meta.save();
}
