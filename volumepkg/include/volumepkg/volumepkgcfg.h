#ifndef _VOLUMEPKGCFG_H_
#define _VOLUMEPKGCFG_H_

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "common/vc_defines.h"
#include "common/types/Metadata.h"
#include "volumepkg/volumepkg_version.h"

class VolumePkgCfg : public volcart::Metadata
{
public:
    VolumePkgCfg();
    VolumePkgCfg(const boost::filesystem::path& path) : Metadata(path){};
    VolumePkgCfg(const volcart::Dictionary& dict,
                 double version = VOLPKG_VERSION);
};

#endif  // _VOLUMEPKGCFG_H_
