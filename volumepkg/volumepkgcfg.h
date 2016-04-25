#ifndef _VOLUMEPKGCFG_H_
#define _VOLUMEPKGCFG_H_

#include <iostream>
#include <fstream>

#include "vc_defines.h"
#include "vc_datatypes.h"

#include "volumepkg_version.h"

class VolumePkgCfg : public volcart::Metadata
{
public:
    VolumePkgCfg();
    VolumePkgCfg(const boost::filesystem::path& path) : Metadata(path){};
    VolumePkgCfg(const volcart::Dictionary& dict,
                 double version = VOLPKG_VERSION);
};

#endif  // _VOLUMEPKGCFG_H_
