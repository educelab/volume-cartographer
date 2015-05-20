#!/bin/sh
root_dir=$PWD
for pkg in */; do
	cd ${pkg}/paths
	for path in */; do 
	    cd ${path}
	    echo "$(basename $pkg) :: $(basename $PWD)"
	    mkdir -p textures
	    
	    for filterindex in 0 1 3 4 5 6; do
	        if [ $filterindex == 0 ]; then
	            filtername="inter"
	        elif [ $filterindex == 1 ]; then
	            filtername="max"
	        elif [ $filterindex == 2 ]; then
	            filtername="max2"
	        elif [ $filterindex == 3 ]; then
	            filtername="min"
	        elif [ $filterindex == 4 ]; then
	            filtername="med+"
	        elif [ $filterindex == 5 ]; then
	            filtername="med"
	        elif [ $filterindex == 6 ]; then
	            filtername="mean"
	        fi

	        for direction in 0; do
	            
	            if [ $direction == 0 ]; then
	                dirname="omni"
	            elif [ $direction == 1 ]; then
	                dirname="pos"
	            elif [ $direction == 2 ]; then
	                dirname="neg"
	            fi

	            if [ $filterindex == 0 ]; then
	                radius=0
	            else
	                radius=7
	            fi

	            vc_texture2 ../../ $(basename $PWD) ${radius} ${filterindex} ${direction}
	            cp texture.tif textures/${filtername}.tif

	            if [ $filterindex == 0 ]; then
	                break
	            fi
	        done
	    done
	    mogrify -auto-level textures/*.tif
	    cd ..
	done
cd $root_dir
done