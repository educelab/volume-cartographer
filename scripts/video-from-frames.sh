#!/bin/sh

start_idx=0
fps=30

if [ "$#" -lt 2 ]; then
    echo "Usage: video-from-frames [input] [output] [opts]
Options:
       -s <start_frame>
       -f <fps>"
    exit 1
fi

while getopts "s:" opt; do
    case "$opt" in
    s)  start_idx=$OPTARG
        ;;
    esac
done

ffmpeg \
    -start_number ${start_idx} \
    -i $1 \
    -c:v "libx264" \
    -crf "18" \
    -maxrate "10M" \
    -bufsize "15M" \
    -pix_fmt "yuv420p" \
    -profile:v high \
    -level 42 \
    -vf "scale=iw*sar:ih,scale='w=if(lt(dar, 16/9), trunc(oh*a/2)*2, min(1920,ceil(iw/2)*2)):h=if(gte(dar, 16/9), trunc(ow/a/2)*2, min(1080,ceil(ih/2)*2))',setsar=1" \
    -movflags faststart \
    -c:a libfdk_aac \
    -b:a "320k" \
    -y $2