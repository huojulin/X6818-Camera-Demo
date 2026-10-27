#!/bin/sh
# transcode_avi.sh -- 板子端：把 MJPEG-AVI 转成 Windows 原生可播的 H.264/yuv420p mp4
# 用法：sh /root/transcode_avi.sh <rec_xxx.avi>
# 依赖：/usr/bin/ffmpeg（由 build_ffmpeg.sh 交叉编译后推到板子）
set -u
AVI="$1"
[ -z "$AVI" ] && { echo "用法: sh transcode_avi.sh <file.avi>"; exit 1; }
[ -f "$AVI" ] || { echo "找不到 $AVI"; exit 1; }
if ! which ffmpeg >/dev/null 2>&1; then
    echo "[transcode] 板子没有 ffmpeg，无法转码（先跑 build_ffmpeg.sh 并 scp 到板子 /usr/bin/ffmpeg）"
    exit 2
fi
MP4="${AVI%.avi}.mp4"
echo "[transcode] $AVI -> $MP4"
ffmpeg -y -i "$AVI" -c:v libx264 -pix_fmt yuv420p -profile:v high -level 4.0 -preset fast -crf 23 -movflags +faststart "$MP4" 2>/tmp/transcode.log
if [ -f "$MP4" ] && [ -s "$MP4" ]; then
    echo "[transcode] 成功: $MP4  ($(wc -c < "$MP4") bytes)"
    echo "[transcode] 保留原 avi（$AVI）供板载 mplayer 直接回放 MJPEG"
else
    echo "[transcode] 失败，保留原 avi（详见 /tmp/transcode.log）"
    tail -5 /tmp/transcode.log
fi
