#!/bin/sh
# boot_cam.sh -- 复位后一键拉起「板载自包含相机系统」（固化在板子 /root，复位不丢）
# 职责（全部在板子本地，不依赖 PC / VM / 网络）：
#   1) 起 sshd（本地，仅维护用；无网络时无害）
#   2) 建相机节点（按 /sys 真实 major:minor，排除 MLC 显示层）
#   3) 起 phone_view（板载相机 + 显示 + 触摸按钮：拍照 / 录·停 / 回放 / 退出）
#   4) 起本地转码守护（录像完自动 AVI→MP4，供板载 mplayer 回放）
# 用法（板子串口）：  sh /root/boot_cam.sh
# 维护联网（仅开发期推二进制 / 调试时用）：另跑  sh /root/wake_ssh.sh
set -u
log(){ echo "[boot] $*"; }

log "1) 起 sshd（本地，维护用）"
mkdir -p /var/empty /run/sshd
[ -x /usr/sbin/sshd ] && /usr/sbin/sshd

log "2) 建相机节点（按 /sys 真实 major:minor）"
sh /root/setup_cam.sh

log "3) 定位相机节点 + 起 phone_view（板载相机 + 显示 + 触摸按钮）"
CAM=""
for d in /sys/class/video4linux/video*; do
    [ -e "$d/name" ] || continue
    n=$(cat "$d/name" 2>/dev/null)
    case "$n" in *MLC*|*SCALER*|*CLIPPER*|*DECIMATOR*) continue ;; esac
    node="/dev/${d##*/}"; dev=$(cat "$d/dev" 2>/dev/null)
    maj=${dev%:*}; min=${dev#*:}
    if [ ! -e "$node" ]; then
        mknod "$node" c "$maj" "$min" 2>/dev/null && CAM="$node"
    else
        CAM="$node"
    fi
    break
done
if [ -z "$CAM" ]; then
    log "!! 未找到相机设备。请确认相机已插板子 USB（别和 WiFi 卡同一下游口），拔插一下等 2 秒后重跑本脚本。"
    exit 1
fi
log "相机节点: $CAM"

PV=""
for p in /usr/bin/phone_view /root/phone_view /tmp/phone_view; do
    [ -x "$p" ] && PV="$p" && break
done
if [ -z "$PV" ]; then
    log "!! 找不到 phone_view 二进制，请先跑 install_to_board.sh 固化。"
    exit 1
fi

log "起 phone_view（-c 板载相机；IP/端口在 -c 模式下被忽略）"
killall phone_view 2>/dev/null; sleep 1
SETSID=$(command -v setsid || true)
if [ -n "$SETSID" ]; then
    setsid "$PV" -c "$CAM" 127.0.0.1 8080 -s -f >/tmp/phone.log 2>&1 </dev/null &
else
    "$PV" -c "$CAM" 127.0.0.1 8080 -s -f >/tmp/phone.log 2>&1 </dev/null &
fi
sleep 3
head -8 /tmp/phone.log

log "4) 转码守护：默认不自动运行（板载直接播 MJPEG-AVI，无需 H.264/ffmpeg，更省电不发烫）"
log "    如需导出到电脑：手动  sh /root/transcode_avi.sh /root/rec_xxx.avi  生成 mp4（保留原 avi）"

log "完成。触屏：点【录】录几秒 → 点【停】 → 点【回放】即可在板子屏上看；点【退出】干净退出。"
