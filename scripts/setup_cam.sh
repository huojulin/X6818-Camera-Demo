#!/bin/sh
# setup_cam.sh -- 板子端：为 USB 相机建立 /dev 节点并选出采集设备
# 背景：X6818 板子无 udev/mdev，gspca 相机热插拔绑定后不会自动建 /dev/video* 节点，
#       且 /dev/video0~9 多为 SoC MLC 显示层(非采集设备)。本脚本：
#         1) 扫描 /sys/class/video4linux，按名称排除 MLC/SCALER/CLIPPER/DECIMATOR 显示层；
#         2) 对候选设备按 /sys 里的 major:minor 手动 mknod 建节点；
#         3) 打印推荐给 phone_view 的 -c 参数。
# 用法：sh /root/setup_cam.sh
set -u

echo "=== 扫描 video4linux 设备 ==="
picked=""
for d in /sys/class/video4linux/video*; do
    [ -e "$d/name" ] || continue
    name=$(cat "$d/name" 2>/dev/null)
    dev=$(cat "$d/dev" 2>/dev/null)
    node="/dev/${d##*/}"
    # 跳过显示/缩放管线（它们名字含 MLC/SCALER/CLIPPER/DECIMATOR，且无 VIDEO_CAPTURE 能力）。
    # 注意：不要按 "VIDEO" 排除，廉价 UVC/gspca 相机节点名常含 "Video"，会误伤。
    case "$name" in
        *MLC*|*SCALER*|*CLIPPER*|*DECIMATOR*)
            echo "  跳过(显示层): $node  name='$name' ($dev)"
            continue ;;
    esac
    major=${dev%:*}; minor=${dev#*:}
    if [ ! -e "$node" ]; then
        mknod "$node" c "$major" "$minor" 2>/dev/null && \
            echo "  建节点: $node  c $major $minor  name='$name'" || \
            echo "  建节点失败: $node (major=$major minor=$minor, 需 root)"
    else
        # 节点已存在，但必须核对 minor —— V4L 的 videoN 名与真实 minor 不一定对应
        # （如本次 gspca 注册在 81:20 而非 81:9）。minor 不符则重建，否则会取到显示层。
        cur=$(stat -c '%t:%T' "$node" 2>/dev/null)
        cur_maj=$(printf '%d' "0x${cur%:*}" 2>/dev/null)
        cur_min=$(printf '%d' "0x${cur#*:}" 2>/dev/null)
        if [ "$cur_maj" = "$major" ] && [ "$cur_min" = "$minor" ]; then
            echo "  已存在且正确: $node  c $major $minor  name='$name'"
        else
            echo "  已存在但 minor 不符(cur=$cur_maj:$cur_min 应为 $major:$minor), 重建"
            rm -f "$node"
            mknod "$node" c "$major" "$minor" 2>/dev/null && \
                echo "    重建成功: $node c $major $minor" || \
                echo "    重建失败(需 root)"
        fi
    fi
    [ -z "$picked" ] && picked="$node"
done

echo ""
if [ -z "$picked" ]; then
    echo "[setup_cam] 没找到候选采集设备！"
    echo "  排查：① 相机是否插在板子 USB 口(别和 WiFi 卡插同一个下游口)；"
    echo "        ② 拔下相机重插，等 2 秒 dmesg 出现 'usb ... new high-speed ... gspca'；"
    echo "        ③ ls /sys/class/video4linux/ 看是否多出一个 videoN。"
else
    echo "[setup_cam] 推荐采集设备: $picked"
    echo "  运行: setsid /tmp/phone_view -c $picked 192.168.2.145 8080 -s -f"
    echo "  (若 -c 后报 '不是视频采集设备'，说明该节点是显示层，换下一个 videoN)"
fi
