#!/bin/bash
# install_to_board.sh -- 把「自包含相机系统」固化到板子持久目录（复位不丢）
# 把二进制放进 /usr/bin（板子根 fs ext4 持久），脚本放进 /root（持久），
# 复位后只需在板子串口跑一次：  sh /root/boot_cam.sh
# 用法（VM 终端）：  bash install_to_board.sh <板子IP>
set -u
IP="${1:-192.168.2.101}"
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== 检查本地产物（需先跑 build_libjpeg_turbo.sh + make + build_ffmpeg.sh）==="
PV="$DIR/phone_view"
# ffmpeg 可能落在 $DIR/ffmpeg（手动放）或 build 脚本产出的 ffmpeg_build 树里
FF="$DIR/ffmpeg"
if [ ! -x "$FF" ]; then
    FF="$(find "$DIR/ffmpeg_build" -type f -name ffmpeg -path '*/bin/*' 2>/dev/null | head -1)"
    [ -z "$FF" ] && FF="$(find "$DIR/ffmpeg_build" -type f -name ffmpeg 2>/dev/null | head -1)"
fi
ok=1
[ -x "$PV" ] || { echo "缺少可执行 $PV（先 make 出 g6o/g6n 二进制）"; ok=0; }
[ -x "$FF" ] || { echo "缺少可执行 $FF（先 bash build_ffmpeg.sh 交叉编译）"; ok=0; }
[ "$ok" = 0 ] && exit 1
echo "  找到 ffmpeg: $FF"

echo "=== scp 二进制 → 板子 /usr/bin（持久）==="
# 抗 WiFi 丢包：长超时 + 保活探测；分步传输，ffmpeg 失败也给重试命令，不整体退出
SCPOPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=30 -o ServerAliveInterval=15 -o ServerAliveCountMax=8"
scp $SCPOPTS "$PV" "root@$IP:/usr/bin/phone_view" || { echo "scp phone_view 失败"; exit 1; }
echo "  phone_view 已传"
if scp $SCPOPTS "$FF" "root@$IP:/usr/bin/ffmpeg"; then
    echo "  ffmpeg 已传"
else
    echo "  !! ffmpeg 传输失败（大文件过 WiFi 易卡）。板子功能已可用，ffmpeg 稍后单独补："
    echo "     scp $SCPOPTS $FF root@$IP:/usr/bin/ffmpeg"
fi
ssh -o StrictHostKeyChecking=no "root@$IP" "chmod +x /usr/bin/phone_view /usr/bin/ffmpeg; echo '  phone_view:'; /usr/bin/phone_view -h 2>&1 | head -1; echo '  ffmpeg:'; /usr/bin/ffmpeg -version 2>&1 | head -1"

echo "=== scp 脚本 → 板子 /root（持久）==="
# 注意：wake_ssh.sh 在 $DIR 的父目录（X6818/），其余在 $DIR（phone_view/）
for s in setup_cam.sh wake_ssh.sh transcode_avi.sh transcode_watch.sh boot_cam.sh; do
    src=""
    if [ -f "$DIR/$s" ]; then
        src="$DIR/$s"
    elif [ -f "$(dirname "$DIR")/$s" ]; then
        src="$(dirname "$DIR")/$s"
    fi
    if [ -z "$src" ]; then
        echo "  !! 找不到本地 $s（应在 $DIR 或其父目录）"
        continue
    fi
    if scp -o StrictHostKeyChecking=no "$src" "root@$IP:/root/" 2>/dev/null; then
        echo "  -> /root/$s  (来源: $src)"
    else
        echo "  !! /root/$s 传输失败"
    fi
done
ssh -o StrictHostKeyChecking=no "root@$IP" "chmod +x /root/*.sh"

echo ""
echo "=== 固化完成 ==="
echo "板子复位后，串口只需一条：  sh /root/boot_cam.sh"
echo "（它会：拉 sshd → 连 WiFi → 建相机节点 → 起 phone_view → 起本地转码守护）"
