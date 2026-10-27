#!/bin/sh
# backup_media.sh —— 把 X6818 相机 demo 生成的媒体文件打包备份（不删除原文件）
# 用法：sh backup_media.sh
# 说明：打包到 /root/media_backup_<时间戳>.tar.gz，原文件保留，方便日后拷走或清理。
set -e

TS=$(date +%Y%m%d_%H%M%S)
BACKUP=/root/media_backup_${TS}.tar.gz
TMP=/tmp/media_backup_list.txt
: > "$TMP"

# 新目录 /root/camera 下的媒体
if [ -d /root/camera ]; then
  for f in /root/camera/shot_*.jpg /root/camera/rec_*.avi /root/camera/rec_*.avi.fps; do
    if [ -e "$f" ]; then echo "$f" >> "$TMP"; fi
  done
fi

# 旧版本残留（/root 根下）
for f in /root/shot_* /root/rec_*.avi /root/rec_*.mp4; do
  if [ -e "$f" ]; then echo "$f" >> "$TMP"; fi
done

if [ ! -s "$TMP" ]; then
  echo "没有找到相机媒体文件，无需备份。"
  exit 0
fi

echo "=== 将备份以下文件 ==="
cat "$TMP"

if tar -czf "$BACKUP" -T "$TMP"; then
  echo ""
  echo "已备份到：$BACKUP"
  echo "大小：$(du -h "$BACKUP" | cut -f1)"
  echo "原文件保留未删除。需要清理时执行之前给的 rm 命令即可。"
else
  echo "打包失败，请检查 busybox tar 是否支持 -z(gzip)。"
  echo "可改用：tar -cf ${BACKUP%.gz} -T \"$TMP\"  (不压缩)"
  exit 1
fi
