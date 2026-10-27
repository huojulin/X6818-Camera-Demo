# phone_view - X6818 手机画面监看查看器
# 交叉编译：arm-linux-gcc
#
# 若链接报 "cannot find -ljpeg"：
#   说明工具链 sysroot 没有 arm 版 libjpeg。两种解法二选一：
#   1) 用静态链接（若工具链带 libjpeg.a）：
#      arm-linux-gcc -O2 -Wall -std=gnu99 -static -o phone_view phone_view.c -ljpeg
#   2) 给工具链补 arm 版 libjpeg 开发包后重编。
#   板子 rootfs 本身有 libjpeg.so（mplayer 依赖），动态链接产物可直接跑。

CC = arm-linux-gcc
CFLAGS = -O2 -Wall -std=gnu99

# libjpeg 来源（二选一，自动判断）：
#   1) 优先用 libjpeg-turbo 静态库（开 NEON，软解 MJPEG 提速 2~3x）。
#      需先在本目录跑 `bash build_libjpeg_turbo.sh` 生成 libjpeg-turbo/lib/libjpeg.a
#   2) 不存在则退回工具链动态 -ljpeg（IJG 原版，无 NEON，软解较慢）
TURBO_A := libjpeg-turbo/lib/libjpeg.a
ifeq ($(wildcard $(TURBO_A)),)
  JPEG_LIB = -ljpeg
  TURBO_DEF =
else
  JPEG_LIB = $(TURBO_A)
  TURBO_DEF = -DHAVE_TURBO_JPEG
endif

phone_view: phone_view.c
	$(CC) $(CFLAGS) $(TURBO_DEF) -o phone_view phone_view.c $(JPEG_LIB) -lpthread -lm

clean:
	rm -f phone_view

