#define _GNU_SOURCE
/*
 * phone_view.c - X6818 高速运动相机端：手机/板载摄像头 MJPEG 投屏 + V4L2 采集 + 录像
 *
 * 两种数据源（功能1 方案A/B 统一落地）：
 *   A) 板载/USB 摄像头：加 -c /dev/videoX 走 V4L2 采集（mmap streaming），
 *      自动选 MJPEG（压缩，复用 libjpeg 解码与零编码录像）或回退 YUYV（软件转+编码）。
 *   B) 手机 IP 摄像头：不加 -c 时，用 TCP socket 连手机（默认 192.168.2.121:8080），
 *      读 HTTP 响应头拿 multipart boundary，按 boundary 切出每帧 JPEG 解码。
 *   两种源都解码成 RGB 后，最近邻缩放到 LCD 分辨率写入 /dev/fb0（BGRA）。
 *
 * 录像：把每帧 JPEG 直接封装进 MJPEG-AVI 容器（零编码，不依赖 ffmpeg/x264），
 *       PC 上 VLC / Windows 播放器 / ffmpeg 均可直接播放（YUYV 源会先 libjpeg 编码）。
 *
 * 编译：arm-linux-gcc -O2 -Wall -std=gnu99 -o phone_view phone_view.c -ljpeg
 * 运行：
 *   手机投屏：    ./phone_view [手机IP] [端口] [流路径] [-o 录像] [-s 防抖] [-f 触摸跟踪] [-p 拍照目录]
 *   板载/USB相机：./phone_view -c /dev/videoX [上述开关]      （功能1 方案A 走 V4L2）
 *       例：
 *            ./phone_view 192.168.2.121 8080                 （手机：只看不录不防抖不检测）
 *            ./phone_view -c /dev/video9 -s -f -o /root/camera/c.avi -p /root/camera
 *                                                     （相机：防抖+框选+录像+拍照）
 *   回放(离线演示)：./phone_view -b <图片目录> [-s 防抖] [-f 触摸跟踪]
 *       例： ./phone_view -b /mnt/sd/frames -s -f            （目录里 BMP/JPG 当帧循环播放，跑同款显示+叠加层）
 *            相机暂不可用（如 USB 高速握手失败掉全速、video 设备注册不出）时，用已采到的帧离线演示效果。
 *       运行中： kill -USR1 $(pidof phone_view)  切换 开始/停止 录像
 *                kill -USR2 $(pidof phone_view)  抓拍一张 JPEG 到 -p 指定目录（默认 /root/camera/shot_xxxx.jpg）
 *       拍照（-p <目录>）：启用拍照，运行中随时发 SIGUSR2 存一张当前帧（MJPEG 源零损耗、
 *               零二次编码；YUYV 源用 libjpeg 编码），文件可被 PC/VLC/Windows 照片查看器直接打开。
 *       LCD 底部四触摸按钮（板载自包含，不回传 VM）：
 *               【拍照】存 /root/camera/shot_*.jpg、【录·停】切换录像（同 SIGUSR1）、
 *               【回放】板载回看 /root/camera 下最新媒体（照片直接显示 / 录像 mplayer 播）、【退出】干净退出进程。
 *       按钮区外的拖拽仍是触摸框选跟踪(-f)。
 *       默认 192.168.2.121 8080 /video；不写路径时自动尝试 /video /videofeed /mjpegfeed
 *       防抖（-s）：块匹配估计帧间平移，中心裁窗+反向补偿，抵消手持抖动（满屏无黑边）
 *
 * 依赖：板子 libjpeg（mplayer 已依赖它，rootfs 自带）。交叉编译需工具链能找到
 *       arm 版 libjpeg（头 + 库）。若链接报 cannot find -ljpeg，见 Makefile 注释。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>   /* TCP_NODELAY：低延迟推流必需 */
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>       /* mkdir：启动时自动创建媒体目录 */
#include <linux/fb.h>
#include <linux/videodev2.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <dirent.h>
#include <strings.h>
#include <pthread.h>
#include <linux/input.h>
#include "glyphs.h"   /* 32x32 中文点阵（拍/照/录/像/上/传/停 等），已随本包附带于同目录 */

#define PV_VERSION "2026-10-23g6v"

/* 解码目标上限：把 MJPEG 软解输出限制在 DECODE_TARGET_W×H 以内（libjpeg 解码期降采样，
   省 CPU）。板子裸板无散热、CPU 弱，软解是单帧最大开销。默认值已放宽到 LCD 原生
   1024×600（不再人为压到 640×360，那档上屏要拉伸、糊）。实际解码尺寸 = min(源分辨率,
   此上限)；源分辨率由 VM 端 cam2http 的 -W -H 决定。仍嫌卡就把 cam2http 的 -W -H 调小
   或 -r 调低（如 -W 800 -H 600 -r 12），卡顿主要来自软解像素量+发热降频，与帧率无关的是
   单帧像素数。 */
#define DECODE_TARGET_W 1024
#define DECODE_TARGET_H 600

static volatile int running = 1;
static void on_sig(int s) { (void)s; running = 0; }
/* SIGUSR1：请求切换录像（在主循环里处理，避免在信号处理器里做不安全操作） */
static volatile int rec_toggle_req = 0;
static void on_usr1(int s) { (void)s; rec_toggle_req = 1; }

/* SIGUSR2：请求抓拍一帧（拍照），主循环里处理 */
static volatile int snap_req = 0;
/* 板载自包含：拍照/录像产物统一存到媒体目录 /root/camera，不回传 VM（上传逻辑已整体移除）。 */
static void on_usr2(int s) { (void)s; snap_req = 1; }
static volatile int playback_req = 0;     /* 触屏「回放」→ 板载回看最新媒体（照片或录像） */
static volatile int exit_req = 0;         /* 触屏「退出」→ 干净退出 */
static int g_playback = 0;                /* 回放进行中（防止重入） */
static char snap_dir[256] = "/root/camera"; /* 媒体目录：拍照/录像统一存放（命令行 -p 可改） */
static int snap_mode = 1;                /* 是否启用拍照（按钮/ SIGUSR2 触发；默认开，始终缓存最新帧） */
static unsigned char *g_snap_buf = NULL; /* 缓存最新一帧原始 JPEG，供抓拍写盘 */
static size_t g_snap_cap = 0, g_snap_len = 0;
static long snap_count = 0;              /* 已拍张数，用于文件名序号 */

/* 板载自包含：拍/录产物统一存到媒体目录 /root/camera，不再回传 VM（回传逻辑已整体移除，见 do_snapshot/rec_stop） */

/* 抓拍：把当前缓存的最新一帧原始 JPEG 直接写盘（MJPEG 帧本就是标准 JPEG，零编码）。
   文件名 shot_0000.jpg 递增；失败仅打日志不退出。 */
static void do_snapshot(void)
{
    if (!snap_mode || g_snap_len == 0) return;
    char fn[384];
    snprintf(fn, sizeof fn, "%s/shot_%04ld.jpg", snap_dir, snap_count);
    FILE *f = fopen(fn, "wb");
    if (!f) { fprintf(stderr, "[snap] 打开失败 %s: %s\n", fn, strerror(errno)); return; }
    size_t wr = fwrite(g_snap_buf, 1, g_snap_len, f);
    fclose(f);
    snap_count++;
    fprintf(stderr, "[snap] 已保存 %s (%u 字节)\n", fn, (unsigned)wr);
    /* 板载自包含：拍照只存本地，不回传 VM */
}

/* ---------------- 全局缓冲（网络接收复用，避免反复 malloc） ---------------- */
static unsigned char *g_buf = NULL;
static size_t g_cap = 0, g_len = 0, g_search = 0;
static int sock = -1;

/* ---------------- 帧缓冲 ---------------- */
static int fb_fd = -1;
static struct fb_var_screeninfo vinfo;
static struct fb_fix_screeninfo finfo;
static unsigned char *fb_real = NULL;   /* 真实 fb0 mmap（仅此缓冲对 LCD 可见） */
static unsigned char *fb_mem = NULL;    /* 离屏绘制缓冲：所有 draw_* 写这里，fb_flush() 每帧一次性拷到 fb_real（双缓冲，消除按钮闪动） */
static long fb_screensize = 0;
static int fb_w, fb_h, fb_bpp, fb_line;

static int fb_init(void)
{
    fb_fd = open("/dev/fb0", O_RDWR);
    if (fb_fd < 0) { perror("open /dev/fb0"); return -1; }
    if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo) < 0) { perror("ioctl var"); return -1; }
    if (ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo) < 0) { perror("ioctl fix"); return -1; }
    fb_w = vinfo.xres; fb_h = vinfo.yres;
    fb_bpp = vinfo.bits_per_pixel; fb_line = finfo.line_length;
    long map_h = vinfo.yres_virtual > 0 ? vinfo.yres_virtual : fb_h;
    long screensize = (long)fb_line * map_h;
    if (finfo.smem_len > 0 && (long)finfo.smem_len > screensize) screensize = finfo.smem_len;
    fb_screensize = screensize;
    fb_real = mmap(NULL, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
    if (fb_real == MAP_FAILED) { perror("mmap fb"); return -1; }
    fb_mem = (unsigned char *)malloc(screensize);
    if (!fb_mem) { perror("malloc fb back"); munmap(fb_real, screensize); return -1; }
    memcpy(fb_mem, fb_real, screensize);   /* 初始同步：避免开场花屏 */
    fprintf(stderr, "[fb] %dx%d %dbpp line=%d (双缓冲)\n", fb_w, fb_h, fb_bpp, fb_line);
    return 0;
}

/* 双缓冲提交：把离屏缓冲一次性拷到真实 fb0。
   关键：LCD 永远不会看到“画完视频帧、但按钮还没补画”的中间态 —— 这正是按钮闪动的根因。 */
static void fb_flush(void) {
    if (fb_real && fb_mem) memcpy(fb_real, fb_mem, fb_screensize);
}

/* 缩放索引表：只算一次（源宽度不变就不重算），避免每帧 malloc/free + 重算 1024 次除法 */
static int g_sx[4096];
static int g_sx_src_w = -1;
/* 防抖绘制(裁窗)专用的 x 索引表：裁窗宽度 sw 与全幅不同，单独建表，免每帧 malloc */
static int g_sx_stab[4096];
static int g_sx_stab_w = -1;

static void sx_build(int sw)
{
    if (g_sx_src_w == sw) return;
    for (int x = 0; x < fb_w && x < 4096; x++) g_sx[x] = (x * sw) / fb_w;
    g_sx_src_w = sw;
}

/* 把 src(RGB,3 字节/像素, sw*sh) 最近邻缩放到全屏帧缓冲(BGRA)
   性能：把 bpp 分支提到循环外（每像素判断一次分支在 61 万像素上是实打实的开销），
        并复用预计算的 x 索引表。 */
static void draw_rgb_scaled(const unsigned char *src, int sw, int sh)
{
    sx_build(sw);
    if (fb_bpp >= 32) {
        for (int y = 0; y < fb_h; y++) {
            int sy = (y * sh) / fb_h;
            const unsigned char *row = src + (long)sy * sw * 3;
            unsigned char *dst = fb_mem + (long)y * fb_line;
            for (int x = 0; x < fb_w; x++) {
                const unsigned char *p = row + g_sx[x] * 3;
                /* 一次 32 位整字写替代 4 次字节写。帧缓冲 BGRA 小端（内存字节序 B,G,R,A），
                   src 为 RGB(p[0]=R,p[1]=G,p[2]=B)：最低字节=B(p[2])、次字节=G、再次=R、
                   最高字节=Alpha(0xFF)。 */
                *(uint32_t *)dst = (0xFFu << 24) | ((uint32_t)p[0] << 16) |
                                   ((uint32_t)p[1] << 8) | (uint32_t)p[2];
                dst += 4;
            }
        }
    } else if (fb_bpp == 16) {
        for (int y = 0; y < fb_h; y++) {
            int sy = (y * sh) / fb_h;
            const unsigned char *row = src + (long)sy * sw * 3;
            unsigned char *dst = fb_mem + (long)y * fb_line;
            for (int x = 0; x < fb_w; x++) {
                const unsigned char *p = row + g_sx[x] * 3;
                *(unsigned short *)dst = (unsigned short)(((p[0] >> 3) << 11) |
                    ((p[1] >> 2) << 5) | (p[2] >> 3));
                dst += 2;
            }
        }
    } else {
        for (int y = 0; y < fb_h; y++) {
            int sy = (y * sh) / fb_h;
            const unsigned char *row = src + (long)sy * sw * 3;
            unsigned char *dst = fb_mem + (long)y * fb_line;
            for (int x = 0; x < fb_w; x++) {
                const unsigned char *p = row + g_sx[x] * 3;
                *dst++ = (unsigned char)((p[0] + p[1] + p[2]) / 3);
            }
        }
    }
}

/* ================= 电子防抖（功能5：画面稳定/防抖） =================
 * 思路：相邻帧之间用手工块匹配（SAD）估计相机的平移运动，
 *       在绘制前把源图按“估计位移的反向”平移补偿，抵消手持抖动。
 *       为避免平移后露出黑边，采用“中心裁窗”——从原图裁掉一圈 margin（活动余量），
 *       窗口随补偿量平移，再缩放铺满 LCD，画面始终满屏。
 * 默认关闭，命令行加 -s 开启。 */

#define STAB_STEP   8      /* 灰度下采样步长（性能 vs 精度）：8 比 4 计算量约 1/4，粗防抖够用 */
#define STAB_BLOCK  16     /* 匹配块大小（低分辨率） */
#define STAB_RANGE  14     /* 搜索半径（低分辨率），×STEP 即原分辨率可补偿范围 */

static int stab_enable = 0;
static unsigned char *stab_prev = NULL;   /* 上一帧低分辨率灰度图 */
static int stab_pw = 0, stab_ph = 0;
static int stab_sx = 0, stab_sy = 0;     /* 平滑后的补偿量（原分辨率尺度） */

/* 把 RGB 按 STAB_STEP 步长下采样为灰度图（gw=w/STEP, gh=h/STEP） */
static void rgb_to_gray_down(const unsigned char *rgb, int w, int h, unsigned char *g)
{
    int gw = w / STAB_STEP, gh = h / STAB_STEP;
    for (int y = 0; y < gh; y++) {
        const unsigned char *srow = rgb + (size_t)(y * STAB_STEP) * w * 3;
        unsigned char *grow = g + (size_t)y * gw;
        for (int x = 0; x < gw; x++) {
            const unsigned char *p = srow + x * STAB_STEP * 3;
            grow[x] = (unsigned char)((p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8);
        }
    }
}

/* SAD：以 prev 的 (ptx,pty) 块为模板，与 cur 的 (qx,qy) 块比较（块大小 b） */
static int sad_block(const unsigned char *prev, int ptx, int pty, int pw,
                     const unsigned char *cur, int qx, int qy, int gw, int b)
{
    int sad = 0;
    for (int yy = 0; yy < b; yy++) {
        const unsigned char *pp = prev + ((size_t)(pty + yy) * pw + ptx);
        const unsigned char *qq = cur + ((size_t)(qy + yy) * gw + qx);
        for (int xx = 0; xx < b; xx++) {
            int diff = (int)pp[xx] - (int)qq[xx];
            sad += diff < 0 ? -diff : diff;
        }
    }
    return sad;
}

/* 灰度区域重采样（最近邻）：把 src 中以 (sx0,sy0) 为原点、rw×rh 的区域，
 * 缩放到 dw×dh 写入 dst。stride 为 src 整行字节数（=整图宽）。
 * 用于跟踪的多尺度模板生成与尺度归一更新。 */
static void gray_resize(const unsigned char *src, int stride,
                        int sx0, int sy0, int rw, int rh,
                        unsigned char *dst, int dw, int dh)
{
    for (int y = 0; y < dh; y++) {
        int sy = sy0 + (y * rh) / dh; if (sy < 0) sy = 0; if (sy >= rh + sy0) sy = rh + sy0 - 1;
        const unsigned char *srow = src + (size_t)sy * stride;
        for (int x = 0; x < dw; x++) {
            int sx = sx0 + (x * rw) / dw; if (sx < 0) sx = 0; if (sx >= rw + sx0) sx = rw + sx0 - 1;
            dst[y * dw + x] = srow[sx];
        }
    }
}

/* 块匹配估计相邻帧内容位移，输出 *dx,*dy（原分辨率尺度）。
 * 以“上一帧中心块”为模板，在当前帧邻域搜索 SAD 最小的位置。
 * 返回的位移 = 内容从上一帧到当前帧的平移量（低分辨率偏移 × STEP）。
 * 置信度门限：匹配必须明显优于“静止假设(0 偏移)”，否则判定为低纹理/噪声，
 * 保持上一帧补偿，避免裁窗在极端位置间跳变 → 防抖“两个取景闪烁”。 */
static void estimate_motion(const unsigned char *rgb, int w, int h, int *dx, int *dy)
{
    int gw = w / STAB_STEP, gh = h / STAB_STEP;
    unsigned char *cur = malloc((size_t)gw * gh);
    *dx = 0; *dy = 0;
    if (!cur) return;
    rgb_to_gray_down(rgb, w, h, cur);
    if (!stab_prev) {                 /* 首帧：仅记录，无估计 */
        stab_prev = cur; stab_pw = gw; stab_ph = gh;
        return;
    }
    /* 块大小/搜索半径按帧宽自适应：640×480(gw≈80)用 STAB_BLOCK/RANGE；
       320×240(gw≈40)等比缩小，否则块占比过大、搜索越界导致 EIS 失效。 */
    int b = STAB_BLOCK, r = STAB_RANGE;
    if (gw <= 50)      { b = 8;  r = 6; }
    else if (gw <= 70) { b = 12; r = 10; }
    int cx = gw / 2, cy = gh / 2;
    int tx0 = cx - b / 2; if (tx0 < 0) tx0 = 0;
    int ty0 = cy - b / 2; if (ty0 < 0) ty0 = 0;
    int best_sad = 0x7fffffff, bqx = tx0, bqy = ty0;
    int null_sad = sad_block(stab_prev, tx0, ty0, stab_pw,
                             cur, cx, cy, gw, b);   /* 静止假设(0 偏移) */
    for (int sy = -r; sy <= r; sy++) {
        int qy0 = cy + sy;
        if (qy0 < 0 || qy0 + b > gh) continue;
        for (int sx = -r; sx <= r; sx++) {
            int qx0 = cx + sx;
            if (qx0 < 0 || qx0 + b > gw) continue;
            int sad = sad_block(stab_prev, tx0, ty0, stab_pw, cur, qx0, qy0, gw, b);
            if (sad < best_sad) { best_sad = sad; bqx = qx0; bqy = qy0; }
        }
    }
    free(stab_prev);
    stab_prev = cur; stab_pw = gw; stab_ph = gh;
    /* 置信度门限：必须明显优于静止假设，否则放弃本次估计（保持上一帧补偿）。
       这能根治低纹理场景下块匹配在 ±RANGE 间乱跳、导致裁窗忽左忽右闪烁的问题。 */
    if (null_sad > 0 && best_sad > null_sad * 9 / 10) {
        *dx = 0; *dy = 0;
        return;
    }
    /* 位移 = 匹配块左上角 - 模板块左上角（即内容在图像中的平移量） */
    *dx = (bqx - tx0) * STAB_STEP;
    *dy = (bqy - ty0) * STAB_STEP;
}

/* 防抖绘制：从源图中心裁 (w-2*margin)×(h-2*margin) 窗口，窗口随补偿量 (offx,offy)
 * 平移（off 已在 [-margin,margin]），再最近邻缩放铺满 LCD。满屏无黑边。 */
static void draw_rgb_stab(const unsigned char *src, int w, int h, int offx, int offy)
{
    int margin = (w > h ? w : h) / 10;          /* 预留 10% 活动余量 */
    int sw = w - 2 * margin, sh = h - 2 * margin;
    if (sw <= 0 || sh <= 0) { draw_rgb_scaled(src, w, h); return; }
    int x0 = margin + offx, y0 = margin + offy;
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x0 + sw > w) x0 = w - sw;
    if (y0 + sh > h) y0 = h - sh;
    int bpp = fb_bpp / 8;
    /* 用静态表替代每帧 malloc(16KB)：按裁窗宽 sw 建表，宽不变即复用 */
    if (g_sx_stab_w != sw) {
        for (int x = 0; x < fb_w && x < 4096; x++) g_sx_stab[x] = (x * sw) / fb_w;
        g_sx_stab_w = sw;
    }
    for (int y = 0; y < fb_h; y++) {
        int sy = (y * sh) / fb_h;
        const unsigned char *row = src + ((size_t)(y0 + sy) * w + x0) * 3;
        unsigned char *dst = fb_mem + (size_t)y * fb_line;
        if (bpp >= 4) {
            for (int x = 0; x < fb_w; x++) {
                const unsigned char *p = row + g_sx_stab[x] * 3;
                *(uint32_t *)dst = (0xFFu << 24) | ((uint32_t)p[0] << 16) |
                                   ((uint32_t)p[1] << 8) | (uint32_t)p[2];
                dst += 4;
            }
        } else if (bpp == 2) {
            for (int x = 0; x < fb_w; x++) {
                const unsigned char *p = row + g_sx_stab[x] * 3;
                *(unsigned short *)dst = (unsigned short)(((p[0] >> 3) << 11) |
                    ((p[1] >> 2) << 5) | (p[2] >> 3));
                dst += 2;
            }
        } else {
            for (int x = 0; x < fb_w; x++) {
                const unsigned char *p = row + g_sx_stab[x] * 3;
                *dst++ = (unsigned char)((p[0] + p[1] + p[2]) / 3);
            }
        }
    }
}

/* ================= 触摸框选 + 对象跟踪（替代旧“自动人脸框选”） =================
 * 交互：在 LCD 上用手指/触控笔“拖拽”画一个框选中目标（按下→拖动→抬起）；
 *       程序把该框映射到源帧坐标、提取模板，之后每一帧在源帧里做模板匹配
 *       （块匹配 SAD，复用防抖那套思路）找到目标新位置并画绿框跟随。
 *       若只是“点一下”（按下抬起位移很小），则以该点为中心自动生成默认框。
 * 依赖：触摸屏 /dev/input/event1（gslX680，ABS_X/ABS_Y 或 ABS_MT_* + BTN_TOUCH）。
 *       坐标经 EVIOCGABS 读到的量程映射到 LCD(1024x600)，再用 lcd_to_src 反算
 *       回源帧坐标，因此无论是否开防抖(-s)都能正确对齐。
 * 命令行加 -f 开启（旧版 -f=自动人脸框，现改为触摸框选跟踪）。 */

static int track_enable = 0;

/* 在 RGB 缓冲(rgb, w*h*3)上画空心矩形 (x0,y0)-(x1,y1)，颜色 (r,g,b)，边宽 t */
static void draw_rect(unsigned char *rgb, int w, int h, int x0, int y0, int x1, int y1,
                      unsigned char r, unsigned char g, unsigned char b, int t)
{
    if (x1 < x0) { int tmp = x0; x0 = x1; x1 = tmp; }
    if (y1 < y0) { int tmp = y0; y0 = y1; y1 = tmp; }
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1; if (y1 >= h) y1 = h - 1;
    if (x1 <= x0 || y1 <= y0) return;
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            int edge = (y - y0 <= t) || (y1 - y <= t) ||
                       (x - x0 <= t) || (x1 - x <= t);
            if (edge) {
                unsigned char *p = rgb + ((long)y * w + x) * 3;
                p[0] = r; p[1] = g; p[2] = b;
            }
        }
    }
}

/* 直接在 LCD 帧缓冲坐标画空心矩形（用于触摸即时反馈，不受源→屏映射方向影响）。
   像素格式 BGRA：dst 字节序 B,G,R,A。拖动时画黄框实时显示手指轨迹，
   这样“触摸是否被捕获”一眼可见，也能反推坐标要不要翻轴（TS_FLIP 系列宏与 TS_SWAP）。 */
static void draw_fb_rect(int lx0, int ly0, int lx1, int ly1,
                         unsigned char r, unsigned char g, unsigned char b, int t)
{
    if (lx1 < lx0) { int tmp = lx0; lx0 = lx1; lx1 = tmp; }
    if (ly1 < ly0) { int tmp = ly0; ly0 = ly1; ly1 = tmp; }
    if (lx0 < 0) lx0 = 0; if (ly0 < 0) ly0 = 0;
    if (lx1 >= fb_w) lx1 = fb_w - 1; if (ly1 >= fb_h) ly1 = fb_h - 1;
    if (lx1 <= lx0 || ly1 <= ly0) return;
    for (int y = ly0; y <= ly1; y++) {
        if (y < 0 || y >= fb_h) continue;
        unsigned char *dst = fb_mem + (size_t)y * fb_line;
        for (int x = lx0; x <= lx1; x++) {
            int edge = (y - ly0 <= t) || (ly1 - y <= t) ||
                       (x - lx0 <= t) || (lx1 - x <= t);
            if (edge) {
                unsigned char *p = dst + (size_t)x * 4;   /* BGRA */
                p[0] = b; p[1] = g; p[2] = r;
            }
        }
    }
}

/* ---- 中文点阵绘制（glyphs.h 32x32 1bpp） ---- */
static int u8len(const char *s) {
    int n = 0; while (s && *s) { unsigned char c = (unsigned char)*s; if ((c & 0xC0) != 0x80) n++; s++; } return n;
}
static void draw_cjk(unsigned int cp, int x, int y, int r, int g, int b) {
    int idx = -1;
    for (int i = 0; i < CJK_N; i++) if (cjk_cp[i] == cp) { idx = i; break; }
    if (idx < 0) return;
    const unsigned char *bm = cjk_bm[idx];
    for (int row = 0; row < CJK_SZ; row++) {
        int by = y + row; if (by < 0 || by >= fb_h) continue;
        unsigned char *dst = fb_mem + (size_t)by * fb_line;
        for (int col = 0; col < CJK_SZ; col++) {
            if (bm[row*4 + (col>>3)] & (1 << (7 - (col & 7)))) {
                int bx = x + col; if (bx < 0 || bx >= fb_w) continue;
                unsigned char *p = dst + (size_t)bx * 4; p[0] = b; p[1] = g; p[2] = r;
            }
        }
    }
}
static void draw_cjk_str(const char *s, int x, int y, int r, int g, int b) {
    int i = 0;
    while (s && s[i]) {
        unsigned int cp; int n; unsigned char c = (unsigned char)s[i];
        if (c < 0x80) { cp = c; n = 1; }
        else if ((c & 0xE0) == 0xC0) { cp = c & 0x1F; n = 2; }
        else if ((c & 0xF0) == 0xE0) { cp = c & 0x0F; n = 3; }
        else if ((c & 0xF8) == 0xF0) { cp = c & 0x07; n = 4; }
        else { i++; continue; }
        for (int k = 1; k < n; k++) { if (!s[i+k]) break; cp = (cp << 6) | ((unsigned char)s[i+k] & 0x3F); }
        i += n;
        draw_cjk(cp, x, y, r, g, b);
        x += CJK_SZ;
    }
}

/* ---- 底部四个触摸按钮：拍照 / 录·停 / 回放 / 退出 ----
   全部板载自包含，不再有「上传」按钮（拍/录只留本地，回放用板载 mplayer）。 */
#define BTN_Y0 500
#define BTN_Y1 590
#define B1_X0 20
#define B1_X1 255    /* 拍照 */
#define B2_X0 269
#define B2_X1 504    /* 录·停 */
#define B3_X0 518
#define B3_X1 753    /* 回放 */
#define B4_X0 767
#define B4_X1 1002   /* 退出 */
enum { BTN_NONE = 0, BTN_SNAP = 1, BTN_REC = 2, BTN_PLAY = 3, BTN_EXIT = 4 };
static int hit_button(int x, int y) {
    if (y < BTN_Y0 || y > BTN_Y1) return BTN_NONE;
    if (x >= B1_X0 && x <= B1_X1) return BTN_SNAP;
    if (x >= B2_X0 && x <= B2_X1) return BTN_REC;
    if (x >= B3_X0 && x <= B3_X1) return BTN_PLAY;
    if (x >= B4_X0 && x <= B4_X1) return BTN_EXIT;
    return BTN_NONE;
}
static void draw_button(int x0, int y0, int x1, int y1, const char *label,
                        int fr, int fg, int fb, int tr, int tg, int tb) {
    for (int y = y0; y <= y1; y++) {
        if (y < 0 || y >= fb_h) continue;
        unsigned char *dst = fb_mem + (size_t)y * fb_line;
        for (int x = x0; x <= x1; x++) {
            if (x < 0 || x >= fb_w) continue;
            int edge = (x - x0 <= 3) || (x1 - x <= 3) || (y - y0 <= 3) || (y1 - y <= 3);
            unsigned char *p = dst + (size_t)x * 4;
            if (edge) { p[0] = tb; p[1] = tg; p[2] = tr; }
            else      { p[0] = fb; p[1] = fg; p[2] = fr; }
        }
    }
    int nch = u8len(label);
    int tw = nch * CJK_SZ;
    int tx = x0 + (x1 - x0 + 1 - tw) / 2;
    int ty = y0 + (y1 - y0 + 1 - CJK_SZ) / 2;
    draw_cjk_str(label, tx, ty, tr, tg, tb);
}

/* 触摸屏状态：由触摸屏线程写、主循环读，互斥保护（前置声明，供 draw_bottom_buttons 引用） */
typedef struct {
    int down;                       /* 当前是否按下 */
    int cur_x, cur_y;               /* 当前触点(LCD 坐标) */
    int start_x, start_y;           /* 本次按下起点(LCD) */
    int have_pt;                    /* 起点已锚定（用于抬起时结算框选） */
    int sel_ready;                  /* 一次新框选已完成，待主循环消费 */
    int btn_hit;                    /* 本次按下命中的底部按钮(0=无，见 BTN_*) */
    int sel_x0, sel_y0, sel_x1, sel_y1;  /* 新框选(LCD 坐标) */
} ts_state_t;
static ts_state_t ts;
static pthread_mutex_t ts_mtx = PTHREAD_MUTEX_INITIALIZER;

/* 录像状态（前置声明，供底部按钮绘制引用录制中/已停切换标签） */
static int recording = 0;            /* 当前是否正在录像 */

/* 底部四个按钮统一绘制（带按下反色反馈），两个主循环共用，保证相机/手机模式按钮都可见 */
static void draw_bottom_buttons(void) {
    pthread_mutex_lock(&ts_mtx);
    int ps = (ts.down && ts.have_pt && ts.btn_hit == BTN_SNAP);
    int pr = (ts.down && ts.have_pt && ts.btn_hit == BTN_REC);
    int pp = (ts.down && ts.have_pt && ts.btn_hit == BTN_PLAY);
    int pe = (ts.down && ts.have_pt && ts.btn_hit == BTN_EXIT);
    pthread_mutex_unlock(&ts_mtx);
    draw_button(B1_X0, BTN_Y0, B1_X1, BTN_Y1, "拍照",
                ps ? 150 : 24,  ps ? 200 : 60,  ps ? 255 : 120,
                ps ?  20 : 255, ps ?  30 : 255, ps ?  60 : 255);
    draw_button(B2_X0, BTN_Y0, B2_X1, BTN_Y1, recording ? "停" : "录",
                pr ? 240 : 150, pr ?  90 : 36,  pr ?  90 : 36,
                255, 255, 255);
    draw_button(B3_X0, BTN_Y0, B3_X1, BTN_Y1, "回放",
                pp ? 150 : 24,  pp ? 200 : 60,  pp ? 255 : 120,
                pp ?  20 : 255, pp ?  30 : 255, pp ?  60 : 255);
    draw_button(B4_X0, BTN_Y0, B4_X1, BTN_Y1, "退出",
                pe ? 180 : 60,  pe ?  60 : 36,  pe ?  60 : 36,
                255, 255, 255);
}

/* 板载回放（触屏【回放】按钮，见 start_board_playback）：取 /root/camera 下 mtime 最新的媒体。
   照片(shot_*.jpg)→解码铺满屏、触摸任意处返回；录像(rec_*.avi)→板载 mplayer 播。
   本板无 VPU，H.264 mp4 解不动→黑屏，故只认 MJPEG-AVI；全程不依赖 PC/VM。 */
/* 在 fb0 居中显示一行提示（状态反馈，避免“无录像时静默无响应”让人以为没实现） */
static void fb_show_msg(const char *text) {
    if (!fb_mem) return;
    for (int y = 0; y < fb_h; y++) {
        unsigned char *dst = fb_mem + (size_t)y * fb_line;
        for (int x = 0; x < fb_w; x++) {
            unsigned char *p = dst + (size_t)x * 4;
            p[0] = 12; p[1] = 12; p[2] = 32;   /* 深蓝底 */
        }
    }
    int nch = u8len(text);
    int tx = (fb_w - nch * CJK_SZ) / 2;
    int ty = (fb_h - CJK_SZ) / 2;
    draw_cjk_str(text, tx, ty, 255, 255, 255);
}
/* 解码并铺满显示一张照片，触摸任意处返回（板载自包含，不依赖 PC/VM） */
static unsigned char *decode_jpeg_file(const char *path, int *w, int *h);
static void start_board_playback(void) {
    if (g_playback) return;        /* 防重入 */
    g_playback = 1;
    fprintf(stderr, "[play] 板载回放：查找最新媒体（照片/录像）...\n");
    /* 取 /root/camera 下 mtime 最新的 shot_*.jpg 或 rec_*.avi：拍照后回看自然是最新 jpg，
       录完回看是最新 avi，一个按钮统管两类媒体，无需切模式。 */
    char media[256];
    {
        char ls_cmd[512];
        snprintf(ls_cmd, sizeof ls_cmd, "ls -t %s/shot_*.jpg %s/rec_*.avi 2>/dev/null | head -1", snap_dir, snap_dir);
        FILE *pf = popen(ls_cmd, "r");
        media[0] = 0;
        if (pf) {
            if (fgets(media, sizeof media, pf)) {
                size_t L = strlen(media);
                if (L && media[L-1] == '\n') media[L-1] = 0;
            }
            pclose(pf);
        }
    }
    if (!media[0]) {
        fprintf(stderr, "[play] 无照片/录像（请先点【拍照】或【录】）\n");
        fb_show_msg("无媒体，请先拍照或录制");
        fb_flush();          /* 离屏→真实 fb，否则提示不显示 */
        sleep(2);
        g_playback = 0;
        return;
    }
    size_t ml = strlen(media);
    int is_avi = (ml >= 4 && strcmp(media + ml - 4, ".avi") == 0);
    if (is_avi) {
        /* 录像：mplayer 播（含实测帧率旁注 + g6t 修复后的 seek 索引） */
        char fpsarg[32] = "";
        char fpsfn[400];
        snprintf(fpsfn, sizeof fpsfn, "%s.fps", media);
        FILE *ff = fopen(fpsfn, "r");
        if (ff) {
            unsigned int fr = 0;
            if (fscanf(ff, "%u", &fr) == 1 && fr > 0 && fr <= 120)
                snprintf(fpsarg, sizeof fpsarg, "-fps %u ", fr);
            fclose(ff);
        }
        char cmd[1024];
        snprintf(cmd, sizeof cmd,
            "echo \"[play] mplayer -> %s ($(stat -c%%s \"%s\" 2>/dev/null) 字节) %s\"; "
            "mplayer -vo fbdev2 -nosound %s-zoom -fs \"%s\" >/tmp/mplay.log 2>&1; "
            "echo \"[play] mplayer 退出码=$?\"",
            media, media, fpsarg, fpsarg, media);
        int r = system(cmd);
        (void)r;
    } else {
        /* 照片：解码铺满 fb0，触摸任意处返回 */
        int w = 0, h = 0;
        unsigned char *rgb = decode_jpeg_file(media, &w, &h);
        if (!rgb) {
            fprintf(stderr, "[play] 照片解码失败: %s\n", media);
            fb_show_msg("照片解码失败");
            fb_flush();
            sleep(2);
        } else {
            draw_rgb_scaled(rgb, w, h);
            free(rgb);
            /* 底部提示：触摸任意处返回（白字叠加在照片上） */
            draw_cjk_str("触摸任意处返回",
                         (fb_w - 7 * CJK_SZ) / 2, fb_h - CJK_SZ - 8, 255, 255, 255);
            fb_flush();
            fprintf(stderr, "[play] 显示照片 %s（触摸任意处返回）\n", media);
            /* 进入时 ts.down 已为 0（点击回放按钮在抬起时触发），轮询“重新按下”即退出。
               本 while 阻塞期间主循环不会重绘相机帧，照片不会被覆盖。 */
            while (g_playback) {
                pthread_mutex_lock(&ts_mtx);
                int down = ts.down;
                pthread_mutex_unlock(&ts_mtx);
                if (down) break;
                usleep(40000);
            }
            /* 防误触：照片界面触摸可能落在底部按钮区，清除可能产生的请求标志，
               避免退出回放后主循环误触发拍照/录像/再次回放/退出 */
            snap_req = 0; rec_toggle_req = 0; playback_req = 0; exit_req = 0;
        }
    }
    g_playback = 0;
    fprintf(stderr, "[play] 回放结束\n");
}

/* ================= 触摸框选 + 对象跟踪 实现 =================
 * 跟踪在“源帧坐标”里进行（与防抖/缩放无关）：触摸框选 → 映射到源坐标提取模板 →
 * 每帧在源帧做块匹配(SAD)找目标新位置 → 画绿框。模板匹配复用防抖那套灰度下采样思路。 */

/* 触摸屏坐标纠偏：不同屏贴/驱动可能 X/Y 反相或互换，按需把 0 改成 1 */
#define TS_FLIP_X 0
#define TS_FLIP_Y 0
#define TS_SWAP   0

/* 触摸坐标自动量程：部分驱动(gslX680 MT)未设 absinfo，运行时观察最大坐标自适应映射 */
static int obs_maxX = 1023, obs_maxY = 599;

/* 对象跟踪状态（低分辨率网格坐标，源/TRACK_STEP） */
#define TRACK_STEP 8                /* 灰度下采样步长：8 vs 4 计算量约 1/4 */
#define TRACK_SR   7                /* 搜索半径(网格)；STEP×SR 保持≈56原像素搜索范围，与旧版一致 */
static unsigned char *tr_tpl = NULL;   /* 模板灰度(网格) */
static int tr_tgw = 0, tr_tgh = 0;     /* 模板尺寸(网格) */
static int tr_px = 0, tr_py = 0;       /* 模板左上角(网格) */
static int tr_active = 0;              /* 跟踪是否已激活 */
static int tr_lost = 0;                /* 连续失配计数（用于提示） */
static int tr_sw = 0, tr_sh = 0;       /* 模板创建时的源帧尺寸（尺寸变化则失效） */

/* 把 LCD 点(lx,ly) 反算回源帧坐标并写入 *sx,*sy，与 draw_rgb_scaled /
   draw_rgb_stab 的正向映射严格互逆（无防抖：线性铺满；防抖：裁窗后铺满）。 */
static void lcd_to_src(int lx, int ly, int w, int h, int stab_on, int offx, int offy,
                       int *sx, int *sy)
{
    if (!stab_on) {
        *sx = (lx * w) / fb_w;
        *sy = (ly * h) / fb_h;
    } else {
        int margin = (w > h ? w : h) / 10;
        int sw = w - 2 * margin, sh = h - 2 * margin;
        if (sw <= 0 || sh <= 0) { *sx = (lx * w) / fb_w; *sy = (ly * h) / fb_h; return; }
        int x0 = margin + offx, y0 = margin + offy;
        if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
        if (x0 + sw > w) x0 = w - sw; if (y0 + sh > h) y0 = h - sh;
        *sx = x0 + (lx * sw) / fb_w;
        *sy = y0 + (ly * sh) / fb_h;
    }
    if (*sx < 0) *sx = 0; if (*sy < 0) *sy = 0;
    if (*sx > w - 1) *sx = w - 1; if (*sy > h - 1) *sy = h - 1;
}

/* 从一帧 RGB 提取矩形(源坐标)内灰度模板（网格），并激活跟踪 */
static void track_extract_template(const unsigned char *rgb, int w, int h,
                                   int bx0, int by0, int bx1, int by1)
{
    if (bx1 < bx0) { int t = bx0; bx0 = bx1; bx1 = t; }
    if (by1 < by0) { int t = by0; by0 = by1; by1 = t; }
    if (bx0 < 0) bx0 = 0; if (by0 < 0) by0 = 0;
    if (bx1 >= w) bx1 = w - 1; if (by1 >= h) by1 = h - 1;
    int tgx0 = bx0 / TRACK_STEP, tgy0 = by0 / TRACK_STEP;
    int tgx1 = bx1 / TRACK_STEP, tgy1 = by1 / TRACK_STEP;
    if (tgx1 <= tgx0) tgx1 = tgx0 + 1;
    if (tgy1 <= tgy0) tgy1 = tgy0 + 1;
    int tgw = tgx1 - tgx0 + 1, tgh = tgy1 - tgy0 + 1;
    if (tgw < 2) tgw = 2; if (tgh < 2) tgh = 2;
    unsigned char *t = (unsigned char *)malloc((size_t)tgw * tgh);
    if (!t) return;
    for (int y = 0; y < tgh; y++) {
        int sy = (tgy0 + y) * TRACK_STEP;
        const unsigned char *srow = rgb + (size_t)sy * w * 3;
        for (int x = 0; x < tgw; x++) {
            int sx = (tgx0 + x) * TRACK_STEP;
            const unsigned char *p = srow + sx * 3;
            t[y * tgw + x] = (unsigned char)((p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8);
        }
    }
    free(tr_tpl); tr_tpl = t; tr_tgw = tgw; tr_tgh = tgh;
    tr_px = tgx0; tr_py = tgy0; tr_active = 1; tr_lost = 0;
    tr_sw = w; tr_sh = h;
}

/* 一帧模板匹配：在当前帧灰度里于上一位置邻域搜索 SAD 最小处，更新框并画绿框。
   命中良好时缓慢更新模板(0.12)以跟随外观缓慢变化；失配(平均差过大)则保持上一框、
   不强行更新，避免乱飘。 */
static void track_step(const unsigned char *rgb, int w, int h)
{
    if (w != tr_sw || h != tr_sh) { tr_active = 0; return; }   /* 源尺寸变化：失效 */
    int gw = w / TRACK_STEP, gh = h / TRACK_STEP;
    unsigned char *cur = (unsigned char *)malloc((size_t)gw * gh);
    if (!cur) return;
    rgb_to_gray_down(rgb, w, h, cur);
    if (!tr_tpl || tr_tgw <= 0 || tr_tgh <= 0) { free(cur); return; }

    /* 多尺度搜索：目标远近变化时尺寸会变，单尺度易跟丢。
       在 0.8 / 1.0 / 1.2 三个尺度上各做 SAD 搜索，取全局最优；
       命中后将命中区域重采样回固定模板尺寸，使模板随尺度归一 → 尺度自适应。 */
    static const int SCALES[] = {8, 10, 12};   /* /10 */
    int ns = (int)(sizeof(SCALES) / sizeof(SCALES[0]));
    int best_sad = 0x7fffffff, bqx = tr_px, bqy = tr_py, btw = tr_tgw, bth = tr_tgh, bsi = 1, found = 0;
    unsigned char *ts = NULL;
    for (int si = 0; si < ns; si++) {
        int tw = (tr_tgw * SCALES[si] + 5) / 10;
        int th = (tr_tgh * SCALES[si] + 5) / 10;
        if (tw < 2) tw = 2; if (th < 2) th = 2;
        unsigned char *nts = (unsigned char *)realloc(ts, (size_t)tw * th);
        if (!nts) { free(ts); free(cur); return; }
        ts = nts;
        gray_resize(tr_tpl, tr_tgw, 0, 0, tr_tgw, tr_tgh, ts, tw, th);
        for (int dy = -TRACK_SR; dy <= TRACK_SR; dy++) {
            int qy0 = tr_py + dy;
            if (qy0 < 0 || qy0 + th > gh) continue;
            for (int dx = -TRACK_SR; dx <= TRACK_SR; dx++) {
                int qx0 = tr_px + dx;
                if (qx0 < 0 || qx0 + tw > gw) continue;
                int sad = 0;
                for (int yy = 0; yy < th; yy++) {
                    const unsigned char *pp = ts + (size_t)yy * tw;
                    const unsigned char *qq = cur + ((size_t)(qy0 + yy) * gw + qx0);
                    for (int xx = 0; xx < tw; xx++) {
                        int diff = (int)pp[xx] - (int)qq[xx];
                        sad += diff < 0 ? -diff : diff;
                    }
                }
                if (sad < best_sad) {
                    best_sad = sad; bqx = qx0; bqy = qy0;
                    btw = tw; bth = th; bsi = si; found = 1;
                }
            }
        }
    }
    if (found) {
        int area = btw * bth;
        int avg = best_sad / area;
        if (avg > 60) {                       /* 失配：保持上一框，提示重选 */
            tr_lost++;
            if ((tr_lost % 30) == 1)
                fprintf(stderr, "[track] 匹配度低(avg=%d,scale=%d)，可能跟丢，保持上一框；请重新触摸框选\n", avg, SCALES[bsi]);
        } else {
            tr_lost = 0;
            tr_px = bqx; tr_py = bqy;
            /* 把命中区域(当前尺度)重采样回固定模板尺寸，模板随尺度归一 */
            unsigned char *upd = (unsigned char *)malloc((size_t)tr_tgw * tr_tgh);
            if (upd) {
                gray_resize(cur, gw, bqx, bqy, btw, bth, upd, tr_tgw, tr_tgh);
                for (int y = 0; y < tr_tgh; y++)  /* 缓慢更新模板，跟随外观 */
                    for (int x = 0; x < tr_tgw; x++)
                        tr_tpl[y * tr_tgw + x] =
                            (unsigned char)(tr_tpl[y * tr_tgw + x] * 88 / 100 + upd[y * tr_tgw + x] * 12 / 100);
                free(upd);
            }
        }
        int bx0 = bqx * TRACK_STEP, by0 = bqy * TRACK_STEP;
        int bx1 = bx0 + btw * TRACK_STEP, by1 = by0 + bth * TRACK_STEP;
        draw_rect(rgb, w, h, bx0, by0, bx1, by1, 0, 255, 0, 2);
    }
    free(ts);
    free(cur);
}

/* 主循环每帧调用：消费新框选(若有)→初始化跟踪；已激活则跑一帧跟踪。 */
static void track_apply(unsigned char *rgb, int w, int h, int stab_on, int offx, int offy)
{
    pthread_mutex_lock(&ts_mtx);
    if (ts.sel_ready) {
        int lx0 = ts.sel_x0, ly0 = ts.sel_y0, lx1 = ts.sel_x1, ly1 = ts.sel_y1;
        ts.sel_ready = 0;
        pthread_mutex_unlock(&ts_mtx);
        if (lx1 < lx0) { int t = lx0; lx0 = lx1; lx1 = t; }
        if (ly1 < ly0) { int t = ly0; ly0 = ly1; ly1 = t; }
        int dx = lx1 - lx0, dy = ly1 - ly0;
        if (dx < fb_w / 20 && dy < fb_h / 20) {       /* 点一下：以该点为中心默认框 */
            int cx = (lx0 + lx1) / 2, cy = (ly0 + ly1) / 2;
            int half = (w > h ? w : h) / 6;
            lx0 = cx - half; ly0 = cy - half; lx1 = cx + half; ly1 = cy + half;
        }
        int s0x, s0y, s1x, s1y;
        lcd_to_src(lx0, ly0, w, h, stab_on, offx, offy, &s0x, &s0y);
        lcd_to_src(lx1, ly1, w, h, stab_on, offx, offy, &s1x, &s1y);
        track_extract_template(rgb, w, h, s0x, s0y, s1x, s1y);
        fprintf(stderr, "[track] 已框选并启动跟踪 @源(%d,%d)-(%d,%d)\n", s0x, s0y, s1x, s1y);
    } else {
        pthread_mutex_unlock(&ts_mtx);
    }
    if (tr_active) track_step(rgb, w, h);
}

/* ================= 触摸屏读取线程（gslX680 / 通用 input 事件） =================
 * 阻塞读 /dev/input/event1，把触点映射成 LCD 坐标，按下→抬起结算一次框选。
 * 兼容：ABS_X/ABS_Y + BTN_TOUCH（单点），或 ABS_MT_POSITION_X/Y + ABS_MT_TRACKING_ID（多点协议B）。 */
static volatile int ts_stop = 0;

static int ts_map_x(int v, int lo, int hi) {
    if (hi <= lo) return 0;
    int x = (v - lo) * fb_w / (hi - lo + 1);
    return x < 0 ? 0 : (x >= fb_w ? fb_w - 1 : x);
}
static int ts_map_y(int v, int lo, int hi) {
    if (hi <= lo) return 0;
    int y = (v - lo) * fb_h / (hi - lo + 1);
    return y < 0 ? 0 : (y >= fb_h ? fb_h - 1 : y);
}

/* 统一的“按下/抬起”处理：三种信号(BTN_TOUCH / MT_TRACKING_ID / ABS_PRESSURE)都归一到这里。
   按下：复位起点（等首个 ABS 锚定，支持“重新触摸”无需干净抬起）；
   抬起且有锚定起点：提交一次框选(LCD 坐标)，供主循环消费。 */
static void ts_set_down(int nd)
{
    pthread_mutex_lock(&ts_mtx);
    if (nd && !ts.down) {
        ts.down = 1; ts.have_pt = 0;          /* 新一次触摸：起点等首个 ABS 锚定 */
    } else if (!nd && ts.down) {
        if (ts.have_pt) {
            if (ts.btn_hit == BTN_SNAP) {
                snap_req = 1;
                fprintf(stderr, "[btn] 拍照\n");
            } else if (ts.btn_hit == BTN_REC) {
                rec_toggle_req = 1;
                fprintf(stderr, "[btn] 录像切换\n");
            } else if (ts.btn_hit == BTN_PLAY) {
                playback_req = 1;
                fprintf(stderr, "[btn] 回放\n");
            } else if (ts.btn_hit == BTN_EXIT) {
                exit_req = 1;
                fprintf(stderr, "[btn] 退出\n");
            } else {
                ts.sel_x0 = ts.start_x; ts.sel_y0 = ts.start_y;
                ts.sel_x1 = ts.cur_x;  ts.sel_y1 = ts.cur_y;
                ts.sel_ready = 1; ts.have_pt = 0;
                fprintf(stderr, "[touch] up   框选 LCD(%d,%d)-(%d,%d)\n",
                        ts.sel_x0, ts.sel_y0, ts.sel_x1, ts.sel_y1);
            }
        }
        ts.down = 0; ts.have_pt = 0; ts.btn_hit = BTN_NONE;
    }
    pthread_mutex_unlock(&ts_mtx);
}

static void *touch_thread(void *arg)
{
    (void)arg;
    const char *dev = "/dev/input/event1";
    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "[touch] 打开 %s 失败: %s（触摸屏不可用，跟踪需手动框选将无效；显示不受影响）\n",
                dev, strerror(errno));
        return NULL;
    }
    struct input_absinfo info;
    int ax = ABS_X, ay = ABS_Y, minX = 0, maxX = 0, minY = 0, maxY = 0;
    if (ioctl(fd, EVIOCGABS(ABS_MT_POSITION_X), &info) == 0 && info.maximum > info.minimum) {
        ax = ABS_MT_POSITION_X; minX = info.minimum; maxX = info.maximum;
    } else if (ioctl(fd, EVIOCGABS(ABS_X), &info) == 0 && info.maximum > info.minimum) {
        minX = info.minimum; maxX = info.maximum;
    }
    if (ioctl(fd, EVIOCGABS(ABS_MT_POSITION_Y), &info) == 0 && info.maximum > info.minimum) {
        ay = ABS_MT_POSITION_Y; minY = info.minimum; maxY = info.maximum;
    } else if (ioctl(fd, EVIOCGABS(ABS_Y), &info) == 0 && info.maximum > info.minimum) {
        minY = info.minimum; maxY = info.maximum;
    }
    int mt = (ax == ABS_MT_POSITION_X);
    int range_valid = (maxX > minX);
    obs_maxX = 1023; obs_maxY = 599;   /* 自动量程基线(gslX680 典型 1024x600，遇更大值自增) */
    fprintf(stderr, "[touch] %s 量程 X[%d,%d] Y[%d,%d] 模式=%s 量程来源=%s\n",
            dev, minX, maxX, minY, maxY, mt ? "多点(MT)" : "单点",
            range_valid ? "absinfo" : "自动(观察最大)");

    struct input_event ev;
    while (!ts_stop) {
        ssize_t n = read(fd, &ev, sizeof ev);
        if (n < (ssize_t)sizeof ev) {
            if (n == 0) break;
            if (errno == EINTR) continue;
            break;
        }
        if (ev.type == EV_KEY && ev.code == BTN_TOUCH) {
            ts_set_down(ev.value != 0);
            continue;
        }
        if (ev.type == EV_ABS) {
            if (mt && ev.code == ABS_MT_TRACKING_ID) {
                ts_set_down(ev.value >= 0);
                continue;
            }
            if (ev.code == ABS_PRESSURE) {           /* 部分单点屏只用压力表示按下 */
                ts_set_down(ev.value > 0);
                continue;
            }
            if (ev.code != ax && ev.code != ay) continue;
            int nx = ts.cur_x, ny = ts.cur_y;
            if (range_valid) {
                if (ev.code == ax) nx = ts_map_x(ev.value, minX, maxX);
                else               ny = ts_map_y(ev.value, minY, maxY);
            } else {
                /* 量程无效(如 gslX680 MT 驱动未设 absinfo)：按运行时观察到的最大坐标自适应映射 */
                if (ev.code == ax) {
                    if (ev.value > obs_maxX) obs_maxX = ev.value;
                    nx = (ev.value * (fb_w - 1)) / obs_maxX;
                } else {
                    if (ev.value > obs_maxY) obs_maxY = ev.value;
                    ny = (ev.value * (fb_h - 1)) / obs_maxY;
                }
                if (nx < 0) nx = 0; if (nx >= fb_w) nx = fb_w - 1;
                if (ny < 0) ny = 0; if (ny >= fb_h) ny = fb_h - 1;
            }
            if (TS_SWAP) { int t = nx; nx = ny; ny = t; }
            if (TS_FLIP_X) nx = fb_w - 1 - nx;
            if (TS_FLIP_Y) ny = fb_h - 1 - ny;
            pthread_mutex_lock(&ts_mtx);
            ts.cur_x = nx; ts.cur_y = ny;
            if (ts.down && !ts.have_pt) {
                ts.start_x = nx; ts.start_y = ny; ts.have_pt = 1;
                ts.btn_hit = hit_button(nx, ny);
                fprintf(stderr, "[touch] down @(%d,%d) btn=%d\n", nx, ny, ts.btn_hit);
            }
            pthread_mutex_unlock(&ts_mtx);
            continue;
        }
        /* 其它事件（SYN 等）忽略 */
    }
    close(fd);
    return NULL;
}

/* ---------------- libjpeg 内存源（兼容老版本，不依赖 jpeg_mem_src） ---------------- */
typedef struct {
    struct jpeg_source_mgr pub;
    const JOCTET *data;
    size_t len;
} mem_src_mgr;

static void mem_init(j_decompress_ptr c) { (void)c; }
static boolean mem_fill(j_decompress_ptr c)
{
    mem_src_mgr *s = (mem_src_mgr *)c->src;
    s->pub.next_input_byte = s->data;
    s->pub.bytes_in_buffer = s->len;
    return TRUE;
}
static void mem_skip(j_decompress_ptr c, long n)
{
    mem_src_mgr *s = (mem_src_mgr *)c->src;
    if (n > 0) { s->pub.next_input_byte += n; s->pub.bytes_in_buffer -= n; }
}
static void mem_term(j_decompress_ptr c) { (void)c; }

static void jpeg_mem_src_custom(j_decompress_ptr c, const JOCTET *data, size_t len)
{
    mem_src_mgr *s;
    if (c->src == NULL) {
        c->src = (struct jpeg_source_mgr *)(*c->mem->alloc_small)(
            (void *)c, JPOOL_PERMANENT, sizeof(mem_src_mgr));
    }
    s = (mem_src_mgr *)c->src;
    s->pub.init_source = mem_init;
    s->pub.fill_input_buffer = mem_fill;
    s->pub.skip_input_data = mem_skip;
    s->pub.resync_to_restart = jpeg_resync_to_restart;
    s->pub.term_source = mem_term;
    s->pub.bytes_in_buffer = len;
    s->pub.next_input_byte = data;
}

struct my_err { struct jpeg_error_mgr pub; jmp_buf jb; };
static void my_err_exit(j_common_ptr c) {
    struct my_err *e = (struct my_err *)c->err;
    longjmp(e->jb, 1);
}

/* 仅读取 JPEG 原始尺寸（用于 AVI 头/录制），不解码像素，开销极小 */
static int jpeg_get_size(const unsigned char *data, size_t len, int *w, int *h)
{
    struct jpeg_decompress_struct c;
    struct my_err e;
    c.err = jpeg_std_error(&e.pub);
    e.pub.error_exit = my_err_exit;
    if (setjmp(e.jb)) { jpeg_destroy_decompress(&c); return -1; }
    jpeg_create_decompress(&c);
    jpeg_mem_src_custom((j_decompress_ptr)&c, data, len);
    jpeg_read_header(&c, TRUE);
    *w = (int)c.image_width; *h = (int)c.image_height;
    jpeg_destroy_decompress(&c);
    return 0;
}

/* 解码内存中 len 字节 JPEG -> 返回 malloc 的 RGB 缓冲(sw*sh*3)，填 *w,*h；失败返回 NULL
   ⚠️ out/rowbuf 必须声明为 volatile：longjmp 返回后被修改过的普通自动变量值是不确定的。
   并且在错误路径上必须释放它们——否则当相机输出非标准“类 JPEG”帧时（gspca 老相机常见
   风险），每帧都会泄漏一块内存，几秒内就能把内存吃光。 */
static unsigned char *decode_jpeg(const unsigned char *data, size_t len, int *w, int *h)
{
    struct jpeg_decompress_struct c;
    struct my_err e;
    unsigned char *volatile out = NULL;
    static JSAMPROW s_rowbuf = NULL;   /* 逐行缓冲静态复用，免每帧 malloc 一行 */
    static size_t s_rowcap = 0;
    JSAMPROW volatile rowbuf = NULL;
    c.err = jpeg_std_error(&e.pub);
    e.pub.error_exit = my_err_exit;
    if (setjmp(e.jb)) {
        if (out)    free((void *)out);     /* 仅释放本帧输出；行缓冲是静态的，不能 free */
        jpeg_destroy_decompress(&c);
        return NULL;
    }
    jpeg_create_decompress(&c);
    jpeg_mem_src_custom((j_decompress_ptr)&c, data, len);
    jpeg_read_header(&c, TRUE);
    c.out_color_space = JCS_RGB;
    /* 自动选缩小倍数：把解码输出限制在 DECODE_TARGET_W×H 以内（g6c 起默认 1024×600=LCD 原生，
       不再人为压低；仅当相机分辨率超过 LCD 时才缩小）。板子裸板无散热、CPU 弱，软解 MJPEG 是
       最大开销，若实测仍卡可把 DECODE_TARGET 调小（如 640×360）。libjpeg 只支持缩小(1/2/4/8)。
       录制存的是原始 JPEG，尺寸由 jpeg_get_size 另取，不受影响。 */
    int scale_denom = 1;
    for (;;) {
        int ow = (int)c.image_width / scale_denom;
        int oh = (int)c.image_height / scale_denom;
        if (ow <= DECODE_TARGET_W && oh <= DECODE_TARGET_H) break;
        if (scale_denom >= 8) break;           /* 已最大缩小，停 */
        scale_denom *= 2;
    }
    c.scale_num = 1; c.scale_denom = scale_denom;
    jpeg_start_decompress(&c);
    int sw = c.output_width, sh = c.output_height;
    unsigned char *o = malloc((long)sw * sh * 3);
    if (!o) { jpeg_destroy_decompress(&c); return NULL; }
    out = o;
    int rowbytes = sw * 3;
    if ((size_t)rowbytes > s_rowcap) {
        free(s_rowbuf);
        s_rowbuf = malloc((size_t)rowbytes);
        s_rowcap = (size_t)rowbytes;
    }
    JSAMPROW r = s_rowbuf;
    if (!r) { free(o); out = NULL; jpeg_destroy_decompress(&c); return NULL; }
    rowbuf = r;
    while (c.output_scanline < (unsigned int)sh) {
        jpeg_read_scanlines(&c, &r, 1);
        memcpy(o + (long)(c.output_scanline - 1) * rowbytes, r, rowbytes);
    }
    jpeg_finish_decompress(&c);
    jpeg_destroy_decompress(&c);
    /* 注意：r 指向静态 s_rowbuf，这里不能 free（已改为静态复用） */
    rowbuf = NULL;
    *w = sw; *h = sh;
    return out;
}

/* ---------------- MJPEG-AVI 录制（零编码，直接存原始 JPEG 帧） ---------------- */
/* recording 已在文件前部声明（供底部按钮绘制引用） */
static int rec_header_done = 0;      /* AVI 头是否已写（首帧到达后才写，因需帧尺寸） */
static FILE *rec_fp = NULL;
static char rec_path[320];
static long rec_frames = 0;
static uint32_t rec_w = 0, rec_h = 0;
static long rec_movi_data_start = 0; /* 'movi' 四字节之后的内容起点 */
static long rec_movi_list_off = 0;   /* 'LIST'(movi) 四字节起点（索引偏移基准，符合 ffmpeg/VLC 约定） */
static long rec_movi_size_off = 0;   /* movi LIST 的 size 字段偏移 */
static long rec_avih_uspf_off = 0;   /* avih.dwMicroSecPerFrame 偏移 */
static long rec_avih_totframes_off = 0;
static long rec_strh_rate_off = 0;   /* strh.dwRate 偏移（按真实帧率回填） */
static long rec_strh_len_off = 0;    /* strh.dwLength 偏移（stop 时回填总帧数） */
static struct timeval rec_t0;

struct rec_idx { uint32_t off; uint32_t sz; };
static struct rec_idx *rec_idx = NULL;
static size_t rec_idx_n = 0, rec_idx_cap = 0;

static void fput32le(FILE *f, uint32_t v){ unsigned char b[4]={(unsigned char)v,(unsigned char)(v>>8),(unsigned char)(v>>16),(unsigned char)(v>>24)}; fwrite(b,1,4,f); }
static void fput16le(FILE *f, uint16_t v){ unsigned char b[2]={(unsigned char)v,(unsigned char)(v>>8)}; fwrite(b,1,2,f); }

static void gen_default_path(char *buf, size_t n)
{
    time_t t = time(NULL); struct tm *tm = localtime(&t);
    snprintf(buf, n, "%s/rec_%04d%02d%02d_%02d%02d%02d.avi", snap_dir,
             tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
}

/* 只记录路径与状态，文件延迟到「首帧到达」才真正 fopen + 写 AVI 头。
   原因：之前在 rec_start 就 fopen("wb") 建空文件，若之后没有任何帧到达
   （流未就绪/被异常终止），磁盘上会留下一个 0 字节文件，
   变成"无法识别的视频"。延迟建文件可从源头杜绝空文件。 */
static void avirec_open(const char *path)
{
    rec_header_done = 0; rec_frames = 0; rec_idx_n = 0;
    strncpy(rec_path, path, sizeof rec_path - 1); rec_path[sizeof rec_path - 1] = 0;
    fprintf(stderr, "[rec] 准备录制 %s（首帧到达后建文件）\n", path);
}

static void avirec_write_header(uint32_t w, uint32_t h)
{
    fseek(rec_fp, 0, SEEK_SET);
    fwrite("RIFF", 1, 4, rec_fp);
    fput32le(rec_fp, 0);                 /* 占位 riff size（stop 时回填） */
    fwrite("AVI ", 1, 4, rec_fp);
    /* hdrl list */
    long hdrl_list_off = ftell(rec_fp);
    fwrite("LIST", 1, 4, rec_fp);
    long hdrl_size_off = ftell(rec_fp);
    fput32le(rec_fp, 0);
    fwrite("hdrl", 1, 4, rec_fp);
    /* avih */
    fwrite("avih", 1, 4, rec_fp);
    fput32le(rec_fp, 56);
    long uspf_off = ftell(rec_fp);
    fput32le(rec_fp, 0);                 /* dwMicroSecPerFrame 占位 */
    fput32le(rec_fp, 0);                 /* dwMaxBytesPerSec */
    fput32le(rec_fp, 0);                 /* dwPaddingGranularity */
    fput32le(rec_fp, 0x10);              /* dwFlags: HAS_INDEX */
    long tot_off = ftell(rec_fp);
    fput32le(rec_fp, 0);                 /* dwTotalFrames 占位 */
    fput32le(rec_fp, 0);                 /* dwInitialFrames */
    fput32le(rec_fp, 1);                 /* dwStreams */
    fput32le(rec_fp, 0);                 /* dwSuggestedBufferSize */
    fput32le(rec_fp, w);                 /* dwWidth */
    fput32le(rec_fp, h);                 /* dwHeight */
    for (int i = 0; i < 4; i++) fput32le(rec_fp, 0);
    /* strl list */
    fwrite("LIST", 1, 4, rec_fp);
    long strl_size_off = ftell(rec_fp);
    fput32le(rec_fp, 0);
    fwrite("strl", 1, 4, rec_fp);
    /* strh */
    fwrite("strh", 1, 4, rec_fp);
    fput32le(rec_fp, 56);
    fwrite("vids", 1, 4, rec_fp);
    fwrite("MJPG", 1, 4, rec_fp);
    fput32le(rec_fp, 0); fput16le(rec_fp, 0); fput16le(rec_fp, 0);
    fput32le(rec_fp, 0);
    fput32le(rec_fp, 1);                 /* dwScale */
    long rate_off = ftell(rec_fp);
    fput32le(rec_fp, 25);                /* dwRate（stop 时按真实帧率回填） */
    /* 以下 5 个 DWORD = dwStart / dwLength / dwSuggestedBufferSize /
       dwQuality / dwSampleSize，必须写满：标准 AVISTREAMHEADER 是 56 字节，
       上面声明了 cb=56。早期版本漏写了 dwSampleSize（只写 52 字节），
       导致解析器按 size=56 跳时会多跳 4 字节、吃掉后面 strf 的 chunk ID，
       整个 RIFF 树错位 —— 表现就是"录出来的 AVI 在电脑上放不了"。 */
    long len_off = ftell(rec_fp);
    fput32le(rec_fp, 0);                 /* dwStart */
    fput32le(rec_fp, 0);                 /* dwLength（stop 时回填总帧数） */
    fput32le(rec_fp, 0);                 /* dwSuggestedBufferSize */
    fput32le(rec_fp, 0xFFFFFFFF);        /* dwQuality = -1（默认质量） */
    fput32le(rec_fp, 0);                 /* dwSampleSize  ← 补上，凑满 56 */
    /* rcFrame */
    fput16le(rec_fp, 0); fput16le(rec_fp, 0);
    fput16le(rec_fp, (uint16_t)w); fput16le(rec_fp, (uint16_t)h);
    /* strf (BITMAPINFOHEADER) */
    fwrite("strf", 1, 4, rec_fp);
    fput32le(rec_fp, 40);
    fput32le(rec_fp, 40);
    fput32le(rec_fp, w); fput32le(rec_fp, h);
    fput16le(rec_fp, 1); fput16le(rec_fp, 24);
    fwrite("MJPG", 1, 4, rec_fp);
    fput32le(rec_fp, 0); fput32le(rec_fp, 0); fput32le(rec_fp, 0); fput32le(rec_fp, 0); fput32le(rec_fp, 0);
    /* 回填 hdrl / strl 的 size 字段 */
    long hdrl_end = ftell(rec_fp);
    fseek(rec_fp, hdrl_size_off, SEEK_SET); fput32le(rec_fp, (uint32_t)(hdrl_end - hdrl_list_off - 8));
    fseek(rec_fp, strl_size_off, SEEK_SET); fput32le(rec_fp, (uint32_t)(hdrl_end - strl_size_off - 8));

    /* ⚠ 关键修复：上面两次 fseek 是为了回填 size，文件指针已被挪到文件「中间」
       （strl_size_off=92，写完变 96）。若不 fseek 回 hdrl_end 就继续写，
       movi 的 "LIST" 会正好覆盖掉 strl 的 form type(@96)，strh / strf 也被一并吞掉
       —— 结果就是录出来的 AVI 里搜不到 strl/strh/strf，播放器解析不了流信息。 */
    fseek(rec_fp, hdrl_end, SEEK_SET);

    /* movi list：先写 LIST，size 字段紧随其后 */
    rec_movi_list_off = ftell(rec_fp);     /* 'LIST' 四字节起点（索引偏移基准） */
    rec_movi_size_off = ftell(rec_fp) + 4; /* 'LIST'(4) 之后的 size 字段位置 */
    fwrite("LIST", 1, 4, rec_fp);
    fput32le(rec_fp, 0);                 /* movi size 占位 */
    fwrite("movi", 1, 4, rec_fp);
    rec_movi_data_start = ftell(rec_fp);
    fflush(rec_fp);
    rec_header_done = 1;
    rec_avih_uspf_off = uspf_off;
    rec_avih_totframes_off = tot_off;
    rec_strh_rate_off = rate_off;
    rec_strh_len_off = len_off + 4;      /* dwStart 之后即 dwLength */
    rec_w = w; rec_h = h;
    gettimeofday(&rec_t0, NULL);
}

static void avirec_add_frame(const unsigned char *jpeg, size_t len, uint32_t w, uint32_t h)
{
    if (!jpeg || len <= 0) return;
    /* 首帧才真正建文件：延迟打开，杜绝"建了空文件却没来帧"的 0 字节录像 */
    if (!rec_fp) {
        if (!rec_path[0]) return;
        rec_fp = fopen(rec_path, "wb");
        if (!rec_fp) { fprintf(stderr, "[rec] 无法打开 %s: %s\n", rec_path, strerror(errno)); return; }
        /* 大 stdio 缓冲：避免每帧多次 tiny write 拖住主循环；配合下方每帧 fflush，
           崩溃/复位最多丢当前一帧，且磁盘文件不会长期停留在 0 字节。 */
        static char rec_iobuf[256 * 1024];
        setvbuf(rec_fp, rec_iobuf, _IOFBF, sizeof rec_iobuf);
        fprintf(stderr, "[rec] 建录制文件 %s\n", rec_path);
    }
    if (!rec_header_done) avirec_write_header(w, h);
    /* AVI 标准：movi 内每个 chunk 数据按 WORD(2 字节) 对齐填充。
       早期写成 4 字节对齐：老 mplayer(1.0rc2) 顺序读时按 2 字节推进，
       遇到奇数长度帧就少推进 2 字节、几帧后错位找不到下一个 00dc → 提前停
       （表现“回放只放开头”）。改 2 字节对齐后顺序读不再错位；且索引 seek 本就不受影响。 */
    uint32_t padded = (uint32_t)((len + 1) & ~1u);
    long frame_pos = ftell(rec_fp);
    fwrite("00dc", 1, 4, rec_fp);
    fput32le(rec_fp, (uint32_t)len);
    fwrite(jpeg, 1, len, rec_fp);
    if (padded > (uint32_t)len) {
        unsigned char z = 0;
        for (uint32_t i = 0; i < padded - (uint32_t)len; i++) fputc(z, rec_fp);
    }
    if (rec_idx_n >= rec_idx_cap) {
        rec_idx_cap = rec_idx_cap ? rec_idx_cap * 2 : 256;
        rec_idx = realloc(rec_idx, rec_idx_cap * sizeof(struct rec_idx));
    }
    /* AVI 标准：idx1 的 dwOffset 必须相对 'movi' 数据起点（'movi' 四字节之后），
       不是相对 'LIST' 标记起点。之前写成 rec_movi_list_off（多含 LIST/size/movi 共 12 字节），
       导致每帧偏移都偏 12 字节 —— 板子 mplayer 1.0rc2 死信 idx，每帧错位解码失败、提前停，
       表现就是“回放只放开头一小段”（电脑 VLC/ffmpeg 容错重索引能放完整，掩盖了此 bug）。 */
    rec_idx[rec_idx_n].off = (uint32_t)(frame_pos - rec_movi_data_start);
    rec_idx[rec_idx_n].sz  = padded;
    rec_idx_n++;
    rec_frames++;
    fflush(rec_fp);   /* 每帧落盘：限制崩溃/复位时的数据丢失，并避免磁盘文件长期为 0 字节 */
}

static void avirec_stop(void)
{
    if (!rec_fp) { rec_path[0] = 0; return; }   /* 没建文件 → 无内容，清空路径 */
    if (rec_header_done && rec_frames > 0) {
        long idx1_start = ftell(rec_fp);
        fwrite("idx1", 1, 4, rec_fp);
        fput32le(rec_fp, (uint32_t)(rec_idx_n * 16));
        for (size_t i = 0; i < rec_idx_n; i++) {
            fwrite("00dc", 1, 4, rec_fp);
            fput32le(rec_fp, 0x10);              /* AVIIF_KEYFRAME */
            fput32le(rec_fp, rec_idx[i].off);
            fput32le(rec_fp, rec_idx[i].sz);
        }
        long file_end = ftell(rec_fp);
        /* 回填 movi size（= LIST 内容字节数 = 'movi'四字节 + 各帧，即 idx1_start - LIST起点 - 8） */
        fseek(rec_fp, rec_movi_size_off, SEEK_SET);
        fput32le(rec_fp, (uint32_t)(idx1_start - rec_movi_list_off - 8));
        /* 回填 riff size */
        fseek(rec_fp, 4, SEEK_SET);
        fput32le(rec_fp, (uint32_t)(file_end - 8));
        /* 按真实时长回填帧率（avih + strh） */
        struct timeval t1; gettimeofday(&t1, NULL);
        long dt = (t1.tv_sec - rec_t0.tv_sec) * 1000000L + (t1.tv_usec - rec_t0.tv_usec);
        uint32_t uspf = dt > 0 ? (uint32_t)(dt / rec_frames) : 40000;
        uint32_t rate = uspf ? (1000000u / uspf) : 25;
        fseek(rec_fp, rec_avih_uspf_off, SEEK_SET); fput32le(rec_fp, uspf);
        fseek(rec_fp, rec_avih_totframes_off, SEEK_SET); fput32le(rec_fp, (uint32_t)rec_frames);
        fseek(rec_fp, rec_strh_rate_off, SEEK_SET); fput32le(rec_fp, rate);
        /* 回填 strh.dwLength（流长度=总帧数）：部分播放器据此算时长/进度条 */
        if (rec_strh_len_off > 0) {
            fseek(rec_fp, rec_strh_len_off, SEEK_SET);
            fput32le(rec_fp, (uint32_t)rec_frames);
        }
        fflush(rec_fp);
        double dur = dt > 0 ? (double)dt / 1000000.0 : 0.0;
        fprintf(stderr, "[rec] 已保存 %s （%ux%u, %ld 帧, %.1fs 录制, ~%u fps, %lu kB）\n",
                rec_path, rec_w, rec_h, rec_frames, dur, rate, (unsigned long)(file_end / 1024));
        /* 写实测帧率旁注，供板载 mplayer 回放时 -fps 强制按真实速率播放
           （MJPEG 常被 mplayer 按默认高帧率飞快播完，导致“回放明显短于录制时长”）。 */
        char fpsfn[400];
        snprintf(fpsfn, sizeof fpsfn, "%s.fps", rec_path);
        FILE *ff = fopen(fpsfn, "w");
        if (ff) { fprintf(ff, "%u\n", rate); fclose(ff); }
    } else {
        fprintf(stderr, "[rec] 未录到任何帧，删除空文件 %s\n", rec_path);
        fclose(rec_fp); rec_fp = NULL;
        remove(rec_path);
        rec_path[0] = 0;               /* 清空：rec_stop 才不会指向一个已删除的文件 */
        return;
    }
    fclose(rec_fp); rec_fp = NULL;
    /* 录制结束；rec_path 保留指向已保存文件，留板子本地（transcode_watch 转 mp4） */
}

/* 开始录像（若未指定路径则生成默认路径）
   注意：avirec_open 现已改为"只记路径、延迟到首帧才 fopen 建文件"（治 0 字节录像）。
   因此不能再以 rec_fp 是否存在来判定启动是否成功——否则 rec_fp 始终为 NULL，
   会在这里直接 return，导致 recording 永不置 1（现象：按下按钮有反色反馈，但怎么点都不开始录）。 */
static void rec_start(const char *path)
{
    if (recording) return;
    if (path && *path) avirec_open(path);
    else { char buf[320]; gen_default_path(buf, sizeof buf); avirec_open(buf); }
    if (!rec_path[0]) return;            /* 极端：路径都没记下，放弃 */
    recording = 1;
    fprintf(stderr, "[rec] 开始录像（首帧到达后写文件头）\n");
}

static void rec_stop(void)
{
    if (!recording) return;
    recording = 0;
    avirec_stop();
    /* 板载自包含：录完只留本地 MJPEG-AVI（/root/camera），导出转 mp4 由可选 transcode_avi.sh 单独跑（不回传 VM） */
}

/* ================= V4L2 摄像头采集（功能1 方案A：板载/USB 摄像头） =================
 * 把数据源从“手机 HTTP MJPEG”换成“板载/USB 摄像头 V4L2 采集”。
 * 自动选像素格式：优先 MJPEG（压缩，复用现有 libjpeg 解码与零编码录像），
 * 否则回退 YUYV（软件转 RGB + libjpeg 编码）。兼容 gspca_zc3xx 这类只给 MJPEG 的廉价
 * USB 摄像头，也兼容 OV5640 CSI 等给 YUYV 的板载模组。
 * 采集到帧后，显示/防抖/触摸框选跟踪/拍照/录像 全部复用现有流水线（与手机路径完全一致）。 */

enum { CAM_FMT_MJPEG = 0, CAM_FMT_YUYV = 1 };
static int cam_fd = -1;
static int cam_fmt = CAM_FMT_MJPEG;
static int cam_w = 0, cam_h = 0;
struct pv_cam_buf { void *start; size_t len; };
static struct pv_cam_buf *cam_bufs = NULL;
static unsigned int cam_nbufs = 0;

static int cam_init(const char *dev)
{
    cam_fd = open(dev, O_RDWR);
    if (cam_fd < 0) { perror("open cam"); return -1; }
    struct v4l2_capability cap;
    if (ioctl(cam_fd, VIDIOC_QUERYCAP, &cap) < 0) { perror("querycap"); return -1; }
    /* 老内核 gspca 只填 capabilities、不填 device_caps，必须 fallback */
    __u32 caps = (cap.device_caps != 0) ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) { fprintf(stderr, "%s 不是视频采集设备\n", dev); return -1; }
    if (!(caps & V4L2_CAP_STREAMING))      { fprintf(stderr, "%s 不支持 streaming mmap\n", dev); return -1; }

    /* 枚举格式：优先选 MJPEG（压缩），否则回退 YUYV */
    cam_fmt = CAM_FMT_YUYV;
    struct v4l2_fmtdesc fm;
    memset(&fm, 0, sizeof fm);
    fm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (ioctl(cam_fd, VIDIOC_ENUM_FMT, &fm) >= 0) {
        if (fm.pixelformat == V4L2_PIX_FMT_MJPEG || fm.pixelformat == V4L2_PIX_FMT_JPEG)
            cam_fmt = CAM_FMT_MJPEG;
        fm.index++;
    }

    /* 设定格式 + 分辨率（800x600 优先，失败回退 640x480 / 320x240） */
    __u32 pixfmt = (cam_fmt == CAM_FMT_MJPEG) ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
    int sizes[][2] = { {800,600}, {640,480}, {320,240} };
    int ok = 0;
    for (int s = 0; s < 3 && !ok; s++) {
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = sizes[s][0];
        fmt.fmt.pix.height = sizes[s][1];
        fmt.fmt.pix.pixelformat = pixfmt;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(cam_fd, VIDIOC_S_FMT, &fmt) == 0) { cam_w = fmt.fmt.pix.width; cam_h = fmt.fmt.pix.height; ok = 1; }
    }
    if (!ok) { perror("s_fmt"); return -1; }
    fprintf(stderr, "[cam] %s : %s 格式=%s %dx%d\n", dev, cap.card,
            (cam_fmt == CAM_FMT_MJPEG) ? "MJPEG" : "YUYV", cam_w, cam_h);

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof req);
    req.count = 4; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(cam_fd, VIDIOC_REQBUFS, &req) < 0) { perror("reqbufs"); return -1; }
    cam_nbufs = req.count;
    cam_bufs = calloc(cam_nbufs, sizeof(struct pv_cam_buf));
    if (!cam_bufs) { perror("calloc"); return -1; }
    for (unsigned int i = 0; i < cam_nbufs; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (ioctl(cam_fd, VIDIOC_QUERYBUF, &b) < 0) { perror("querybuf"); return -1; }
        cam_bufs[i].len = b.length;
        cam_bufs[i].start = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam_fd, b.m.offset);
        if (cam_bufs[i].start == MAP_FAILED) { perror("mmap cam"); return -1; }
    }
    for (unsigned int i = 0; i < cam_nbufs; i++) {
        struct v4l2_buffer b;
        memset(&b, 0, sizeof b);
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (ioctl(cam_fd, VIDIOC_QBUF, &b) < 0) { perror("qbuf"); return -1; }
    }
    enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam_fd, VIDIOC_STREAMON, &t) < 0) { perror("streamon"); return -1; }
    return 0;
}

/* 取一帧：返回 mmap 指针/长度/缓冲索引；用完后 cam_release_frame(idx) 重新入队 */
static int cam_capture_frame(const unsigned char **pdata, size_t *plen, unsigned int *idx)
{
    struct v4l2_buffer b;
    memset(&b, 0, sizeof b);
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP;
    if (ioctl(cam_fd, VIDIOC_DQBUF, &b) < 0) {
        if (errno == EINTR) return -2;          /* 被信号打断（Ctrl+C），调用方退出 */
        perror("dqbuf"); return -1;
    }
    *pdata = (const unsigned char *)cam_bufs[b.index].start;
    *plen = b.bytesused;
    *idx = b.index;
    return 0;
}

static void cam_release_frame(unsigned int idx)
{
    struct v4l2_buffer b;
    memset(&b, 0, sizeof b);
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = idx;
    ioctl(cam_fd, VIDIOC_QBUF, &b);
}

static void cam_cleanup(void)
{
    if (cam_fd >= 0) {
        enum v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam_fd, VIDIOC_STREAMOFF, &t);
        for (unsigned int i = 0; i < cam_nbufs; i++)
            if (cam_bufs[i].start != MAP_FAILED) munmap(cam_bufs[i].start, cam_bufs[i].len);
        free(cam_bufs); cam_bufs = NULL; cam_nbufs = 0;
        close(cam_fd); cam_fd = -1;
    }
}

/* YUYV(2 字节/像素) -> RGB(3 字节/像素)，BT.601 整数近似 */
static unsigned char *yuyv_to_rgb(const unsigned char *src, int w, int h)
{
    unsigned char *out = malloc((long)w * h * 3);
    if (!out) return NULL;
    for (long y = 0; y < h; y++) {
        const unsigned char *srow = src + y * w * 2;
        unsigned char *drow = out + y * w * 3;
        for (int x = 0; x < w; x += 2) {
            int Y0 = srow[x*2], U = srow[x*2+1], Y1 = srow[x*2+2], V = srow[x*2+3];
            int uu = U - 128, vv = V - 128;
            int r0 = Y0 + ((360*vv)>>8), g0 = Y0 - ((88*uu+183*vv)>>8), b0 = Y0 + ((455*uu)>>8);
            int r1 = Y1 + ((360*vv)>>8), g1 = Y1 - ((88*uu+183*vv)>>8), b1 = Y1 + ((455*uu)>>8);
            #define CL(v) ((v)<0?0:((v)>255?255:(v)))
            drow[x*3]   = (unsigned char)CL(r0); drow[x*3+1] = (unsigned char)CL(g0); drow[x*3+2] = (unsigned char)CL(b0);
            drow[x*3+3] = (unsigned char)CL(r1); drow[x*3+4] = (unsigned char)CL(g1); drow[x*3+5] = (unsigned char)CL(b1);
            #undef CL
        }
    }
    return out;
}

/* 自定义 JPEG 内存输出目标（老 libjpeg 无 jpeg_mem_dest） */
typedef struct {
    struct jpeg_destination_mgr pub;
    unsigned char *buf; size_t bufsize; size_t datalen;
} pv_jpeg_dest;

static void pv_jdest_init(j_compress_ptr c) {
    pv_jpeg_dest *d = (pv_jpeg_dest *)c->dest;
    d->buf = malloc(65536); d->bufsize = 65536; d->datalen = 0;
    c->dest->next_output_byte = d->buf; c->dest->free_in_buffer = d->bufsize;
}
static boolean pv_jdest_empty(j_compress_ptr c) {
    pv_jpeg_dest *d = (pv_jpeg_dest *)c->dest;
    size_t newcap = d->bufsize * 2;
    unsigned char *nb = realloc(d->buf, newcap);
    if (!nb) return FALSE;
    d->buf = nb; d->datalen = d->bufsize; d->bufsize = newcap;
    c->dest->next_output_byte = d->buf + d->datalen;
    c->dest->free_in_buffer = d->bufsize - d->datalen;
    return TRUE;
}
static void pv_jdest_term(j_compress_ptr c) {
    pv_jpeg_dest *d = (pv_jpeg_dest *)c->dest;
    d->datalen = d->bufsize - c->dest->free_in_buffer;
}

/* 把 RGB(3B/px) 编码为 JPEG；输出到 *out（调用方 free），返回长度；失败返回 0 */
static size_t encode_rgb_to_jpeg(const unsigned char *rgb, int w, int h, unsigned char **out)
{
    *out = NULL;
    struct jpeg_compress_struct c;
    struct jpeg_error_mgr jerr;
    c.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&c);
    pv_jpeg_dest dst;
    dst.pub.init_destination = pv_jdest_init;
    dst.pub.empty_output_buffer = pv_jdest_empty;
    dst.pub.term_destination = pv_jdest_term;
    c.dest = (struct jpeg_destination_mgr *)&dst;
    c.image_width = w; c.image_height = h; c.input_components = 3; c.in_color_space = JCS_RGB;
    jpeg_set_defaults(&c);
    jpeg_set_quality(&c, 80, TRUE);
    jpeg_start_compress(&c, TRUE);
    JSAMPROW row = malloc((size_t)w * 3);
    if (row) {
        while (c.next_scanline < (unsigned int)h) {
            memcpy(row, rgb + (long)c.next_scanline * w * 3, (size_t)w * 3);
            jpeg_write_scanlines(&c, &row, 1);
        }
        free(row);
    }
    jpeg_finish_compress(&c);
    jpeg_destroy_compress(&c);
    if (dst.datalen == 0) return 0;
    *out = dst.buf;               /* 调用方负责 free */
    return dst.datalen;
}

/* 摄像头主循环：采集 + 显示/防抖/框选/拍照/录像（与手机路径共用同一套流水线） */
static void cam_run_loop(const char *opath)
{
    long frames = 0;
    long decode_fail = 0;
    fprintf(stderr, "[phone_view] 摄像头采集中... Ctrl+C 退出\n");
    while (running) {
        if (exit_req) { running = 0; break; }
        if (playback_req) { playback_req = 0; start_board_playback(); }
        if (rec_toggle_req) { rec_toggle_req = 0; if (recording) rec_stop(); else rec_start(opath); }

        const unsigned char *data; size_t len; unsigned int idx;
        int r = cam_capture_frame(&data, &len, &idx);
        /* -2 = 阻塞的 DQBUF 被信号打断(EINTR)。只有 running 已被清才真退出（SIGTERM/Ctrl+C）；
           若是 USR1(录像)/USR2(拍照) 打进来的，必须回到循环开头去处理请求。 */
        if (r == -2) { if (running) continue; break; }
        if (r < 0)   { fprintf(stderr, "[cam] 采集异常，退出\n"); break; }
        if (len == 0) { cam_release_frame(idx); continue; }

        unsigned char *rgb = NULL;
        int rgb_w = 0, rgb_h = 0;
        const unsigned char *jpeg_bytes = NULL;
        size_t jpeg_len = 0;
        int jw = 0, jh = 0;

        if (cam_fmt == CAM_FMT_MJPEG) {
            rgb = decode_jpeg(data, len, &rgb_w, &rgb_h);
            int rw = 0, rh = 0;
            if (jpeg_get_size(data, len, &rw, &rh) == 0) { jw = rw; jh = rh; }
            jpeg_bytes = data; jpeg_len = len;
        } else {
            rgb = yuyv_to_rgb(data, cam_w, cam_h);
            if (rgb) {
                rgb_w = cam_w; rgb_h = cam_h; jw = cam_w; jh = cam_h;
                unsigned char *enc = NULL;
                size_t elen = encode_rgb_to_jpeg(rgb, cam_w, cam_h, &enc);
                if (elen > 0) { jpeg_bytes = enc; jpeg_len = elen; }
            }
        }

        if (rgb) {
            /* 拍照缓存（独立于 mmap/编码缓冲，循环里会 reuse） */
            if (snap_mode && jpeg_bytes && jpeg_len > 0) {
                if (g_snap_cap < jpeg_len) { unsigned char *nb = realloc(g_snap_buf, jpeg_len); if (nb) { g_snap_buf = nb; g_snap_cap = jpeg_len; } }
                if (g_snap_buf && g_snap_cap >= jpeg_len) { memcpy(g_snap_buf, jpeg_bytes, jpeg_len); g_snap_len = jpeg_len; }
            }
            /* 录像：存 JPEG 帧（MJPEG 源用相机原始帧；YUYV 源用刚编码的帧） */
            if (recording && jpeg_bytes && jpeg_len > 0) {
                if (!rec_header_done) avirec_add_frame(jpeg_bytes, jpeg_len, (uint32_t)jw, (uint32_t)jh);
                else avirec_add_frame(jpeg_bytes, jpeg_len, rec_w, rec_h);
            }
            if (stab_enable) {
                int ex = 0, ey = 0;
                estimate_motion(rgb, rgb_w, rgb_h, &ex, &ey);
                stab_sx = (int)(stab_sx * 8 / 10 + (-ex) * 2 / 10);
                stab_sy = (int)(stab_sy * 8 / 10 + (-ey) * 2 / 10);
                int m = (rgb_w > rgb_h ? rgb_w : rgb_h) / 10;
                if (stab_sx >  m) stab_sx =  m; if (stab_sx < -m) stab_sx = -m;
                if (stab_sy >  m) stab_sy =  m; if (stab_sy < -m) stab_sy = -m;
            }
            if (track_enable) track_apply(rgb, rgb_w, rgb_h, stab_enable, stab_sx, stab_sy);
            if (stab_enable) {
                draw_rgb_stab(rgb, rgb_w, rgb_h, stab_sx, stab_sy);
            } else {
                draw_rgb_scaled(rgb, rgb_w, rgb_h);
            }
            if (track_enable) {                       /* 触摸拖动即时反馈(黄框,LCD坐标) */
                pthread_mutex_lock(&ts_mtx);
                if (ts.down && ts.have_pt)
                    draw_fb_rect(ts.start_x, ts.start_y, ts.cur_x, ts.cur_y, 255, 255, 0, 2);
                pthread_mutex_unlock(&ts_mtx);
            }
            draw_bottom_buttons();                    /* 相机模式也要画出底部四按钮（之前只有手机模式画） */
            fb_flush();                               /* 双缓冲一次性提交，按钮不再闪动 */
            free(rgb);
            if (cam_fmt == CAM_FMT_YUYV && jpeg_bytes) free((void *)jpeg_bytes);
            frames++;
            if (snap_req) { snap_req = 0; do_snapshot(); }
        } else {
            /* 解码失败一定要“看得见”：否则相机给了非标准帧时画面全黑又没日志，
               很难判断是相机没出图还是 libjpeg 解不出来。 */
            if (++decode_fail % 30 == 1)
                fprintf(stderr, "[cam] 已 %ld 帧无法解码：请先用 /tmp/v4l2_probe 确认相机实际格式；"
                                "若它报 MJPEG 但这里持续失败，多半是 gspca 私有“类 JPEG”，"
                                "改用 YUYV 或换 UVC 摄像头\n", decode_fail);
        }
        cam_release_frame(idx);
        if (frames % 30 == 0 && frames > 0)
            fprintf(stderr, "[phone_view] frames=%ld\n", frames);
    }
    fprintf(stderr, "[phone_view] 摄像头共收 %ld 帧\n", frames);
}

/* ---------------- 网络 / HTTP / multipart ---------------- */

static int net_connect(const char *ip, int port)
{
    fprintf(stderr, "[http] connecting to %s:%d ...\n", ip, port);
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { perror("socket"); return -1; }

    /* 非阻塞 connect + 5 秒超时：手机 IP/端口不对时不再无限阻塞 */
    fcntl(sock, F_SETFL, O_NONBLOCK);
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port);
    a.sin_addr.s_addr = inet_addr(ip);
    int r = connect(sock, (struct sockaddr *)&a, sizeof a);
    if (r < 0 && errno != EINPROGRESS) {
        fprintf(stderr, "[http] connect 错误: %s\n", strerror(errno));
        close(sock); sock = -1; return -1;
    }
    fd_set wf; FD_ZERO(&wf); FD_SET(sock, &wf);
    struct timeval tv; tv.tv_sec = 5; tv.tv_usec = 0;
    r = select(sock + 1, NULL, &wf, NULL, &tv);
    if (r <= 0) {
        fprintf(stderr, "[http] connect 超时(5s)。请确认：板子能 ping 通手机? 手机IP=%s 是否正确、IP 摄像头 app 是否已 Start server、手机是否亮屏在前台\n", ip);
        close(sock); sock = -1; return -1;
    }
    int err = 0; socklen_t el = sizeof err;
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &el) < 0 || err != 0) {
        fprintf(stderr, "[http] connect 失败: %s\n", err ? strerror(err) : "unknown");
        close(sock); sock = -1; return -1;
    }
    /* 低延迟：
       1) TCP_NODELAY —— 禁 Nagle。MJPEG 每帧 24KB 会被拆成多个 TCP 段，
          Nagle 会攒够一个 MSS 再发，白白增加几十毫秒延迟。
       2) SO_RCVBUF 调小到 64KB —— 内核接收缓冲里若积压多帧陈旧画面，
          板子读到的永远是"过去"，端到端延迟变大。 */
    {
        int one = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        int rb = 65536;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rb, sizeof rb);
    }
    /* 恢复阻塞模式，但给接收设 2 秒超时：Ctrl+C 能及时退出、连接断了能感知 */
    fcntl(sock, F_SETFL, 0);
    struct timeval rt; rt.tv_sec = 2; rt.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rt, sizeof rt);
    fprintf(stderr, "[http] connected.\n");
    return 0;
}

/* 确保 g_buf 至少有 need 字节（不足则 recv 补足）；返回 -1 表示连接关闭/出错/收到退出信号 */
static int net_recv_fill(size_t need)
{
    while (g_len < need) {
        if (g_len >= g_cap) { g_cap = (g_cap ? g_cap * 2 : 65536); g_buf = realloc(g_buf, g_cap); }
        int n = recv(sock, (char *)g_buf + g_len, g_cap - g_len, 0);
        if (n > 0) { g_len += (size_t)n; continue; }
        if (n == 0) { fprintf(stderr, "[net] 对端关闭连接\n"); return -1; }
        if (errno == EINTR) continue;                              /* 被信号打断，重试 */
        if (errno == EAGAIN || errno == EWOULDBLOCK) {              /* 2s 接收超时 */
            if (!running) return -1;                                /* 已收到退出信号 */
            continue;                                              /* 继续等数据 */
        }
        fprintf(stderr, "[net] recv 错误: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/* 在 b[from, n) 中找子串 pat（长 plen），找不到返回 (size_t)-1 */
static size_t find_sub(const unsigned char *b, size_t n, size_t from,
                       const unsigned char *pat, size_t plen)
{
    if (plen == 0) return from;
    if (from + plen > n) return (size_t)-1;
    for (size_t i = from; i + plen <= n; i++)
        if (memcmp(b + i, pat, plen) == 0) return i;
    return (size_t)-1;
}

/* 在头部区域 h[0,n) 中解析 Content-Length（大小写不敏感），没有返回 0 */
static int parse_content_length(const unsigned char *h, size_t n)
{
    for (size_t i = 0; i + 15 <= n; i++) {
        if ((h[i] | 0x20) == 'c') {
            const char *pat = "content-length:";
            int ok = 1;
            for (int k = 0; k < 15; k++)
                if ((h[i + k] | 0x20) != (unsigned char)pat[k]) { ok = 0; break; }
            if (ok) return atoi((const char *)h + i + 15);
        }
    }
    return 0;
}

/* 把收到的原始字节（前 lim 字节）以可读方式 dump 到日志，方便确认手机到底回了什么 */
static void dump_raw(const unsigned char *b, size_t n, size_t lim)
{
    size_t m = n < lim ? n : lim;
    fprintf(stderr, "[http] ---- 原始返回前 %zu 字节 (共 %zu) ----\n", m, n);
    for (size_t i = 0; i < m; i++) {
        unsigned char c = b[i];
        if (c >= 0x20 && c < 0x7f) fputc(c, stderr);
        else if (c == '\r') fprintf(stderr, "\\r");
        else if (c == '\n') fprintf(stderr, "\\n\n");
        else fprintf(stderr, ".");
    }
    fprintf(stderr, "\n[http] ---- end dump ----\n");
}

/*
 * 尝试向手机请求某个 path 的 MJPEG 流。成功（拿到 boundary）返回 1，
 * 失败（连接/无 boundary）返回 0 并关闭 sock。g_buf/g_len/g_search 复位后写入响应头。
 */
static int try_path(const char *ip, int port, const char *path, char *boundary, size_t bsize)
{
    if (net_connect(ip, port) < 0) return 0;     /* 连不上直接失败（同主机端口，别再试别的路径） */

    char req[512];
    int rl = snprintf(req, sizeof req,
        "GET %s HTTP/1.0\r\nHost: %s\r\nAccept: image/jpeg, */*\r\n\r\n", path, ip);
    fprintf(stderr, "[http] 发送请求:\n%s\n", req);
    if (send(sock, req, (size_t)rl, 0) <= 0) { perror("send"); close(sock); sock = -1; return 0; }

    /* 读 HTTP 响应头直到 \r\n\r\n（最多 8KB，避免异常时死等） */
    g_len = 0; g_search = 0;
    int header_ok = 0;
    for (int guard = 0; guard < 200; guard++) {
        if (net_recv_fill(g_len + 256) < 0) break;
        /* 注意：IP Webcam 等服务器发完 header 会紧接着推 body(JPEG)，
           缓冲区末尾是正文而非 \r\n\r\n，所以必须在“已收数据任意位置”找
           header 结束符，不能只看缓冲区末尾，否则会把正常的流误判为“头不完整”。 */
        if (find_sub(g_buf, g_len, 0, (unsigned char *)"\r\n\r\n", 4) != (size_t)-1) {
            header_ok = 1; break;
        }
        if (g_len >= 8192) break;  /* 头异常长，强制停 */
    }
    if (!header_ok) {
        fprintf(stderr, "[http] 没读到完整响应头 (path=%s)，手机可能未起流/路径错/被防火墙挡\n", path);
        dump_raw(g_buf, g_len, 1500);
        close(sock); sock = -1; return 0;
    }
    fprintf(stderr, "[http] 收到响应头 (path=%s)\n", path);
    dump_raw(g_buf, g_len, 1500);

    char *bp = strstr((char *)g_buf, "boundary=");
    if (!bp) {
        fprintf(stderr, "[http] 该路径(%s)响应头里没有 boundary（可能不是 MJPEG 流）\n", path);
        close(sock); sock = -1; return 0;
    }
    bp += 9;
    /* 跳过值里可能自带的 '--' 前缀（部分 app header 写成 boundary=--xxx） */
    if (bp[0] == '-' && bp[1] == '-') bp += 2;
    size_t bi = 0;
    while (*bp && bi < bsize-1 && *bp != '\r' && *bp != '\n' &&
           *bp != '"' && *bp != ';' && *bp != ' ' && *bp != ',' ) {
        boundary[bi++] = *bp++;
    }
    boundary[bi] = 0;
    if (bi == 0) { fprintf(stderr, "[http] 解析 boundary 失败 (path=%s)\n", path); close(sock); sock = -1; return 0; }
    fprintf(stderr, "[http] 拿到 boundary=%s (path=%s)\n", boundary, path);
    return 1;
}

/* ================= 回放模式（-b：离线演示显示+防抖+人脸框，不依赖实时相机） =================
 * 把目录里的一批图片（默认 BMP 24/32 位；也接受 *.jpg、*.jpeg 走现有 libjpeg 解码）当连续帧，
 * 跑与相机/手机路径完全相同的「缩放上屏 / 防抖 / 人脸框」流水线并循环播放。
 * 用于相机暂时不可用（如 USB 高速握手失败掉全速、video 设备注册不出）时的效果演示与流水线验证。
 * 用法：./phone_view -b <图片目录> [-s 防抖] [-f 人脸框]        （仅显示+叠加层，不录像/不拍照） */

/* 加载 24/32 位 BMP -> 返回 malloc 的 RGB(sw*sh*3)，填 *w,*h；失败返回 NULL。
   BMP 为 BGR 且通常自下而上存储，这里统一翻转为自上而下 RGB。 */
static unsigned char *load_bmp_rgb(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    unsigned char hdr[54];
    if (fread(hdr, 1, 54, f) != 54) { fclose(f); return NULL; }
    if (hdr[0] != 'B' || hdr[1] != 'M') { fclose(f); return NULL; }
    int bw, bh, bpp, off;
    memcpy(&bw, hdr + 18, 4);
    memcpy(&bh, hdr + 22, 4);
    memcpy(&bpp, hdr + 28, 2);
    memcpy(&off, hdr + 10, 4);
    if (bw <= 0 || bh == 0 || (bpp != 24 && bpp != 32)) { fclose(f); return NULL; }
    int ah = bh < 0 ? -bh : bh;
    int bppb = bpp / 8;
    int stride = (bw * bppb + 3) & ~3;
    unsigned char *rgb = malloc((long)bw * ah * 3);
    if (!rgb) { fclose(f); return NULL; }
    unsigned char *row = malloc(stride);
    if (!row) { free(rgb); fclose(f); return NULL; }
    if (off > 54) fseek(f, off, SEEK_SET);
    for (int y = 0; y < ah; y++) {
        if (fread(row, 1, stride, f) != (size_t)stride) break;
        int out_y = (bh < 0) ? y : (ah - 1 - y);   /* 自下而上需翻转到顶部起算 */
        unsigned char *dst = rgb + (long)out_y * bw * 3;
        for (int x = 0; x < bw; x++) {
            unsigned char *p = row + x * bppb;
            dst[0] = p[2]; dst[1] = p[1]; dst[2] = p[0];   /* BGR -> RGB */
            dst += 3;
        }
    }
    free(row); fclose(f);
    *w = bw; *h = ah;
    return rgb;
}

/* 从文件读 JPEG 并解码为 RGB（回放 .jpg/.jpeg 用），失败返回 NULL */
static unsigned char *decode_jpeg_file(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return NULL; }
    unsigned char *buf = malloc((size_t)sz);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) { free(buf); fclose(f); return NULL; }
    fclose(f);
    unsigned char *rgb = decode_jpeg(buf, (size_t)sz, w, h);
    free(buf);
    return rgb;
}

static int name_cmp(const void *a, const void *b)
{
    return strcmp(*(const char *const *)a, *(const char *const *)b);
}

static void playback_run(const char *dir, int fps)
{
    DIR *dp = opendir(dir);
    if (!dp) { fprintf(stderr, "[play] 无法打开目录 %s: %s\n", dir, strerror(errno)); return; }
    char **names = NULL; size_t n = 0, cap = 0;
    struct dirent *de;
    while ((de = readdir(dp)) != NULL) {
        const char *nm = de->d_name;
        size_t l = strlen(nm);
        if (l < 4) continue;
        int is_img = (strcasecmp(nm + l - 4, ".bmp") == 0) ||
                     (l >= 5 && (strcasecmp(nm + l - 5, ".jpg") == 0 ||
                                 strcasecmp(nm + l - 5, ".jpeg") == 0));
        if (!is_img) continue;
        if (n >= cap) { cap = cap ? cap * 2 : 256; names = realloc(names, cap * sizeof(char *)); }
        names[n] = strdup(nm);
        if (names[n]) n++;
    }
    closedir(dp);
    if (n == 0) { fprintf(stderr, "[play] 目录 %s 下没有 .bmp/.jpg/.jpeg 文件\n", dir); free(names); return; }
    qsort(names, n, sizeof(char *), name_cmp);
    fprintf(stderr, "[play] 共载入 %zu 张图片，循环播放 @%dfps（Ctrl+C 退出）\n", n, fps);
    if (fps <= 0) fps = 30;
    long frames = 0;
    while (running) {
        for (size_t i = 0; i < n && running; i++) {
            char path[1024];
            snprintf(path, sizeof path, "%s/%s", dir, names[i]);
            size_t l = strlen(names[i]);
            int w = 0, h = 0;
            unsigned char *rgb = (strcasecmp(names[i] + l - 4, ".bmp") == 0)
                                 ? load_bmp_rgb(path, &w, &h)
                                 : decode_jpeg_file(path, &w, &h);
            if (!rgb) { fprintf(stderr, "[play] 跳过 %s\n", names[i]); continue; }
            if (stab_enable) {
                int ex = 0, ey = 0;
                estimate_motion(rgb, w, h, &ex, &ey);
                stab_sx = (int)(stab_sx * 8 / 10 + (-ex) * 2 / 10);
                stab_sy = (int)(stab_sy * 8 / 10 + (-ey) * 2 / 10);
                int m = (w > h ? w : h) / 10;
                if (stab_sx >  m) stab_sx =  m; if (stab_sx < -m) stab_sx = -m;
                if (stab_sy >  m) stab_sy =  m; if (stab_sy < -m) stab_sy = -m;
            }
            if (track_enable) track_apply(rgb, w, h, stab_enable, stab_sx, stab_sy);
            if (stab_enable) {
                draw_rgb_stab(rgb, w, h, stab_sx, stab_sy);
            } else {
                draw_rgb_scaled(rgb, w, h);
            }
            if (track_enable) {                       /* 触摸拖动即时反馈(黄框,LCD坐标) */
                pthread_mutex_lock(&ts_mtx);
                if (ts.down && ts.have_pt)
                    draw_fb_rect(ts.start_x, ts.start_y, ts.cur_x, ts.cur_y, 255, 255, 0, 2);
                pthread_mutex_unlock(&ts_mtx);
            }
            fb_flush();                               /* 双缓冲提交，照片幻灯才可见 */
            free(rgb);
            frames++;
            if (frames % 30 == 0) fprintf(stderr, "[play] frames=%ld\n", frames);
            usleep(1000000 / fps);
        }
    }
    for (size_t i = 0; i < n; i++) free(names[i]);
    free(names);
    fprintf(stderr, "[play] 回放结束，共显示 %ld 帧\n", frames);
}

/* ================= 单实例守卫 =================
 * 板子上若残留多个 phone_view（切模式时旧进程没杀干净），会同时往 /dev/fb0
 * 写图，造成“一大一小两个取景”交替闪烁。启动时先干掉同名旧实例，保证 fb0
 * 永远只有一个写者。用 /proc/<pid>/comm 校验进程名，避免 PID 被复用后误杀
 * 无关进程。 */
static int is_phone_view(pid_t pid)
{
    char path[64];
    snprintf(path, sizeof path, "/proc/%d/comm", (int)pid);
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    char comm[32];
    int ok = 0;
    if (fgets(comm, sizeof comm, f)) {
        size_t n = strlen(comm);
        while (n && (comm[n-1] == '\n' || comm[n-1] == '\r')) comm[--n] = 0;
        if (strcmp(comm, "phone_view") == 0) ok = 1;
    }
    fclose(f);
    return ok;
}

static void single_instance_takeover(void)
{
    /* 扫 /proc 直接杀掉所有“名为 phone_view 且不是自己”的旧实例。
     * 不依赖 pidfile：旧版本（g6h 及之前）从没写过 pidfile，靠它找不到残留
     * 实例，升级第一次仍会闪；扫 /proc 能覆盖任何手滑残留与升级过渡。 */
    pid_t self = getpid();
    DIR *d = opendir("/proc");
    if (!d) return;
    struct dirent *de;
    while ((de = readdir(d)) != NULL) {
        const char *p = de->d_name;
        if (*p < '0' || *p > '9') continue;          /* 只处理数字目录(进程) */
        pid_t pid = (pid_t)atoi(p);
        if (pid <= 1 || pid == self) continue;
        if (is_phone_view(pid)) {
            fprintf(stderr, "[phone_view] 发现同名旧实例 pid=%d，终止它（避免双进程抢屏闪烁）\n", (int)pid);
            kill(pid, SIGTERM);
        }
    }
    closedir(d);
    struct timespec ts = {0, 200000000};             /* 200ms 等其优雅退出 */
    nanosleep(&ts, NULL);
    d = opendir("/proc");                            /* 仍存活者强杀 */
    if (d) {
        while ((de = readdir(d)) != NULL) {
            const char *p = de->d_name;
            if (*p < '0' || *p > '9') continue;
            pid_t pid = (pid_t)atoi(p);
            if (pid <= 1 || pid == self) continue;
            if (is_phone_view(pid)) kill(pid, SIGKILL);
        }
        closedir(d);
    }
}

int main(int argc, char **argv)
{
    setvbuf(stderr, NULL, _IONBF, 0);   /* 日志不缓冲，异常/崩溃也能看到中间信息 */
    fprintf(stderr, "[phone_view] === 版本 %s 启动 ===\n", PV_VERSION);
#ifdef HAVE_TURBO_JPEG
    fprintf(stderr, "[phone_view] 解码库: libjpeg-turbo (NEON 加速)\n");
#else
    fprintf(stderr, "[phone_view] 解码库: IJG 原版 libjpeg (无 NEON)\n");
#endif
    fprintf(stderr, "[phone_view] 录像控制：启动加 -o <文件> 即录；运行中 kill -USR1 $(pidof phone_view) 切换起停\n");
    single_instance_takeover();   /* 先干掉残留旧实例，避免双进程抢 fb0 闪烁 */

    /* 命令行解析： -o <录像文件> / -s 防抖 之外，按 ip / port / 流路径 顺序填充 */
    const char *ip = "192.168.2.121";
    const char *opath = NULL;
    int port = 8080;
    const char *stream_path = NULL;
    const char *cam_dev = NULL;
    const char *play_dir = NULL;
    int ip_set = 0, port_set = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) { opath = argv[++i]; continue; }
        if (strcmp(argv[i], "-s") == 0) { stab_enable = 1; continue; }
        if (strcmp(argv[i], "-f") == 0) { track_enable = 1; continue; }
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            strncpy(snap_dir, argv[++i], sizeof snap_dir - 1);
            snap_dir[sizeof(snap_dir) - 1] = 0; snap_mode = 1; continue;
        }
        if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) { cam_dev = argv[++i]; continue; }
        if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) { play_dir = argv[++i]; continue; }
        /* -u 上传目标参数已移除（板载自包含，不再回传 VM） */
        if (!ip_set) { ip = argv[i]; ip_set = 1; }
        else if (!port_set) { port = atoi(argv[i]); port_set = 1; }
        else { stream_path = argv[i]; }
    }
    /* 板载自包含：无上传目标，拍/录统一存到媒体目录（默认 /root/camera） */
    /* 确保媒体目录存在（板子重启后子目录不会自动建；已存在则忽略 EEXIST） */
    if (mkdir(snap_dir, 0755) != 0 && errno != EEXIST)
        fprintf(stderr, "[phone_view] 警告：创建媒体目录 %s 失败: %s\n", snap_dir, strerror(errno));
    fprintf(stderr, "[phone_view] 媒体目录: %s\n", snap_dir);
    fprintf(stderr, "[phone_view] 防抖: %s\n", stab_enable ? "开启(-s)" : "关闭");
    fprintf(stderr, "[phone_view] 触摸跟踪: %s\n", track_enable ? "开启(-f)" : "关闭");
    fprintf(stderr, "[phone_view] 拍照: %s%s\n", snap_mode ? "开启(-p " : "关闭",
            snap_mode ? snap_dir : "");

    if (fb_init() < 0) return 1;
    /* sa_flags 默认 0 = 不自动重启被中断的系统调用，保证 Ctrl+C 能打断 recv/connect */
    struct sigaction sa; memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sig;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    struct sigaction sa2; memset(&sa2, 0, sizeof sa2);
    sa2.sa_handler = on_usr1;
    /* ⚠️ USR1/USR2 是“请求信号”，必须设 SA_RESTART：否则它们会打断阻塞中的
       VIDIOC_DQBUF 使其返回 EINTR，被误判成退出信号而把整个进程杀掉。 */
    sa2.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa2, NULL);
    struct sigaction sa3; memset(&sa3, 0, sizeof sa3);
    sa3.sa_handler = on_usr2;
    sa3.sa_flags = SA_RESTART;
    sigaction(SIGUSR2, &sa3, NULL);

    /* 触摸屏读取线程：始终启动（不再限定 track_enable）。
       它负责两件事：① 底部按钮点击（拍照/录像）——与 -f 无关，裸跑也必须可用；
       ② 触摸框选跟踪——仅 -f 时由主循环 track_apply 消费。
       原 bug：只在 track_enable 时才创建线程，导致裸跑（不带 -f）时按钮点击完全失效。 */
    pthread_t tt;
    if (pthread_create(&tt, NULL, touch_thread, NULL) != 0)
        fprintf(stderr, "[touch] 线程创建失败，触摸交互(按钮/框选)不可用（显示正常）\n");

    /* 局部变量提前声明（避免下方相机分支 goto done 跨过初始化声明） */
    const char *paths[4]; int npaths = 0;
    char boundary[160]; boundary[0] = 0;
    char sep[176]; int seplen = 0;

    /* 数据源分支：指定 -c 设备则走 V4L2 摄像头（功能1 方案A）；否则走手机 HTTP 投屏 */
    if (cam_dev) {
        fprintf(stderr, "[phone_view] 数据源: 板载/USB 摄像头 %s\n", cam_dev);
        if (cam_init(cam_dev) < 0) goto done;
        cam_run_loop(opath);
        cam_cleanup();
        if (recording) { recording = 0; avirec_stop(); }
        goto done;                          /* 跳过手机 HTTP 路径 */
    }

    /* 回放模式：目录里批量图片当连续帧，跑同款显示+防抖+人脸框流水线（不依赖实时相机） */
    if (play_dir) {
        fprintf(stderr, "[phone_view] 数据源: 回放目录 %s\n", play_dir);
        playback_run(play_dir, 30);
        goto done;
    }

    /* 以下为手机 HTTP MJPEG 投屏路径 */
    if (stream_path) { paths[0] = stream_path; npaths = 1; }
    else { paths[0] = "/video"; paths[1] = "/videofeed"; paths[2] = "/mjpegfeed"; npaths = 3; }

    int got = 0;
    for (int i = 0; i < npaths && !got; i++) {
        if (try_path(ip, port, paths[i], boundary, sizeof boundary)) got = 1;
    }
    if (!got) {
        fprintf(stderr, "[phone_view] 没能从手机拿到 MJPEG 流，退出。\n");
        fprintf(stderr, "排查：① 手机装 IP Webcam 类 app 并点 Start server；② 板子 ping %s 通不通；③ 确认 app 显示的地址端口。\n", ip);
        goto done;
    }

    seplen = snprintf(sep, sizeof sep, "--%s", boundary);

    fprintf(stderr, "[phone_view] 拉流中... Ctrl+C 退出\n");

    /* 主循环：按 boundary 切帧。
       关键策略：每轮只解码显示“最新完整帧”，中间过期的帧直接丢弃（不解码）——否则
       收包快于画屏会让缓冲越堆越多、显示指针永远追不上接收指针，表现为“延迟越来越高”。
       录制则把每一帧的原始 JPEG 都存盘（不依赖解码），保证录像不丢帧。 */
    long frames = 0;
    while (running) {
        if (exit_req) { running = 0; break; }
        if (playback_req) { playback_req = 0; start_board_playback(); }
        /* 处理录像切换请求（SIGUSR1 置位） */
        if (rec_toggle_req) {
            rec_toggle_req = 0;
            if (recording) rec_stop();
            else rec_start(opath);
        }
        /* 板载自包含：拍完/录完只留本地（见 do_snapshot / rec_stop），不回传 VM。 */

        /* 1) 定位下一个帧起始（boundary） */
        size_t first = find_sub(g_buf, g_len, g_search, (unsigned char *)sep, (size_t)seplen);
        if (first == (size_t)-1) {
            if (net_recv_fill(g_len + 65536) < 0) break;  /* 数据不够，继续收 */
            continue;
        }

        /* 2) 从 first 向后扫描，找出“最后一个完整帧”（结尾 boundary 已到） */
        size_t cur = first;
        size_t last_pos = (size_t)-1, last_nx = (size_t)-1;
        for (;;) {
            size_t hp = cur + (size_t)seplen;
            /* 末尾的 "--boundary--" 表示流结束 */
            if (hp + 2 <= g_len && g_buf[hp] == '-' && g_buf[hp + 1] == '-') {
                fprintf(stderr, "[phone_view] 收到流结束标记\n"); running = 0; break;
            }
            size_t he = find_sub(g_buf, g_len, hp, (unsigned char *)"\r\n\r\n", 4);
            if (he == (size_t)-1) break;          /* 帧头没收全，等更多数据 */
            size_t datastart = he + 4;
            size_t nx = find_sub(g_buf, g_len, datastart, (unsigned char *)sep, (size_t)seplen);
            if (nx == (size_t)-1) break;          /* 帧结尾（下一 boundary）未到，等更多数据 */
            int cl = parse_content_length(g_buf + hp, he - hp);
            size_t dataend = nx;
            if (cl > 0 && datastart + (size_t)cl < nx) dataend = datastart + (size_t)cl;
            size_t jlen = dataend - datastart;
            /* 录制：每一帧都存原始 JPEG（首帧若头未写则解码拿尺寸，之后直接用已记录尺寸） */
            if (recording && jlen > 0) {
                if (!rec_header_done) {
                    int w0 = 0, h0 = 0;
                    /* 只读原始尺寸写 AVI 头，不解码像素（省 CPU）；录制存的是原始 JPEG */
                    if (jpeg_get_size(g_buf + datastart, jlen, &w0, &h0) == 0)
                        avirec_add_frame(g_buf + datastart, jlen, (uint32_t)w0, (uint32_t)h0);
                } else {
                    avirec_add_frame(g_buf + datastart, jlen, rec_w, rec_h);
                }
            }
            last_pos = cur; last_nx = nx;
            cur = nx;                              /* 继续往后看是否有更新的帧 */
        }
        if (!running) break;
        if (last_pos == (size_t)-1) {
            /* 还没有任何完整帧（first 那一帧的结尾 boundary 未到），继续收 */
            if (net_recv_fill(g_len + 65536) < 0) break;
            continue;
        }

        /* 3) 只解码并显示最新一帧（过期的中间帧已跳过，不解码，杜绝延迟累积） */
        size_t hp = last_pos + (size_t)seplen;
        size_t he = find_sub(g_buf, g_len, hp, (unsigned char *)"\r\n\r\n", 4);
        size_t datastart = he + 4;
        int cl = parse_content_length(g_buf + hp, he - hp);
        size_t dataend = last_nx;
        if (cl > 0 && datastart + (size_t)cl < last_nx) dataend = datastart + (size_t)cl;
        size_t jlen = dataend - datastart;
        if (jlen > 0) {
            /* 拍照模式：缓存当前帧原始 JPEG（独立于 g_buf，后面 memmove 会移动 g_buf） */
            if (snap_mode) {
                if (g_snap_cap < jlen) { unsigned char *nb = realloc(g_snap_buf, jlen); if (nb) { g_snap_buf = nb; g_snap_cap = jlen; } }
                if (g_snap_buf && g_snap_cap >= jlen) { memcpy(g_snap_buf, g_buf + datastart, jlen); g_snap_len = jlen; }
            }
            int w = 0, h = 0;
            unsigned char *rgb = decode_jpeg(g_buf + datastart, jlen, &w, &h);
            if (rgb) {
                if (stab_enable) {
                    int ex = 0, ey = 0;
                    estimate_motion(rgb, w, h, &ex, &ey);
                    /* 低通平滑补偿：跟随 -内容位移，抵消高频抖动、保留平滑移动 */
                    stab_sx = (int)(stab_sx * 8 / 10 + (-ex) * 2 / 10);
                    stab_sy = (int)(stab_sy * 8 / 10 + (-ey) * 2 / 10);
                    int m = (w > h ? w : h) / 10;     /* 补偿上限 = margin */
                    if (stab_sx >  m) stab_sx =  m; if (stab_sx < -m) stab_sx = -m;
                    if (stab_sy >  m) stab_sy =  m; if (stab_sy < -m) stab_sy = -m;
                }
                if (track_enable) track_apply(rgb, w, h, stab_enable, stab_sx, stab_sy);
                if (stab_enable) {
                    draw_rgb_stab(rgb, w, h, stab_sx, stab_sy);
                } else {
                    draw_rgb_scaled(rgb, w, h);
                }
                if (track_enable) {                   /* 触摸拖动即时反馈(黄框,LCD坐标) */
                    pthread_mutex_lock(&ts_mtx);
                    if (ts.down && ts.have_pt)
                        draw_fb_rect(ts.start_x, ts.start_y, ts.cur_x, ts.cur_y, 255, 255, 0, 2);
                    pthread_mutex_unlock(&ts_mtx);
                }
                /* 底部四按钮（每帧重绘，悬浮于视频上）
                   按下反馈：按下期间按钮反色（底变亮、字变深），
                   手指一按立刻有视觉响应，不用等抬起。 */
                draw_bottom_buttons();
                fb_flush();                           /* 双缓冲一次性提交，按钮不再闪动 */
                free(rgb); frames++;
                if (snap_req) { snap_req = 0; do_snapshot(); }   /* 拍照请求：存当前已缓存帧 */
            }
            else fprintf(stderr, "[phone_view] 帧解码失败，跳过\n");
        }

        /* 4) 丢弃所有已消费数据（含被跳过的中间帧），缓冲立即瘦身 */
        memmove(g_buf, g_buf + last_nx, g_len - last_nx);
        g_len -= last_nx;
        g_search = 0;

        if (frames % 30 == 0 && frames > 0)
            fprintf(stderr, "[phone_view] frames=%ld\n", frames);
    }
    fprintf(stderr, "[phone_view] 共收 %ld 帧\n", frames);

    done:
    if (recording) rec_stop();   /* 退出时停止录像，文件留板子本地（不回传 VM） */
    cam_cleanup();
    if (sock >= 0) close(sock);
    if (fb_real) munmap(fb_real, fb_screensize);
    if (fb_mem) free(fb_mem);
    if (fb_fd >= 0) close(fb_fd);
    free(g_buf);
    free(rec_idx);
    free(stab_prev);
    free(g_snap_buf);
    fprintf(stderr, "[phone_view] stopped.\n");
    return 0;
}
