/*
   Copyright (c) 2018, Adrian Rossiter
   Modified 2026

   Antiprism - http://www.antiprism.com

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

      The above copyright notice and this permission notice shall be included
      in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#include "display.h"
#include "display_info.h"
#include "programopts.h"
#include "timer.h"
#include "utils.h"

#include <errno.h>
#include <locale.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <math.h>
#include <string>
#include <vector>

using std::string;
using std::vector;

const string VERSION = "0.03";
const string PROG_NAME = "mpd_oled";
const int SPECT_WIDTH = 64;

struct CavaContext {
  pid_t pid = -1;
  FILE *fifo_file = nullptr;
  string fifo_path = "";
  string config_path = "";
};

// --- Global Variables ---
ArduiPi_OLED display;
CavaContext g_cava;
ArduiPi_OLED* g_display_ptr = nullptr;

namespace {
pthread_mutex_t disp_info_lock;
}

// Systematically tear down running CAVA instances and clean filesystem allocations
void stop_cava(CavaContext &ctx)
{
  if (ctx.pid > 0) {
    // 1. Send SIGTERM first for graceful cleanup
    kill(ctx.pid, SIGTERM);

    // 2. Poll for up to 100ms to see if it closes gracefully
    int status;
    int retry = 10;
    pid_t res = 0;
    while (retry > 0) {
      res = waitpid(ctx.pid, &status, WNOHANG);
      if (res > 0) {
        break; // Child exited cleanly
      }
      usleep(10000); // Wait 10ms
      retry--;
    }

    // 3. Force kill if CAVA ignores SIGTERM or gets stuck in audio callbacks
    if (res <= 0) {
      kill(ctx.pid, SIGKILL);
      waitpid(ctx.pid, &status, 0); // Clean up the zombie process entry
    }
    ctx.pid = -1;
  }

  if (ctx.fifo_file != nullptr) {
    fclose(ctx.fifo_file);
    ctx.fifo_file = nullptr;
  }

  if (!ctx.fifo_path.empty()) {
    unlink(ctx.fifo_path.c_str());
    ctx.fifo_path = "";
  }

  if (!ctx.config_path.empty()) {
    unlink(ctx.config_path.c_str());
    ctx.config_path = "";
  }
}

void cleanup(void)
{
  stop_cava(g_cava);

  if (g_display_ptr != nullptr) {
    g_display_ptr->invertDisplay(false);
    g_display_ptr->clearDisplay();
    g_display_ptr->display();
    g_display_ptr->close();
  }
}

void signal_handler(int /*sig*/)
{
  cleanup();
  _exit(0);
}

void init_signals(void)
{
  struct sigaction new_action;
  memset(&new_action, 0, sizeof(new_action));
  new_action.sa_handler = &signal_handler;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;

  struct sigaction old_action;
  sigaction(SIGINT, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN)
    sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGHUP, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN)
    sigaction(SIGHUP, &new_action, NULL);
  sigaction(SIGTERM, NULL, &old_action);
  if (old_action.sa_handler != SIG_IGN)
    sigaction(SIGTERM, &new_action, NULL);
}

class OledOpts : public ProgramOpts {
public:
  const double DEF_SCROLL_RATE = 8;
  const double DEF_SCROLL_DELAY = 5;
  int bars = 16;
  string cava_method = "fifo";
  string cava_prog_name = "cava";
  string cava_source = "/tmp/mpd.fifo";
  int clock_format = 0;
  int date_format = 0;
  int framerate = 25;
  int gap = 1;
  unsigned char i2c_addr = 0;
  int i2c_bus = 1;
  double invert = 0;
  bool logo = false;
  int oled = 6;
  vector<double> scroll;
  char pause_screen = 'p';
  int reset_gpio = 25;
  bool rotate180 = false;
  bool sleep = false;
  bool spectrum = true;
  int spi_dc_gpio = OLED_SPI_DC;
  int spi_cs = OLED_SPI_CS0;

  OledOpts() : ProgramOpts(PROG_NAME, VERSION)
  {
    scroll.push_back(DEF_SCROLL_RATE);
    scroll.push_back(DEF_SCROLL_DELAY);
    scroll.push_back(DEF_SCROLL_RATE);
    scroll.push_back(DEF_SCROLL_DELAY);
  }
  void process_command_line(int argc, char **argv);
  void usage();
};

void OledOpts::usage()
{
  string oled_type = "";
  char buf[128];
  for (int i = 0; i < OLED_LAST_OLED; i++) {
    if (strstr(oled_type_str[i], "128x64")) {
      snprintf(buf, sizeof(buf), "%*s%d - %s\n", 16, "", i, oled_type_str[i]);
      oled_type += buf;
    }
  }
  oled_type.pop_back();
  fprintf(stdout, R"(
Usage: mpd_oled [options] [input_file]

Display information about an MPD-based player on an OLED screen

Options
  -a <addr>  I2C address, in hex          (default: from -o <type>)
  -B num     I2C bus number               (default: 1 > /dev/i2c-1)
  -b <num>   number of bars to display    (default: 16)
  -C <fmt>   clock format:                (default: 0)
                 0 - 24h leading 0
                 1 - 24h no leading 0
                 2 - 12h leading 0
                 3 - 12h no leading 0
  -c         cava input method and source (default: %s,%s)
                 e.g. fifo,/tmp/my_fifo, alsa,hw:5,0, pulse
  -D <gpio>  SPI DC GPIO number           (default: 24)
  -d         use USA format MM-DD-YYYY    (default: DD-MM-YYYY)
  -f <hz>    framerate (Hz)               (default: 15)
  -g <sz>    gap between bars (pixel)     (default: 1)
  -h --help  this info
  -I <val>   invert black/white:          (default: n)
                 n - normal
                 i - invert
                 h - switch between n and i with this period (hour),
                     which may help avoid screen burn
  -o <type>  OLED type:                   (default: 6)
%s
  -P <val>   pause screen type:           (default: p)
                 p - play
                 s - stop
  -R         rotate display 180 degrees
  -r <gpio>  I2C/SPI reset GPIO number    (default: 25)
  -S <num>   SPI CS number                (default: 0)
  -s <vals>  scroll rate and start delay  (default: %.1f,%.1f)
             up to four comma separated decimal values:
                 rate_all
                 rate_all,delay_all
                 rate_title,delay_all,rate_artist
                 rate_title,delay_title,rate_artist,delay_artist
  -v         version
  -X         display all data              (default: spectrum only)
  -x         display rAudio logo
  -z         clear display

Example :
%s -o 6 - use a %s OLED
)",
    cava_method.c_str(), cava_source.c_str(),
    oled_type.c_str(),
    DEF_SCROLL_RATE, DEF_SCROLL_DELAY,
    get_program_name().c_str(), oled_type_str[6]);
}

void OledOpts::process_command_line(int argc, char **argv)
{
  opterr = 0;
  int c;
  int method_len;

  handle_long_opts(argc, argv);

  while ((c = getopt(argc, argv, ":ha:B:b:C:c:D:df:g:I:o:P:Rr:S:s:vXxz")) != -1)
  {
    if (common_opts(c, optopt))
      continue;

    switch (c)
    {
    case 'a':
      if (strlen(optarg) != 2 || strspn(optarg, "01234567890aAbBcCdDeEfF") != 2)
        error("I2C address should be two hexadecimal digits", c);

      i2c_addr = (unsigned char)strtol(optarg, NULL, 16);
      break;

    case 'b':
      print_status_or_exit(read_int(optarg, &bars), c);
      if (bars < 2 || bars > 60)
        error("select between 2 and 60 bars", c);
      break;

    case 'C':
      print_status_or_exit(read_int(optarg, &clock_format), c);
      if (clock_format < 0 || clock_format > 3)
        error("clock format number is not 0, 1, 2 or 3", c);
      break;

    case 'c':
      method_len = 5;
      if (strncmp(optarg, "fifo,", method_len) == 0) {
        cava_method = "fifo";
        if (optarg[method_len] == '\0')
          error("cava input method is fifo, but no FIFO path was specified", c);
      }
      else if (strncmp(optarg, "alsa,", method_len) == 0) {
        cava_method = "alsa";
        if (optarg[method_len] == '\0')
          error("cava input method is alsa, but no ALSA stream was specified",
                c);
      }
      else if (strncmp(optarg, "pulse", method_len) == 0) {
        cava_method = "pulse";
        if (optarg[method_len] != '\0')
          error("cava input method is pulse, but is followed by extra text", c);
      }
      else
        error("cava input specifier is not in form 'fifo,fifo_path', "
              "'alsa,alsa_stream', or 'pulse'",
              c);

      cava_source = &optarg[method_len];
      break;

    case 'D':
      print_status_or_exit(read_int(optarg, &spi_dc_gpio), c);
      if (!isdigit(optarg[0]) || reset_gpio < 0 || reset_gpio > 99)
        error("probably invalid (not integer in range 0 - 99), specify the\n"
              "GPIO number of the pin that SPI DC is connected to",
              c);
      break;

    case 'd':
      date_format = 1;
      break;

    case 'f':
      print_status_or_exit(read_int(optarg, &framerate), c);
      if (framerate < 1)
        error("framerate must be a positive integer", c);
      break;

    case 'g':
      print_status_or_exit(read_int(optarg, &gap), c);
      if (gap < 0 || gap > 30)
        error("gap must be between 0 and 30 pixels", c);
      break;

    case 'I':
      if (strcmp(optarg, "n") == 0)
        invert = 0;
      else if (strcmp(optarg, "i") == 0)
        invert = -1;
      else if (read_double(optarg, &invert)) {
        if (invert <= 0)
          error("number of hours for period must be positive number", c);
      }
      else
        error("invalid value, should be n, i or a positive number", c);
      break;

    case 'k':
      cava_prog_name = "cava";
      break;

    case 'o':
      print_status_or_exit(read_int(optarg, &oled), c);
      if (oled < 0 || oled >= OLED_LAST_OLED ||
          !strstr(oled_type_str[oled], "128x64"))
        error(msg_str("invalid 128x64 oled type %d (see -h)", oled), c);
      break;

    case 'P':
      if (strcmp(optarg, "p") == 0)
        pause_screen = 'p';
      else if (strcmp(optarg, "s") == 0)
        pause_screen = 's';
      else
        error("pause screen type is not p or s", c);
      break;

    case 'R':
      rotate180 = true;
      break;

    case 'r':
      print_status_or_exit(read_int(optarg, &reset_gpio), c);
      if (!isdigit(optarg[0]) || reset_gpio < 0 || reset_gpio > 99)
        error("probably invalid (not integer in range 0 - 99), specify the\n"
              "GPIO number of the pin that RST is connected to",
              c);
      break;

    case 'S':
      print_status_or_exit(read_int(optarg, &spi_cs), c);
      if (spi_cs < 0 || spi_cs > 1)
        error("SPI CS should be 0 or 1", c);
      break;

    case 's':
      print_status_or_exit(read_double_list(optarg, scroll, 4), c);
      if (scroll.size() < 1)
        scroll.push_back(DEF_SCROLL_RATE);
      else if (scroll[0] < 0)
        error("scroll rate cannot be negative", c);

      if (scroll.size() < 2)
        scroll.push_back(DEF_SCROLL_DELAY);
      else if (scroll[1] < 0)
        error("scroll delay cannot be negative", c);

      if (scroll.size() < 3)
        scroll.push_back(scroll[0]);
      else if (scroll[2] < 0)
        error("scroll rate (origin/artist) cannot be negative", c);

      if (scroll.size() < 4)
        scroll.push_back(scroll[1]);
      else if (scroll[3] < 0)
        error("scroll delay (origin/artist) cannot be negative", c);
      break;

    case 'v':
      fprintf(stdout, (PROG_NAME + " " + VERSION + "\n").c_str());
      exit(0);
      break;

    case 'X':
      spectrum = false;
      break;

    case 'x':
      logo = true;
      break;

    case 'z':
      sleep = true;
      break;

    default:
      error("unknown command line error");
    }
  }

  if (argc - optind > 0)
    error(msg_str("invalid option or parameter: '%s'", argv[optind]));

  if (oled == 0)
    error("must specify a 128x64 oled", 'o');

  const int min_spect_width = bars + (bars - 1) * gap;
  if (min_spect_width > SPECT_WIDTH)
    error(msg_str(
        "spectrum graph width is %d: to display %d bars with a gap of %d\n"
        "requires a minimum width of %d. Reduce the number of bars and/or the "
        "gap\n",
        SPECT_WIDTH, bars, gap, min_spect_width));
}

string print_config_file(int bars, int framerate, string cava_method,
                         string cava_source, string fifo_path_cava_out)
{
  char templt[] = "/tmp/cava_config_XXXXXX";
  int fd = mkstemp(templt);
  if (fd == -1)
    return "";
  FILE *ofile = fdopen(fd, "w");
  if (ofile == NULL)
    return "";

  fprintf(ofile, R"(
[general]
framerate = %d
bars = %d

[input]
method = %s
source = %s

[output]
method = raw
data_format = binary
channels = mono
raw_target = %s
bit_format = 8bit
)",
    framerate,
    bars,
    cava_method.c_str(),
    cava_source.c_str(),
    fifo_path_cava_out.c_str()
  );
  fclose(ofile);
  return templt;
}

Status start_cava(CavaContext &ctx, const OledOpts &opts)
{
  ctx.fifo_path = msg_str("/tmp/cava_fifo_%d", getpid());
  unlink(ctx.fifo_path.c_str());

  if (mkfifo(ctx.fifo_path.c_str(), 0666) == -1) {
    opts.error("could not create cava output FIFO for writing: " + string(strerror(errno)));
    return Status::error("FIFO initialization failed");
  }

  ctx.config_path = print_config_file(opts.bars, opts.framerate, opts.cava_method,
                                       opts.cava_source, ctx.fifo_path);
  if (ctx.config_path == "") {
    unlink(ctx.fifo_path.c_str());
    opts.error("could not create cava config file: " + string(strerror(errno)));
    return Status::error("Configuration creation failed");
  }

  ctx.pid = fork();
  if (ctx.pid < 0) {
    unlink(ctx.fifo_path.c_str());
    unlink(ctx.config_path.c_str());
    opts.error("could not fork process to spawn cava execution context: " + string(strerror(errno)));
    return Status::error("Fork subprocessing failed");
  }
  else if (ctx.pid == 0) {
    char* args[] = {
        const_cast<char*>(opts.cava_prog_name.c_str()),
        const_cast<char*>("-p"),
        const_cast<char*>(ctx.config_path.c_str()),
        nullptr
    };
    execvp(args[0], args);
    _exit(EXIT_FAILURE);
  }

  ctx.fifo_file = fopen(ctx.fifo_path.c_str(), "rb");
  if (ctx.fifo_file == NULL) {
    kill(ctx.pid, SIGKILL);
    unlink(ctx.fifo_path.c_str());
    unlink(ctx.config_path.c_str());
    opts.error("could not open cava output FIFO for reading");
    return Status::error("Reader channel connection failed");
  }

  return Status::ok();
}

void draw_clock(ArduiPi_OLED &display, const display_info &disp_info)
{
  display.clearDisplay();
  const int W = 6;
  draw_text(display, 22, 0, 16, disp_info.conn.get_ip_addr());
  draw_connection(display, 128 - 2 * W, 0, disp_info.conn);
  draw_time(display, 4, 16, 4, disp_info.clock_format);
  draw_date(display, 32, 56, 1, disp_info.date_format);
}

void draw_spect_display(ArduiPi_OLED &display, const display_info &disp_info)
{
  const int H = 8;
  const int W = 6;
  draw_spectrum(display, 0, 0, SPECT_WIDTH, 32, disp_info.spect);
  draw_connection(display, 128 - 2 * W, 0, disp_info.conn);
  draw_triangle_slider(display, 128 - 5 * W, 1, 11, 6,
                       disp_info.status.get_volume());
  if (disp_info.status.get_kbitrate() > 0)
    draw_text(display, 128 - 10 * W, 0, 4, disp_info.status.get_kbitrate_str());

  int clock_offset = (disp_info.clock_format < 2) ? 0 : -2;
  draw_time(display, 128 - 10 * W + clock_offset, 2 * H, 2,
            disp_info.clock_format);

  vector<double> scroll_origin(disp_info.scroll.begin() + 2,
                               disp_info.scroll.begin() + 4);
  draw_text_scroll(display, 0, 4 * H + 4, 20, disp_info.status.get_origin(),
                   scroll_origin, disp_info.text_change.secs());

  vector<double> scroll_title(disp_info.scroll.begin(),
                              disp_info.scroll.begin() + 2);
  draw_text_scroll(display, 0, 6 * H, 20, disp_info.status.get_title(),
                   scroll_title, disp_info.text_change.secs());

  draw_solid_slider(display, 0, 7 * H + 6, 128, 2,
                    100 * disp_info.status.get_progress());
}

void draw_display(ArduiPi_OLED &display, const display_info &disp_info)
{
  mpd_state state = disp_info.status.get_state();
  if (state == MPD_STATE_UNKNOWN || state == MPD_STATE_STOP ||
      (state == MPD_STATE_PAUSE && disp_info.pause_screen == 's'))
    draw_clock(display, disp_info);
  else
    draw_spect_display(display, disp_info);
}

void draw_logo(ArduiPi_OLED &display)
{
  uint8_t bitmap[] = {
    0x7F, 0xFF, 0xF0, 0xFF, 0xFE, 0xFF, 0xFF, 0xF0, 0x3F, 0xFF, 0xFF, 0xFF, 0xF0, 0x1F, 0xFF, 0xFF,
    0xFF, 0xF0, 0x0F, 0xFF, 0xFF, 0xFF, 0xF0, 0x07, 0xFF, 0xFF, 0xFF, 0xF0, 0x03, 0xFF, 0xFF, 0xFF,
    0xF0, 0x83, 0xFF, 0xFF, 0xFF, 0xF0, 0xC1, 0xFF, 0xFF, 0xFF, 0xF0, 0xE1, 0xFF, 0xFF, 0xFF, 0xF0,
    0xE1, 0xFF, 0xFF, 0x0F, 0xF0, 0xE1, 0xFF, 0xFF, 0x0F, 0xF0, 0xE1, 0xFF, 0xFF, 0x0F, 0xF0, 0xE1,
    0xFF, 0xFF, 0x0F, 0xF0, 0xE1, 0xFF, 0xFF, 0x0F, 0xF0, 0xC1, 0xFF, 0xFF, 0x0F, 0xF0, 0x83, 0xFF,
    0xFF, 0x0F, 0xF0, 0x03, 0xFF, 0xFF, 0x0F, 0xF0, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0x00,
    0x00, 0x00, 0x1F, 0xFF, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0x0F,
    0xF0, 0x3F, 0xFF, 0xFF, 0x0F, 0xF0, 0x1F, 0xFF, 0xFF, 0x0F, 0xF0, 0x0F, 0xFF, 0xFF, 0x0F, 0xF0,
    0x07, 0xFF, 0xFF, 0x0F, 0xF0, 0x03, 0xFF, 0xFF, 0x0F, 0xF0, 0x01, 0xFF, 0xFF, 0x0F, 0xF0, 0x80,
    0xFF, 0xFF, 0x0F, 0xF0, 0xC0, 0x7F, 0xFF, 0xFF, 0xF0, 0xE0, 0x3F, 0xFF, 0xFF, 0xF0, 0xF0, 0x1F,
    0xFF, 0xFF, 0xF0, 0xF8, 0x0F, 0xFF, 0xFF, 0xF0, 0xFC, 0x07, 0xFF, 0xFF, 0xF0, 0xFE, 0x03, 0xFF,
    0xFF, 0xF0, 0xFF, 0x01, 0xFF, 0xFF, 0xF0, 0xFF, 0x80, 0xFF, 0xFF, 0xF0, 0xFF, 0xC0, 0xFF, 0xFF,
    0xF0, 0xFF, 0xE0, 0x7F, 0xFF, 0xF0, 0xFF, 0xF0
  };
  const int H = 40;
  const int W = 40;
  display.drawBitmap( (128 - W) / 2, (64 - H) / 2, bitmap, W, H, WHITE);
}

void *update_info(void *data)
{
  const float delay_secs = 0.3;
  display_info *disp_info_orig = (display_info *)data;
  while (true) {
    pthread_mutex_lock(&disp_info_lock);
    display_info disp_info = *disp_info_orig;
    pthread_mutex_unlock(&disp_info_lock);

    disp_info.status.init();
    disp_info.conn_init();

    pthread_mutex_lock(&disp_info_lock);
    disp_info_orig->update_from(disp_info);
    pthread_mutex_unlock(&disp_info_lock);

    usleep(delay_secs * 1000000);
  }
};

bool get_invert(double period)
{
  return (period > 0) ? (fmod(time(0) / 3600.0, 2 * period) > period) : period;
}

int start_idle_loop(ArduiPi_OLED &display, const OledOpts &opts)
{
  const double update_sec = 1 / (0.9 * opts.framerate);
  const long select_usec = update_sec * 1100000;
  Timer timer;

  display_info disp_info;
  disp_info.scroll = opts.scroll;
  disp_info.clock_format = opts.clock_format;
  disp_info.date_format = opts.date_format;
  disp_info.pause_screen = opts.pause_screen;
  disp_info.spect.init(opts.bars, opts.gap);
  disp_info.status.init();

  pthread_t update_info_thread;
  if (pthread_create(&update_info_thread, NULL, update_info, (void *)(&disp_info))) {
    fprintf(stderr, "error: could not create pthread\n");
    return 1;
  }

  if (pthread_mutex_init(&disp_info_lock, NULL) != 0) {
    fprintf(stderr, "error: could not create pthread mutex\n");
    return 2;
  }

  int fifo_fd = -1;
  int zero_read_cnt = 0;

  while (true) {
    int num_bars_read = 0;
    if (fifo_fd >= 0) {
      fd_set set;
      FD_ZERO(&set);
      FD_SET(fifo_fd, &set);

      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = select_usec;

      if (select(FD_SETSIZE, &set, NULL, NULL, &timeout) > 0) {
        do {
          num_bars_read = fread(&disp_info.spect.heights[0], sizeof(unsigned char),
                                disp_info.spect.heights.size(), g_cava.fifo_file);

          FD_ZERO(&set);
          FD_SET(fifo_fd, &set);
          timeout.tv_sec = 0;
          timeout.tv_usec = 0;
        } while (select(FD_SETSIZE, &set, NULL, NULL, &timeout) > 0);
      }
    }

    if (num_bars_read == 0)
      zero_read_cnt++;
    else
      zero_read_cnt = 0;

    if (zero_read_cnt > 1 || disp_info.status.get_state() != MPD_STATE_PLAY) {
      std::fill(disp_info.spect.heights.begin(), disp_info.spect.heights.end(), 0);
      usleep(0.1 * 1000000);
    }

    if (timer.finished() || num_bars_read) {
      display.clearDisplay();
      pthread_mutex_lock(&disp_info_lock);
      display.invertDisplay(get_invert(opts.invert));
      if (opts.spectrum)
        draw_spectrum(display, 0, 0, 128, 64, disp_info.spect);
      else
        draw_display(display, disp_info);

      pthread_mutex_unlock(&disp_info_lock);
      display.display();
    }

    if (timer.finished()) {
      display.reset_offset();
      if (disp_info.status.get_state() == MPD_STATE_PLAY && fifo_fd < 0) {
        opts.print_status_or_exit(start_cava(g_cava, opts));
        fifo_fd = fileno(g_cava.fifo_file);
      }

      timer.set_timer(update_sec);
    }
  }

  return 0;
}

int main(int argc, char **argv)
{
  setlocale(LC_CTYPE, "C.UTF-8");
  OledOpts opts;
  opts.process_command_line(argc, argv);

  g_display_ptr = &display;

  if (!init_display(display, opts.oled, opts.i2c_addr, opts.i2c_bus,
                    opts.reset_gpio, opts.spi_dc_gpio, opts.spi_cs,
                    opts.rotate180))
    opts.error("could not initialise OLED");

  if ( opts.logo ) {
    display.clearDisplay();
    draw_logo(display);
    display.display();
    exit(0);
  }

  if ( opts.sleep ) {
    display.clearDisplay();
    display.display();
    exit(0);
  }

  init_signals();
  atexit(cleanup);

  int loop_ret = start_idle_loop(display, opts);

  if (loop_ret != 0)
    exit(EXIT_FAILURE);

  return 0;
}
