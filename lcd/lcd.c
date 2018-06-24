// ELECOM NASのLCDを制御する

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

#include <time.h>

//#define BUSYLOOP_DEBUG


// GPIO

#if 0

#define GPIO_VCC (1u << 0) // GPIO64
#define GPIO_VDD (1u << 1) // GPIO65
#define GPIO_SCL (1u << 2) // GPIO66
#define GPIO_SDA (1u << 3) // GPIO67

#define BUSY_LOOP_COUNT 400 // I2C出力時のビジーループ周回数

// SoCのレジスタに直接アクセスするためのポインタ
static uint32_t *gpio_regs_0; // 0xF000_0000
static uint32_t *gpio_regs_7; // 0xF007_0000
static int fd_mem;


// GPIOレジスタの初期設定
void gpio_init(void)
{
  volatile uint32_t *p;

  // SoCのGPIO関連レジスタに直接アクセスする準備
  fd_mem = open("/dev/mem", O_RDWR|O_SYNC);
  gpio_regs_0 = (uint32_t *)mmap(0, 4096, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd_mem, 0xf0000000);
  gpio_regs_7 = (uint32_t *)mmap(0, 4096, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd_mem, 0xf0070000);

  // GPIO64〜67を有効にする
  p = gpio_regs_0 + (0x0024 / sizeof(uint32_t)); // 0xf000_0024 : GPIO_MUX_2
  *p |= 0xf;
  // 入出力の方向を出力にする
  p = gpio_regs_7 + (0x0210 / sizeof(uint32_t)); // 0xf007_0210 : GPIO2_CFG
  *p &= ~0xf;
}

void gpio_uninit(void)
{
  munmap(gpio_regs_0, 4096);
  munmap(gpio_regs_7, 4096);
  close(fd_mem);
}

// GPIO出力値更新
// @param reg レジスタ (GPIO_VCC, GPIO_VDD, GPIO_SCL, GPIO_SDA)
// @param value 出力値
void gpio_set(int reg, int value)
{
  volatile uint32_t *p;
  p = gpio_regs_7 + (0x0214 / 4); // 0xf007_0214 : GPIO2_OUT
  
  if (value) {
    *p |= reg;
  } else {
    *p &= ~reg;
  }
}

#else

#define GPIO_VCC 0 // GPIO64
#define GPIO_VDD 1 // GPIO65
#define GPIO_SCL 2 // GPIO66
#define GPIO_SDA 3 // GPIO67

#define BUSY_LOOP_COUNT 240 // I2C出力時のビジーループ周回数

static int fd_gpio[4];

// GPIOレジスタの初期設定
void gpio_init(void)
{
  int i;
  for (i = 0; i < 4; i++) {
    char buf[256];

    // /sys/class/gpio/gpio** が存在しないときだけexportする
    // ※既にexport済みのGPIOポートをさらにexportすると異常動作してしまう!
    sprintf(buf, "/sys/class/gpio/gpio%d/value", i + 64);
    if (access(buf, F_OK) != 0) {
      sprintf(buf, "echo %d > /sys/class/gpio/export", i + 64);
      system(buf);
    }
    sprintf(buf, "echo 0 > /sys/class/gpio/gpio%d/value", i + 64);
    system(buf);
    sprintf(buf, "echo out > /sys/class/gpio/gpio%d/direction", i + 64);
    system(buf);

    sprintf(buf, "/sys/class/gpio/gpio%d/value", i + 64);
    fd_gpio[i] = open(buf, O_RDWR|O_SYNC);
  }

#if 0
  fd_gpio[0] = open("/sys/class/gpio/gpio64/value", O_RDWR|O_SYNC);
  fd_gpio[1] = open("/sys/class/gpio/gpio65/value", O_RDWR|O_SYNC);
  fd_gpio[2] = open("/sys/class/gpio/gpio66/value", O_RDWR|O_SYNC);
  fd_gpio[3] = open("/sys/class/gpio/gpio67/value", O_RDWR|O_SYNC);
  // todo direction, export
#endif
}

void gpio_uninit(void)
{
}

// GPIO出力値更新
// @param reg レジスタ (GPIO_VCC, GPIO_VDD, GPIO_SCL, GPIO_SDA)
// @param value 出力値
void gpio_set(int reg, int value)
{
  const char *buf[] = {"0\n", "1\n"};
  write(fd_gpio[reg], buf[value], 2);
}


#endif

// I2C

volatile static uint32_t i2c_dummy;

// I2C通信タイミング用のウェイト
// 送信1ビットごとに本関数が3回呼ばれる。
void i2c_wait(void)
{
  // ビジーループ
  int i;
  for (i = BUSY_LOOP_COUNT; i > 0; i--) {
    i2c_dummy++;
  }
}

// ここでackを受信するつもりだが、
// 面倒なのでやってない。
void i2c_ack(void)
{
  gpio_set(GPIO_SDA, 1);
  i2c_wait();
  gpio_set(GPIO_SCL, 1);
  i2c_wait();
  // ここでスレーブからのackを待つつもり
  gpio_set(GPIO_SCL, 0);
  i2c_wait();
}

#ifdef BUSYLOOP_DEBUG
static long t_max = 0;
static long t_min = 1000000000;
#endif

// dataをMSBから順に8ビット出力する
// 初期状態はSCL=0であること。
void i2c_write_byte(uint32_t data)
{
  int i;

#ifdef BUSYLOOP_DEBUG
  struct timespec tv;
  long t0, t;
  clock_gettime(CLOCK_REALTIME, &tv);
  t0 = tv.tv_nsec;
#endif

  for (i = 0; i < 8; i++) {
    gpio_set(GPIO_SDA, ((data & 0x80) != 0));
    i2c_wait();
    gpio_set(GPIO_SCL, 1);
    i2c_wait();
    gpio_set(GPIO_SCL, 0);
    i2c_wait();
    data <<= 1;
  }
  i2c_ack();

#ifdef BUSYLOOP_DEBUG
  clock_gettime(CLOCK_REALTIME, &tv);
  // 1bitあたりの所要時間[ns]
  t = ((1000000000 + tv.tv_nsec - t0) % 1000000000) / 9;
  if (t_max < t) t_max = t;
  if (t_min > t) t_min = t;
#endif
}

// スタートコンディションとスレーブアドレスを出力する
void i2c_start(void)
{
  // スタートコンディション = SCL=1のときにSDAを1→0に変更
  gpio_set(GPIO_SDA, 1);
  i2c_wait();
  gpio_set(GPIO_SCL, 1);
  i2c_wait();
  gpio_set(GPIO_SDA, 0);
  i2c_wait();
  gpio_set(GPIO_SCL, 0);
  i2c_wait();
  // OLEDのスレーブアドレス出力
  i2c_write_byte(0x78);
}

// ストップコンディションを出力する
void i2c_Stop(void)
{
  // ストップコンディション = SCL=1のときにSDAを0→1に変更
  gpio_set(GPIO_SDA, 0);
  i2c_wait();
  gpio_set(GPIO_SCL, 1);
  i2c_wait();
  gpio_set(GPIO_SDA, 1);
  i2c_wait();
}
  
void i2c_write_command(uint32_t cmd)
{
  i2c_start();
  i2c_write_byte(0x00);
  i2c_write_byte(cmd);
  i2c_Stop();
}

void i2c_write_data(uint32_t data)
{
  i2c_start();
  i2c_write_byte(0x40);
  i2c_write_byte(data);
  i2c_Stop();
}

/////////////////////////////////


// Function Set
// N : number of line is 2/1
// DH: Double height font control for 2-line mode enable/disable
// RE: Extension register
// IS: Extension register
void oled_function_set(uint32_t DH, uint32_t RE, uint32_t IS)
{
  i2c_write_command(0x28 | (DH << 2) | (RE << 1) | IS);
}


// OLED Characterization
// SD: 0 = Normal Register, 1 = Extension Register
void oled_characterization(uint32_t SD)
{
  i2c_write_command(0x78 | SD);
}


void Set_CGRAM_Blink(uint32_t arg0)
{
  i2c_write_command(0x2a | arg0);
}



// IS=X, RE=X, SD=0
void oled_clear_display(void)
{
  oled_characterization(0);
  i2c_write_command(0x01);
  usleep(1520); // 1.52ms
}


// IS=X, RE=0, SD=0

void oled_return_home(void)
{
  oled_function_set(0, 0, 0);
  oled_characterization(0);
  i2c_write_command(0x01);
  usleep(1520); // 1.52ms
}

void oled_entry_mode_set(uint32_t ID, uint32_t S)
{
  oled_function_set(0, 0, 0);
  oled_characterization(0);
  i2c_write_command(0x04 | (ID << 1) | S);
}

// Display ON/OFF
// D: display
// C: cursor
// B: blink
void oled_display_on_off(uint32_t D, uint32_t C, uint32_t B)
{
  oled_function_set(0, 0, 0);
  oled_characterization(0);
  i2c_write_command(0x08 | (D << 2) | (C << 1) | B);
}

// IS=0, RE=0, SD=0

void oled_set_cursor_or_display_shift(uint32_t SC, uint32_t RL)
{
  oled_function_set(0, 0, 0);
  oled_characterization(0);
  i2c_write_command(0x18 | (SC << 3) | (RL << 2));
}

void oled_set_cgram_address(uint32_t AC)
{
  oled_function_set(0, 0, 0);
  oled_characterization(0);
  i2c_write_command(0x40 | AC);
}

void oled_set_ddram_address(uint32_t AC)
{
  oled_function_set(0, 0, 0);
  oled_characterization(0);
  i2c_write_command(0x80 | AC);
}

// IS=0, RE=1, SD=0

// IS=1, RE=1, SD=0

// IS=X, RE=1, SD=0

void oled_extended_function_set(uint32_t FW, uint32_t BW, uint32_t NW)
{
  oled_function_set(0, 1, 0);
  oled_characterization(0);
  i2c_write_command(0x08 | (FW << 2) | (BW << 1) | NW);
}

// Function Selection A
void oled_function_selection_a(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(0);
  i2c_write_command(0x71);
  i2c_write_data(A);
}

// Function Selection B
// op: CGROM, CGRAMのサイズ (1だと1だとCGRAMを8個使える)
// ro: character ROM (0=記号多め, 1=ヨーロッパ, 2=カタカナ)
void oled_function_selection_b(uint32_t RO, uint32_t OP)
{
  oled_function_set(0, 1, 0);
  oled_characterization(0);
  i2c_write_command(0x72);
  i2c_write_data((RO << 2) | OP);
}

// IS=X, RE=1, SD=1 (?)


void oled_set_contrast_control(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(1);
  i2c_write_command(0x81);
  i2c_write_command(A);
}

// Set Display Clock Divide Ratio / Oscillator Frequency
// A[3:0] Display Clock Divide ratio
// A[7:4] Oscillator Frequency
void oled_set_display_clock(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(1);
  i2c_write_command(0xd5);
  i2c_write_command(A);
}

// Set Phase Length
void oled_set_phase_length(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(1);
  i2c_write_command(0xd9);
  i2c_write_command(A);
}

// Set SEG Pins Hardware Configuration
void oled_seg_pins_hardware_configuration(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(1);
  i2c_write_command(0xda);
  i2c_write_command(A);
}

// Set VCOMH Deselect Level
void oled_set_vcomh_deselect_level(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(1);
  i2c_write_command(0xdb);
  i2c_write_command(A);
}

// Function Selection C
void oled_function_selection_c(uint32_t A)
{
  oled_function_set(0, 1, 0);
  oled_characterization(1);
  i2c_write_command(0xdc);
  i2c_write_command(A);
}

void oled_set_cgram_pattern(uint32_t code, const uint8_t *ptn)
{
  int i;
  
  oled_set_cgram_address(code << 3);
  for (i = 0; i < 8; i++) {
    i2c_write_data(ptn[i]);
  }
}


void oled_init(void)
{
  gpio_init();

  gpio_set(GPIO_VCC, 0);
  gpio_set(GPIO_VDD, 0);
  gpio_set(GPIO_SDA, 1);
  gpio_set(GPIO_SCL, 1);
  usleep(100);
  
  gpio_set(GPIO_VDD, 1);
  usleep(100);
  
  oled_function_selection_a(0);
  oled_display_on_off(0, 0, 0); // Display off
  oled_set_display_clock(0x70);
  oled_extended_function_set(0, 0, 0);
  oled_entry_mode_set(1, 0);
  oled_function_selection_b(2, 1);
  oled_seg_pins_hardware_configuration(0x10);
  oled_function_selection_c(0x80);
  oled_set_contrast_control(0x10);
  oled_set_phase_length(0x4f);
  oled_set_vcomh_deselect_level(0);
  oled_clear_display();
  oled_set_ddram_address(0);

  gpio_set(GPIO_VCC, 1);
    
  oled_display_on_off(1, 0, 0); // Display on
}

void OLED_Print(int x, int y, const char *str, int len)
{
  int i;
  oled_set_ddram_address(x + y * 0x40);
  for (i = 0; i < len; i++) {
    i2c_write_data(str[i]);
  }
}

///////////////////////////////////////////



void test(void)
{
  const char msg0[] = "Hello! \x06\x07\xc8\xba\xc0\xde\xb2\xbd\xb7";
  const char msg1[] = "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f";
  int i;
  printf("oled_clear_display()\n");
  oled_clear_display();
  printf("oled_set_ddram_address()\n");
  oled_set_ddram_address(0);
  for (i = 0; i < sizeof(msg0) - 1; i++) {
    printf("i2c_write_data()\n");
    i2c_write_data(msg0[i]);
  }
  oled_set_ddram_address(0x40);
  for (i = 0; i < sizeof(msg1) - 1; i++) {
    printf("i2c_write_data()\n");
    i2c_write_data(msg1[i]);
  }
}

// "0"〜"9"までのフォント
const uint8_t font[][8] = {
  {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e, 0x00},
  {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e, 0x00},
  {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f, 0x00},
  {0x1f, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0e, 0x00},
  {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02, 0x00},
  {0x1f, 0x10, 0x1e, 0x01, 0x01, 0x11, 0x0e, 0x00},
  {0x06, 0x08, 0x10, 0x1e, 0x11, 0x11, 0x0e, 0x00},
  {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08, 0x00},
  {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e, 0x00},
  {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x02, 0x0c, 0x00}
};

// 
void print_time(const char *now, const char *next, int frame)
{
  int i, j, k, n;
  uint8_t ptn[8];
  
  for (i = 0; i < 8; i++) {
    if (now[i] == next[i]) continue;
    j = 0;
    n = now[i] - '0';
    for (k = frame; k < 8; j++, k++) {
      ptn[j] = font[n][k];
    }
    n = next[i] - '0';
    for (k = 0; j < 8; j++, k++) {
      ptn[j] = font[n][k];
    }
    oled_set_cgram_pattern(i, ptn);
  }
}

void print_date(const char *date)
{
  OLED_Print(1, 1, date, 5);
}

void clocktest(void)
{
  struct timespec tv0, tv1;
  struct tm tm;
  long tv_sec_old = 0;
  long dt_nsec;
  char time_str_now[10];
  char time_str_next[10];
  char date_str[10];
  int frame = 0;

  OLED_Print(2, 0, "\xfb \xc8\xba\xc0\xde\xb2\xbd\xb7! \xfc", 12);
  OLED_Print(0, 1, " 00/00 \x00\x01:\x03\x04:\x06\x07 ", 16);
  
  while (1) {
    clock_gettime(CLOCK_REALTIME, &tv0);

    if (tv0.tv_sec != tv_sec_old) {
#ifdef BUSYLOOP_DEBUG
      printf("%6ld %6ld\n", t_min, t_max);
      t_min = 1000000000;
      t_max = 0;
#endif
      //printf("%ld\n", tv.tv_nsec);
      tv_sec_old = tv0.tv_sec;
      localtime_r(&tv0.tv_sec, &tm);
      strftime(date_str, sizeof(date_str), "%m/%d", &tm);
      print_date(date_str);
      
      strftime(time_str_now, sizeof(time_str_now), "%T", &tm);
      tv0.tv_sec++;
      localtime_r(&tv0.tv_sec, &tm);
      strftime(time_str_next, sizeof(time_str_next), "%T", &tm);
      // OLED_Print(8, 1, time_str_now, 8);
      frame = 0;
      print_time(time_str_now, "--:--:--", 0);
    } else {
      if (tv0.tv_nsec >= 700 * 1000 * 1000) {
        if (frame < 8) frame++;
        print_time(time_str_now, time_str_next, frame);
      }
    }

    // 表示更新周期が40msとなるようにウェイト挿入
    clock_gettime(CLOCK_REALTIME, &tv1);
    dt_nsec = 40 * 1000 * 1000 - (tv1.tv_nsec - tv0.tv_nsec);
    if (tv1.tv_sec > tv0.tv_sec) dt_nsec -= 1000 * 1000 * 1000;
    if (dt_nsec > 0) {
      usleep(dt_nsec / 1000);
    }
  }
}

int main(void)
{
  oled_init();
  // test();
  clocktest();

  return 0;
}

