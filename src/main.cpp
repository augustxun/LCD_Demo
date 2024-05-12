// =================================================
// * Recitation 6: Final Recitation - LCD Screen *
// =================================================

// TODOs:
// [1] Recap on Polling, Interrupts, Debouncing, Multithreading, etc.
// [2] LCD Screen - Reading the datasheet and writing the code to display text and shapes on to the Screen!  
// [3] Extra --> Graphs on Screen!, Semaphores, Introduction to DSP!
// [4] Embedded Challenge Tips

// Introduction to DSP and Filters
#include "mbed.h"  
#include "arm_math.h"  // 包含FFT库，用于进行频率分析
// #include "stm32f429i_discovery_lcd.h"  // 包含LCD显示库，用于显示屏设置
#include <drivers/LCD_DISCO_F429ZI.h>

#define WINDOW_SIZE 256  // Window size

// Define Regs & Configurations --> Gyroscope's settings
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23 // Second configure to set the DPS // page 33
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

#define CTRL_REG3 0x22 // page 32
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

#define OUT_X_L 0x28

#define SPI_FLAG 1  // SPI完成的FLAG
#define DATA_READY_FLAG 2  // 数据准备好的FLAG

// #define SCALING_FACTOR ()  // 陀螺仪灵敏度设置

// 转换为弧度/秒：
#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

// #define FILTER_COEFFICIENT 0.1f // Adjust this value as needed

// 定义震颤检测的频率范围3-6和阈值, 该阈值用于消抖 (阈值先设为1.0f,可以根据需求和实际数据更改)
#define TREMOR_FREQ_LOW 3
#define TREMOR_FREQ_HIGH 6
#define THRESHOLD 1.0f

// EventFlags object declaration
EventFlags flags;

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);  // 初始化SPI接口

float32_t fft_input_x[WINDOW_SIZE * 2];  // 定义x轴FFT输入数组
float32_t fft_input_y[WINDOW_SIZE * 2];  // 定义y轴FFT输入数组
float32_t fft_input_z[WINDOW_SIZE * 2];  // 定义z轴FFT输入数组
float32_t fft_output_x[WINDOW_SIZE];  // 定义x轴FFT输出数组
float32_t fft_output_y[WINDOW_SIZE];  // 定义y轴FFT输出数组
float32_t fft_output_z[WINDOW_SIZE];  // 定义z轴FFT输出数组
arm_rfft_fast_instance_f32 s;  // FFT实例

// spi callback function
void spi_cb(int event) {
    flags.set(SPI_FLAG);  // 设置SPI完成的flag
}

// data ready callback function
void data_ready_cb() {
    flags.set(DATA_READY_FLAG);  // 设置数据准备好的flag
}

void init_fft() {
    arm_rfft_fast_init_f32(&s, WINDOW_SIZE);  // 初始化FFT
}

void init_lcd() {
    BSP_LCD_Init();  // 初始化LCD
    BSP_LCD_LayerDefaultInit(1, LCD_FRAME_BUFFER);  // 初始化LCD层
    BSP_LCD_SelectLayer(1);  // 选择LCD层
    BSP_LCD_DisplayOn();  // 打开显示
    BSP_LCD_Clear(LCD_COLOR_WHITE);  // 清屏为白色
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);  // 设置白色背景色
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);  // 设置黑色文字色
    BSP_LCD_DisplayStringAt(0, LINE(0), (uint8_t *)"Tremor Detection Active", CENTER_MODE);  // 显示文字(Active)
}

void setup_spi_and_gyro() {
    spi.format(8, 3);  // SPI格式
    spi.frequency(1000000);  // SPI频率
    
    uint8_t write_buf[32], read_buf[32];  // 缓冲区
    write_buf[0] = CTRL_REG1;  // 设置寄存器地址
    write_buf[1] = CTRL_REG1_CONFIG;  // 设置寄存器配置
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);  // 写入配置
    
    write_buf[0] = CTRL_REG4;  // 设置寄存器地址
    write_buf[1] = CTRL_REG4_CONFIG;  // 设置寄存器配置
    spi.transfer(write_buf, 2, read_buf, 2,spi_cb);  // 写入配置
    
    write_buf[0] = CTRL_REG3;  // 设置寄存器地址
    write_buf[1] = CTRL_REG3_CONFIG;  // 设置寄存器配置
    spi.transfer(write_buf, 2, read_buf, 2,spi_cb);  // 写入配置
    
    InterruptIn data_ready(PA_2);  // 接收来自陀螺仪的数据就绪sign
    data_ready.rise(&data_ready_cb);  // 设置中断触发函数
}

/* //这是只分析x轴的分析方法
void analyze_tremor(float32_t* fft_output, float& tremor_strength, bool& tremor_detected) {
    tremor_strength = 0.0f;  // 初始化震颤强度为0
    tremor_detected = false;  // 初始化震颤检测状态为否
    for (int i = TREMOR_FREQ_LOW; i <= TREMOR_FREQ_HIGH; i++) {
        float strength = sqrtf(fft_output[i] * fft_output[i]);  // 计算频率的强度
        if (strength > THRESHOLD) {  // 检查强度是否超过阈值 (去抖)
            tremor_detected = true;  // 超过则检测到震颤，改震颤检测状态为是
            tremor_strength = max(tremor_strength, strength);  // 更新最大震颤强度
        }
    }
}
*/

// 这是综合分析三个轴的分析方法
void analyze_tremor(float32_t* fft_output_x, float32_t* fft_output_y, float32_t* fft_output_z, float& tremor_strength, bool& tremor_detected) {
    tremor_strength = 0.0f; // 初始化震颤强度为0
    tremor_detected = false; // 初始化震颤检测状态为否
    for (int i = TREMOR_FREQ_LOW; i <= TREMOR_FREQ_HIGH; i++) {
        float strength_x = sqrtf(fft_output_x[i] * fft_output_x[i]); // 计算x轴频率的强度
        float strength_y = sqrtf(fft_output_y[i] * fft_output_y[i]); // 计算y轴频率的强度
        float strength_z = sqrtf(fft_output_z[i] * fft_output_z[i]); // 计算z轴频率的强度
        float combined_strength = (strength_x + strength_y + strength_z) / 3;  // 使用平均值综合三个轴的强度
        
        if (combined_strength > THRESHOLD) { // 检查强度是否超过阈值 (去抖)
            tremor_detected = true; // 超过则检测到震颤，改震颤检测状态为是
            tremor_strength = max(tremor_strength, combined_strength); // 更新最大震颤强度
        }
    }
}

// 更新显示内容的函数
void update_lcd_display(bool tremor_detected, float tremor_strength) {
    if (tremor_detected) {
        BSP_LCD_Clear(LCD_COLOR_RED);  // 如果检测到震颤，将背景设为红色
        BSP_LCD_SetTextColor(LCD_COLOR_WHITE);  // 文字颜色设为白色
        BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"Tremor Detected!", CENTER_MODE);  // 显示检测到震颤
        char buf[32];
        sprintf(buf, "Strength: %.2f Hz", tremor_strength);  // 震颤强度保留至小数点后两位输出
        BSP_LCD_DisplayStringAt(0, LINE(6), (uint8_t *)buf, CENTER_MODE);  // 显示震颤强度文字
    } else {
        BSP_LCD_Clear(LCD_COLOR_GREEN);  // 如果未检测到震颤，则将背景设为绿色
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);  // 文字颜色设为黑色
        BSP_LCD_DisplayStringAt(0, LINE(5), (uint8_t *)"No Tremor", CENTER_MODE);  // 显示未检测到震颤
    }
}

int main() {
    init_lcd();  // 初始化LCD显示
    init_fft();  // 初始化FFT
    setup_spi_and_gyro();  // 设置SPI和陀螺仪

    while (1) {
        flags.wait_all(DATA_READY_FLAG);  // 等待数据准备好
        memset(fft_input_x, 0, sizeof(fft_input_x));  // 清空x轴FFT输入数组
        memset(fft_input_y, 0, sizeof(fft_input_y));  // 清空y轴FFT输入数组
        memset(fft_input_z, 0, sizeof(fft_input_z));  // 清空z轴FFT输入数组
        
        for (int i = 0; i < WINDOW_SIZE; i++) {
            flags.wait_all(SPI_FLAG);  // 等待SPI完成
            uint8_t write_buf[1] = {OUT_X_L | 0x80}, read_buf[6];  // 设置读取命令和缓冲区
            spi.transfer(write_buf, 1, read_buf, 6, spi_cb);  // 读取陀螺仪数据

            int16_t raw_x = (read_buf[1] << 8) | read_buf[0];  // 处理x轴数据
            int16_t raw_y = (read_buf[3] << 8) | read_buf[2];  // 处理y轴数据
            int16_t raw_z = (read_buf[5] << 8) | read_buf[4];  // 处理z轴数据

            fft_input_x[2 * i] = (float)raw_x * SCALING_FACTOR;  // 存储处理后的x轴数据
            fft_input_y[2 * i] = (float)raw_y * SCALING_FACTOR;  // 存储处理后的y轴数据
            fft_input_z[2 * i] = (float)raw_z * SCALING_FACTOR;  // 存储处理后的z轴数据
        }

        arm_rfft_fast_f32(&s, fft_input_x, fft_output_x, 0);  // 对x轴数据进行FFT
        arm_rfft_fast_f32(&s, fft_input_y, fft_output_y, 0);  // 对y轴数据进行FFT
        arm_rfft_fast_f32(&s, fft_input_z, fft_output_z, 0);  // 对z轴数据进行FFT

        float tremor_strength;
        bool tremor_detected;
        //analyze_tremor(fft_output_x, tremor_strength, tremor_detected); // 这是只分析x轴的分析方法
        analyze_tremor(fft_output_x, fft_output_y, fft_output_z, tremor_strength, tremor_detected);  // 调用函数综合分析三个轴的震颤
        update_lcd_display(tremor_detected, tremor_strength);  // 调用函数更新显示

        ThisThread::sleep_for(1000);  // 每秒更新一次
    }
}
