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
#define FILTER_COEFFICIENT 0.1f // Adjust this value as needed

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
void data_cb() {
    flags.set(DATA_READY_FLAG);  // 设置数据准备好的flag
}

void init_fft() {
    arm_rfft_fast_init_f32(&s, WINDOW_SIZE);  // 初始化FFT
}

void init_lcd() {
   LCD_DISCO_F429ZI lcd;  // 初始化LCD
   lcd.SetBackColor(LCD_COLOR_WHITE);  // 设置白色背景色
   lcd.SetTextColor(LCD_COLOR_BLACK);  // 设置黑色文字色
   lcd.DisplayStringAt(0, LINE(3), (uint8_t *)"Detection Active", CENTER_MODE);  // 显示文字(Active)
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
        printf("tremor:%4.5f\n",combined_strength);
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
    init_lcd();
    //spi initialization
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];

    //interrupt initialization
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);
    
    //spi format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Write to control registers --> spi transfer
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[1] = 0xFF;

    //(polling for\setting) data ready flag
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
        flags.set(DATA_READY_FLAG);
    }

    // init_lcd();  // 初始化LCD显示
    // init_fft();  // 初始化FFT

    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f;
    int cycle=0;
    while (1) {
        int16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        flags.wait_all(DATA_READY_FLAG);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Process raw data
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = ((float)raw_gx) * SCALING_FACTOR;
        gy = ((float)raw_gy) * SCALING_FACTOR;
        gz = ((float)raw_gz) * SCALING_FACTOR;

        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;

        // printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d\t\n", raw_gx, raw_gy, raw_gz);

        printf(">x_axis_low:%4.5f\n", filtered_gx);
        printf(">y_axis_low:%4.5f\n", filtered_gy);
        printf(">z_axis_low:%4.5f\n", filtered_gz);
        printf("%d\n", cycle);

        fft_input_x[2 * cycle] = (float)filtered_gx * SCALING_FACTOR;  // 存储处理后的x轴数据
        fft_input_y[2 * cycle] = (float)filtered_gy * SCALING_FACTOR;  // 存储处理后的y轴数据
        fft_input_z[2 * cycle] = (float)filtered_gz * SCALING_FACTOR;  // 存储处理后的z轴数据
        
        if (cycle==255) {
            printf("finish reading");
            // arm_rfft_fast_f32(&s, fft_input_x, fft_output_x, 0);  // 对x轴数据进行FFT
            // arm_rfft_fast_f32(&s, fft_input_y, fft_output_y, 0);  // 对y轴数据进行FFT
            // arm_rfft_fast_f32(&s, fft_input_z, fft_output_z, 0);  // 对z轴数据进行FFT

            // float tremor_strength;
            // bool tremor_detected;
            // //analyze_tremor(fft_output_x, tremor_strength, tremor_detected); // 这是只分析x轴的分析方法
            // analyze_tremor(fft_output_x, fft_output_y, fft_output_z, tremor_strength, tremor_detected);  // 调用函数综合分析三个轴的震颤
            // update_lcd_display(tremor_detected, tremor_strength);  // 调用函数更新显示

            // ThisThread::sleep_for(1000);  // 每秒更新一次
            cycle = 0;
            continue;
        }
        cycle++;
    }
}
