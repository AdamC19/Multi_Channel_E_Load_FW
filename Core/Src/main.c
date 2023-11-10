/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "ads1120.h"
#include "printf.h"
#include "LCD_Config.h"
#include "LCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_INIT_CONFIG   ((ADS1120_MUX_P0_N1 << 24) | (ADS1120_DR_6 << 16) | (ADS1120_VREF_EXT << 8) | (0))
#define VOLT_SNS_MUX_SET  (ADS1120_MUX_P0_N1)
#define CURR_SNS_MUX_SET  (ADS1120_MUX_P2_N3)
#define VIN_DIV           20.0
#define R_SHUNT           0.005
#define V_SHUNT_GAIN      34.0
#define V_CS_GAIN         (33.0/43.0)
#define VIN_MIN           0.5
#define VIN_ENAB_CHANS_HYST 0.1

#define CHAN_COUNT        8
#define DAC_ADDR          (0x0D)
#define DAC_CURR_PER_LSB  0.018975
#define DAC_SETPT_UPDATE_INTERVAL   50000

#define DIR_A_SIGN        (1)
#define DIR_B_SIGN        (0 - DIR_A_SIGN)
#define MAX_CURR_DELTA_MA (500)
#define MIN_DELTA_CURR    (10)
#define CURR_SETPT_MAX    (150.0)

#define I_ERR_ALLOWED     (0.05)

#define LCD_BUF_SIZE          32
#define UPDATE_LCD_INTERVAL   100000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP(x, min, max)          (x > max ? max : (x < min ? min : x))
#define ADC_VOLTS_TO_CURRENT(vadc)  (vadc/(V_CS_GAIN*V_SHUNT_GAIN*R_SHUNT))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint64_t micros = 0;
Ads1120_t adc;
volatile bool poll_adc = false;
int channels_enabled = 0;
volatile int cs_mux_setting = 0;
float curr_samples[CHAN_COUNT];
float curr_sum = 0.0;
volatile uint8_t output_state_req = 0;
uint8_t output_state = 0;
volatile uint64_t enc_edge_ts = 0;
volatile float current_setpt = 0.0;
volatile float current_meas = 0.0;
volatile float vin_meas = 0.0;
volatile int channels_on = 0;
volatile int enc_counts = 0;
volatile bool dac_update_done = false;
volatile int dac_ind = 0;
uint64_t dac_setpt_update_ts = 0;
volatile uint16_t dac_setpts[CHAN_COUNT];
volatile bool dac_update[CHAN_COUNT];
volatile uint64_t i2c_tx_timestamp = 0;
uint64_t update_lcd_ts = 0;
uint8_t lcd_scratch_buf[LCD_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int nabs(int num);
void enable_output();
void hard_disable_output();
void enable_channel(int ind);
void disable_channel(int ind);
void enable_disable_channels(int chans);
void set_cs_mux(int mux);
void update_lcd(int line);
bool begin_update_dacs();
void write_dac(int dac_ind, uint16_t data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  adc.cs_port = ADC_CSB_GPIO_Port;
  adc.cs_pin = ADC_CSB_Pin;
  adc.hspi = &hspi1;
  adc.vref = 2.5;
  ads_reset(&adc);
  HAL_Delay(10);
  ads_begin(&adc, ADC_INIT_CONFIG);
  ads_set_input_mux(&adc, VOLT_SNS_MUX_SET);
  ads_start_sync(&adc);

  LCD_Init();
  LCD_Puts(0, 0, "Booting...");

  enable_disable_channels(0); // disable all channels to begin with

  HAL_TIM_Base_Start_IT(&htim1); // start microsecond timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // If requested output state has changed from the actual state
    if (output_state_req && !output_state){
      if (vin_meas > VIN_MIN + VIN_ENAB_CHANS_HYST){
        enable_output();
      }
    } else if (!output_state_req && output_state){
      hard_disable_output();
    }
    
    // things to do when output is enabled
    if (output_state){
      // If VIN has drooped too low, hard-disable the output
      if(vin_meas < VIN_MIN - VIN_ENAB_CHANS_HYST){
        hard_disable_output(); // hard disables the output
        // as long as output_state_req is TRUE, we will try to turn back on
        // once vin_meas goes above the enable threshold
      } else if (micros - dac_setpt_update_ts > DAC_SETPT_UPDATE_INTERVAL) {
        float curr_err = current_setpt - current_meas;
        if (fabs(curr_err) > I_ERR_ALLOWED){
          // calculate how many LSBs of change we need
          int dac_lsbs = (int)((curr_err / DAC_CURR_PER_LSB) + 0.5);
          int16_t lsbs_per_dac = dac_lsbs / CHAN_COUNT;
          int extra_lsbs = nabs(dac_lsbs) % CHAN_COUNT;
          for (int i = 0; i < CHAN_COUNT; i++){
            int16_t setpt = (int16_t)dac_setpts[i];
            setpt += lsbs_per_dac;
            if (i < extra_lsbs){
              setpt += (curr_err >= 0.0 ? 1 : -1);
            }
            if (setpt > 1023){
              dac_setpts[i] = 1023;
            } else if (setpt < 0){
              setpt = 0;
            }
            dac_setpts[i] = (uint16_t)setpt;
          }
          int timeout = 0;
          while (!begin_update_dacs() && timeout < 100){
            timeout++;
            HAL_Delay(1);
          }
        }
        dac_setpt_update_ts = micros;
      }
    }

    

    if (micros - i2c_tx_timestamp > 100000){
      // if we're hung up in some bad state, abort
      if (HAL_I2C_GetState(&hi2c1) != HAL_OK){
        HAL_I2C_Master_Abort_IT(&hi2c1, DAC_ADDR << 1);
      }
    }
    if (micros - update_lcd_ts > UPDATE_LCD_INTERVAL){
      update_lcd(0);
      update_lcd(1);
      update_lcd(2);
      update_lcd(3);
      update_lcd_ts = micros;
    }

    if (poll_adc){
      float adc_volts = ads_get_voltage(&adc);
      if (adc.mux_setting == VOLT_SNS_MUX_SET){
        // this is our voltage sensing
        vin_meas = VIN_DIV * adc_volts;
        ads_set_input_mux(&adc, CURR_SNS_MUX_SET);
        ads_start_sync(&adc);
      }else{
        curr_samples[cs_mux_setting] = ADC_VOLTS_TO_CURRENT(adc_volts);
        curr_sum += curr_samples[cs_mux_setting];
        cs_mux_setting++;
        if(cs_mux_setting >= CHAN_COUNT){
          current_meas = curr_sum;
          curr_sum = 0.0;
          cs_mux_setting = 0;
          ads_set_input_mux(&adc, VOLT_SNS_MUX_SET);
        }
        set_cs_mux(cs_mux_setting);
        ads_start_sync(&adc);
      }
      poll_adc = false;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


int nabs(int num){
  if (num < 0){
    return 0 - num;
  } else {
    return num;
  }
}


void enable_output(){
  for(int i = 0; i < CHAN_COUNT; i++){
    enable_channel(i);
    dac_setpts[i] = 0;
  }
  output_state = 1;
  begin_update_dacs();
}


/**
 * Hard-disable each load channel
*/
void hard_disable_output(){
  for(int i = 0; i < CHAN_COUNT; i++){
    disable_channel(i);
    dac_setpts[i] = 0;
  }
  output_state = 0;
}


void enable_channel(int ind){
  channels_enabled |= (1 << ind);
  enable_disable_channels(channels_enabled);
}


void disable_channel(int ind){
  channels_enabled &= ~(1 << ind);
  enable_disable_channels(channels_enabled);
}

/**
 * Powers on or off channels flagged by bits in the argument
*/
void enable_disable_channels(int chans){
  HAL_GPIO_WritePin(EN_0_GPIO_Port, EN_0_Pin, chans & 1);
  HAL_GPIO_WritePin(EN_1_GPIO_Port, EN_1_Pin, (chans >> 1) & 1);
  HAL_GPIO_WritePin(EN_2_GPIO_Port, EN_2_Pin, (chans >> 2) & 1);
  HAL_GPIO_WritePin(EN_3_GPIO_Port, EN_3_Pin, (chans >> 3) & 1);
  HAL_GPIO_WritePin(EN_4_GPIO_Port, EN_4_Pin, (chans >> 4) & 1);
  HAL_GPIO_WritePin(EN_5_GPIO_Port, EN_5_Pin, (chans >> 5) & 1);
  HAL_GPIO_WritePin(EN_6_GPIO_Port, EN_6_Pin, (chans >> 6) & 1);
  HAL_GPIO_WritePin(EN_7_GPIO_Port, EN_7_Pin, (chans >> 7) & 1);
}


void set_cs_mux(int mux){
  HAL_GPIO_WritePin(CS_MUX_SEL_0_GPIO_Port, CS_MUX_SEL_0_Pin, mux & 1);
  HAL_GPIO_WritePin(CS_MUX_SEL_1_GPIO_Port, CS_MUX_SEL_1_Pin, (mux >> 1) & 1);
  HAL_GPIO_WritePin(CS_MUX_SEL_2_GPIO_Port, CS_MUX_SEL_2_Pin, (mux >> 2) & 1);
}


void update_lcd(int line){
  if (line == 0){
    if (output_state){
      snprintf(lcd_scratch_buf, LCD_BUF_SIZE, "Imeas:%5.2f A | ON", current_meas);
    } else {
      snprintf(lcd_scratch_buf, LCD_BUF_SIZE, "Imeas:%5.2f A | OFF", current_meas);
    }
    LCD_Puts(0, 0, lcd_scratch_buf);
  } else if (line == 1){
    snprintf(lcd_scratch_buf, LCD_BUF_SIZE, "Iset :%5.2f A | %d", current_setpt, channels_on);
    LCD_Puts(0, 1, lcd_scratch_buf);
  } else if (line == 2){
    snprintf(lcd_scratch_buf, LCD_BUF_SIZE, "Vin  :%5.2f V", vin_meas);
    LCD_Puts(0, 1, lcd_scratch_buf);
  } else if (line == 3){
    snprintf(lcd_scratch_buf, LCD_BUF_SIZE, "Power:%5.2f W", vin_meas * current_meas);
    LCD_Puts(0, 1, lcd_scratch_buf);
  } 
}

/**
 * Set-off the update of all DACs
*/
bool begin_update_dacs(){
  dac_ind = 0;
  if (!dac_update_done){
    // we're in the middle of updating the DACs, don't do anything
    return false;
  }
  dac_update_done = false;
  write_dac(dac_ind, dac_setpts[dac_ind]);
  return true;
}

/**
 * Write to a single DAC at the specified dac_ind
*/
void write_dac(int dac_ind, uint16_t data){
  HAL_GPIO_WritePin(I2C_MUX_SEL0_GPIO_Port, I2C_MUX_SEL0_Pin, dac_ind & 1);
  HAL_GPIO_WritePin(I2C_MUX_SEL1_GPIO_Port, I2C_MUX_SEL1_Pin, (dac_ind >> 1) & 1);
  HAL_GPIO_WritePin(I2C_MUX_SEL2_GPIO_Port, I2C_MUX_SEL2_Pin, (dac_ind >> 2) & 1);

  uint8_t bytes[2] = {0};
  bytes[0] = (data >> 8) & 0xFF;
  bytes[1] = data & 0xFF;
  i2c_tx_timestamp = micros;
  HAL_I2C_Master_Transmit_IT(&hi2c1, DAC_ADDR << 1, bytes, 2);
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
  dac_ind++;
  if (dac_ind < CHAN_COUNT){
    write_dac(dac_ind, dac_setpts[dac_ind]);
  }else{
    dac_update_done = true;
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c){
  HAL_I2C_MasterTxCpltCallback(hi2c);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == ENC_A_Pin || GPIO_Pin == ENC_B_Pin){
    // ENCODER THINGS
    int dir = 0;
    if (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) && !HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin)){
      // direction is A
      dir = DIR_A_SIGN;
    } else if ( HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) && !HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin)){
      // direction is B
      dir = DIR_B_SIGN;
    } else {
      // something ain't right
      return;
    }
    if (micros < enc_edge_ts){
      // micros has wrapped around
      enc_edge_ts = 0;
    }
    int64_t pulse_period_raw = micros - enc_edge_ts;

    int32_t pulse_period = 1;
    if (pulse_period_raw > INT32_MAX){
      pulse_period = INT32_MAX;
    }
    int32_t delta_curr = 5000000 / pulse_period;

    if (delta_curr < MIN_DELTA_CURR){
      delta_curr = MIN_DELTA_CURR;
    } else if (delta_curr > MAX_CURR_DELTA_MA){
      delta_curr = MAX_CURR_DELTA_MA;
    }

    if (dir < 0){
      current_setpt -= delta_curr/1000.0;
    }else{
      current_setpt += delta_curr/1000.0;
    }
    current_setpt = CLAMP(current_setpt, 0.0, CURR_SETPT_MAX);
    
    enc_edge_ts = micros;
  } else if (GPIO_Pin == ENC_BTN_Pin){
    // ENC BUTTON THINGS
    output_state_req = !output_state_req;
  } else if (GPIO_Pin == ON_OFF_BTN_Pin){
    // ON-OFF BUTTON THINGS
  } else if (GPIO_Pin == ADC_DRDYB_Pin){
    // ADC THINGS
    poll_adc = true;
  } else if (GPIO_Pin == TRIG_Pin){
    // TRIG BUTTON THINGS
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  micros += 10;
} 

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
