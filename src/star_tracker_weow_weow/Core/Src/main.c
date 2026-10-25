/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ssd1306.h>
#include <ssd1306_fonts.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_RESET GPIO_PIN_12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//button flags

uint8_t reset_flag=0;


//oled
char myText[100] = ":(";
char retVal;
char msg[100];


//temperature
static const uint8_t TMP102_ADDR = 0x48 << 1; // Use 8-bit address
static const uint8_t REG_TEMP = 0x00;
HAL_StatusTypeDef ret; 	// to hold communication errors if such occur
uint8_t Temperature_buf[16]; 			// to hold the character string for UART communication or the bytes from the TMP102 chip
int16_t val; 				// to hold raw value from the TMP102 chip
float temp_c;

//magnetometer
float direction;
float base_direction;
int16_t magX;
int16_t magY;
int16_t magZ;


float degree_1;
float degree_2;
uint8_t read;


//GPS
uint8_t rx_byte;
char gps_buffer[128];
uint8_t gps_index = 0;
volatile uint8_t line_ready = 0;
char gps_line[128];

typedef struct {
    uint8_t fix;          // 1 = jest sygnał ('A'), 0 = brak ('V')
    uint8_t hours;        // Czas UTC
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    double latitude;      // Stopnie dziesiętne (np. 52.229770)
    double longitude;     // Stopnie dziesiętne (np. 21.012220)
} GPS_Data_t;

typedef struct {
    double latitude;      // Stopnie dziesiętne (np. 52.229770)
    double longitude;     // Stopnie dziesiętne (np. 21.012220)
} GPS_Location_t;

GPS_Data_t gps_data = {0};
GPS_Location_t location ={0};

// Konwersja formatu NMEA (DDMM.MMMM) na stopnie dziesiętne (DD.DDDDDD)
double NMEA_To_Decimal(double nmea_val, char quadrant)
{
    int degrees = (int)(nmea_val / 100);
    double minutes = nmea_val - (degrees * 100);
    double dec = degrees + (minutes / 60.0);

    if (quadrant == 'S' || quadrant == 'W') {
        dec = -dec;
    }
    return dec;
}

// Bezpieczne wyciąganie kolejnego pola między przecinkami (obsługuje puste pola ",,")
char* get_next_token(char **str)
{
    if (*str == NULL) return NULL;
    char *start = *str;
    char *end = strpbrk(start, ",*");
    if (end != NULL) {
        *end = '\0';
        *str = end + 1;
    } else {
        *str = NULL;
    }
    return start;
}

// Parser ramki $GNRMC / $GPRMC
void GPS_Parse_RMC(char *line, GPS_Data_t *gps)
{
    if (strncmp(line, "$GNRMC", 6) != 0 && strncmp(line, "$GPRMC", 6) != 0) {
        return; // Ignoruj inne ramki
    }

    char *rest = line;
    char *token;
    int field = 0;

    double raw_lat = 0.0, raw_lon = 0.0;
    char lat_dir = 'N', lon_dir = 'E';

    while ((token = get_next_token(&rest)) != NULL)
    {
        switch (field)
        {
            case 1: // Czas UTC: HHMMSS.sss (czas bywa dostępny jeszcze przed złapaniem fixa 3D!)
                if (strlen(token) >= 6) {
                    int t = atoi(token);
                    gps->hours   = (t / 10000);
                    gps->minutes = (t % 10000) / 100;
                    gps->seconds = (t % 100);
                }
                break;

            case 2: // Status: 'A' = Active, 'V' = Void
                gps->fix = (token[0] == 'A') ? 1 : 0;
                break;

            case 3: // Szerokość (DDMM.MMMM)
                if (strlen(token) > 0) raw_lat = atof(token);
                break;

            case 4: // Półkula N/S
                if (strlen(token) > 0) lat_dir = token[0];
                break;

            case 5: // Długość (DDDMM.MMMM)
                if (strlen(token) > 0) raw_lon = atof(token);
                break;

            case 6: // Półkula E/W
                if (strlen(token) > 0) lon_dir = token[0];
                break;

            case 9: // Data: DDMMYY
                if (strlen(token) == 6) {
                    int d = atoi(token);
                    gps->day   = (d / 10000);
                    gps->month = (d % 10000) / 100;
                    gps->year  = 2000 + (d % 100);
                }
                break;
        }
        field++;
    }

    if (gps->fix) {
        gps->latitude  = NMEA_To_Decimal(raw_lat, lat_dir);
        gps->longitude = NMEA_To_Decimal(raw_lon, lon_dir);
    }
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void servo_control(TIM_HandleTypeDef *htim, uint32_t Channel, float degree)
{
    uint16_t pulseWidth = (uint16_t)((degree * 150.0f / 180.0f) + 75.0f);
    __HAL_TIM_SetCompare(htim, Channel, pulseWidth);
}

float read_potentiometer(ADC_HandleTypeDef *hadc ){
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc,1000);

	return ((float)HAL_ADC_GetValue(hadc))/255.0f*180.0f;

}



void write_spi(uint8_t REG_ADRES, uint8_t msg){
	 uint8_t DATA[2];
	 DATA[0]= writeMask | REG_ADRES ;
	 DATA[1]=msg;
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Pull the chip select line low
	 HAL_SPI_Transmit(&hspi1, DATA, 2, 100); // Tell the ICM20948 what you want
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

void read_spi(void* rx_data_pointer, uint8_t REG_ADRES){

	   uint8_t tx_data = 0x80 |REG_ADRES; // Prepare the instruction " read register
  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Pull the chip select line low
  	   HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100); // Tell the ICM20948 what you want
  	   HAL_SPI_Receive(&hspi1, rx_data_pointer, 1, 100); // Get the answer
  	   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // Release the slave chip by bringing the line back up


}

void spi_slave0_write(uint8_t SLV_REG, uint8_t SLV_CMD){
	write_spi(ICM20648_REG_BANK_SEL, 0x30);
	write_spi(ICM20648_REG_I2C_SLV0_ADDR, 0x0C);
	write_spi(ICM20648_REG_I2C_SLV0_REG, SLV_REG);
	write_spi(ICM20648_REG_I2C_SLV0_DO, SLV_CMD);
	write_spi(ICM20648_REG_I2C_SLV0_CTRL, 0x81);
}

void spi_slave0_read( uint8_t SLV_REG, uint8_t len){
	write_spi(ICM20648_REG_BANK_SEL, 0x30);
	write_spi(ICM20648_REG_I2C_SLV0_ADDR, 0x8C);
	write_spi(ICM20648_REG_I2C_SLV0_REG, SLV_REG);
	write_spi(ICM20648_REG_I2C_SLV0_CTRL, 0x80|len);


}


float read_magnetometer(int16_t *magX,int16_t *magY,int16_t *magZ, uint8_t print)
{
	  write_spi(ICM20648_REG_BANK_SEL, 0x00); // Ensure Bank 0 is selected

	  uint8_t st1;
	  read_spi(&st1, 0x3B); // Read ST1 (Status 1) register

	  // Check if the "Data Ready" bit (bit 0) is set to 1
	  if (st1 & 0x01) {
		  uint8_t hxl, hxh, hyl, hyh, hzl, hzh, st2;

		  read_spi(&hxl, 0x3C); // X Low
		  read_spi(&hxh, 0x3D); // X High
		  read_spi(&hyl, 0x3E); // Y Low
		  read_spi(&hyh, 0x3F); // Y High
		  read_spi(&hzl, 0x40); // Z Low
		  read_spi(&hzh, 0x41); // Z High

		  // CRITICAL: You MUST read ST2 (0x42) at the end of every cycle.
		  // If you do not read ST2, the AK09916 locks its data registers and stops updating!
		  read_spi(&st2, 0x43);

		  // Reconstruct the 16-bit values correctly (Little-Endian)
		  *magX = (int16_t)((hxh << 8) | hxl);
		  *magY = (int16_t)((hyh << 8) | hyl);
		  *magZ = (int16_t)((hzh << 8) | hzl);


	  }


		// cound directions in radians
		float heading = atan2((float)*magY, (float)*magX);

		// radians to angles
		heading = heading * 180.0 / 3.14159265;

		// Normalisation
		if (heading < 0) {
			heading += 360.0;
		}

		if (print>0)
		{
		  char msg[50];
		  sprintf(msg, "X:%hd Y:%hd Z:%hd\r\n", *magX, *magY, *magZ);
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		  sprintf(msg, "direction %f\r\n", heading);
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		}

					// heading = 0 (lub 360) N
					// heading = 90 W
					// heading = 180 S
					// heading = 270 E
		return heading;
}




uint8_t read_gps(volatile uint8_t *line_ready,char *gps_line,GPS_Data_t *gps_data, uint8_t print)
		{
		if (*line_ready)
				     {
				         *line_ready = 0;

				         // Parsujemy tylko gdy przyszła ramka $GNRMC lub $GPRMC
				         if (strncmp(gps_line, "$GNRMC", 6) == 0 || strncmp(gps_line, "$GPRMC", 6) == 0)
				         {
				             GPS_Parse_RMC(gps_line, gps_data);
				         if (print){
				        	 char msg[100];
				             if (gps_data->fix)
				             {
				                 // Mamy sygnał z satelitów!
				                 snprintf(msg, sizeof(msg),
				                          " UTC: %02d:%02d:%02d | Data: %02d.%02d.%04d | Lat: %.6f, Lon: %.6f\r\n",
				                          gps_data->hours, gps_data->minutes, gps_data->seconds,
				                          gps_data->day,gps_data->month, gps_data->year,
				                          gps_data->latitude, gps_data->longitude);
				                 return 1;
				             }
				             else
				             {
				                 // Brak fixa (np. w budynku), ale czas z satelitów lub RTC modułu może już płynąć
				                 snprintf(msg, sizeof(msg),
				                          "Finding satelites... UTC: %02d:%02d:%02d\r\n",
				                          gps_data->hours, gps_data->minutes, gps_data->seconds);
				                 return 0;
				             }


				             HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
				         	 }
				         }
				     }else{
				    	 return 2;
				     }

		}


void reset(float *direction,GPS_Location_t *location,GPS_Data_t *gps_data ){

	float sum = 0.0f;
	for (int i = 0; i < 10; i++) {
	    sum += read_magnetometer(&magX, &magY, &magZ, 0);
	}
	*direction = sum / 10.0f;
    sprintf(msg, "direction %.1f\r\n", *direction);
    ssd1306_SetCursor(0, 5);
    retVal = ssd1306_WriteString(msg, Font_6x8, White);


    if (gps_data->fix)
                {snprintf(msg, sizeof(msg),
                        "UTC: %02d:%02d:%02d | Data: %02d.%02d.%04d | Lat: %.6f, Lon: %.6f | direction %.1f\r\n",
                        gps_data->hours, gps_data->minutes, gps_data->seconds,
                        gps_data->day, gps_data->month, gps_data->year,
                        gps_data->latitude, gps_data->longitude,*direction);
                HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
                    // Mamy sygnał z satelitów!
                    snprintf(msg, sizeof(msg),
                             "Lat:%.2f, Lon:%.2f\r\n",

                             gps_data->latitude, gps_data->longitude);
                }
                else
                {
                    // Brak fixa (np. w budynku), ale czas z satelitów lub RTC modułu może już płynąć
                	snprintf(msg, sizeof(msg),
                	                         "Finding satelites...  UTC: %02d:%02d:%02d | direction %.1f \r\n",
                	                         gps_data->hours, gps_data->minutes, gps_data->seconds,*direction);
                	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);

                    snprintf(msg, sizeof(msg),
                             "No satelites");
                }
    ssd1306_SetCursor(0, 15);
    retVal = ssd1306_WriteString(msg, Font_6x8, White);
    ssd1306_UpdateScreen();



}

void read_temp(uint8_t *Temperature_buf, HAL_StatusTypeDef *ret, int16_t *val,float *temp_c , uint8_t print){
// Tell TMP102 that we want to read from the temperature register
*Temperature_buf = REG_TEMP;
*ret = HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, Temperature_buf, 1, 200);
	if ( *ret != HAL_OK ) {
	  strcpy(Temperature_buf, "Error Tx\r\n");
	}
	else{
  // Read 2 bytes from the temperature register
  *ret = HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, Temperature_buf, 2, 200);
	  if ( *ret != HAL_OK ) {
		strcpy(Temperature_buf, "Error Rx\r\n");
	  } else {

		//Combine the bytes
		*val = ((int16_t)*Temperature_buf << 4) | (*(Temperature_buf+1) >> 4);

		// Convert to 2's complement, since temperature can be negative
		if ( *val > 0x7FF ) {
			*val |= 0xF000;
		}

		// Convert to float temperature value (Celsius)
		*temp_c = *val * 0.0625;

		// Convert temperature to decimal format
		*temp_c *= 100;
		sprintf((char*)Temperature_buf,"%u.%u C\r\n",
			  ((unsigned int)*temp_c / 100),
			  ((unsigned int)*temp_c % 100));
	  }

	}



if (print){
	HAL_UART_Transmit(&huart2, Temperature_buf, strlen((char*)Temperature_buf), 10);
}

}

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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // oled
  ssd1306_Init();
   ssd1306_SetCursor(0, 5);
   retVal = ssd1306_WriteString(myText, Font_6x8, White);
   ssd1306_UpdateScreen();

  //servos
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // Start the timer 3 for PWM on Channel 2 (PC7)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // Start the timer 2 for PWM on Channel 2 (PB10)

  //magnetometer
  do {
	  write_spi(ICM20648_REG_BANK_SEL, 0x00);
	   read_spi(&read,ICM20648_REG_WHO_AM_I);
     } while (read != 0xEA);
  	  write_spi(ICM20648_REG_PWR_MGMT_1, 0x01);
  	  read_spi(&read,ICM20648_REG_PWR_MGMT_1);// turn off sleep mode



  	  // magnetometer cobfiguration
     write_spi(ICM20648_REG_BANK_SEL, 0x00);
     write_spi(0x09, 0x01); // odr align
     HAL_Delay(5);
     write_spi(ICM20648_REG_USER_CTRL, 0x20);
     HAL_Delay(5);
     write_spi(ICM20648_REG_BANK_SEL, 0x30);
     HAL_Delay(5);
     write_spi(ICM20648_REG_I2C_MST_CTRL, 0x0D);
     HAL_Delay(5);
     write_spi(ICM20648_REG_LP_CONFIG, (1 << 6)); // i2c master in duty cycle mode
     HAL_Delay(5);
     write_spi(ICM20648_REG_I2C_MST_ODR_CONFIG, 0x03);    // odr to 137Hz
     HAL_Delay(10);



     spi_slave0_write(0x32, 0x01);// Write 0x01 to the magnetometer's CNTL3 register (0x32).
     HAL_Delay(20);
     spi_slave0_write(0x31, 0x08);// Write 0x08 to the magnetometer's CNTL2 register (0x31). This puts the magnetometer into Continuous Measurement Mode 4 (100 Hz update rate).
     HAL_Delay(20);
     spi_slave0_read(0x10, 0x0A);
     HAL_Delay(20);
     write_spi(ICM20648_REG_BANK_SEL, 0x00);


     reset(&direction,&location,&gps_data);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //servos
	  degree_1=read_potentiometer(&hadc1 );
	  degree_2=read_potentiometer(&hadc2 );
	  servo_control(&htim2,TIM_CHANNEL_3,degree_2);
	  servo_control(&htim3,TIM_CHANNEL_2,degree_1);
	  //sprintf(msg,"%f\r\n", degree_2);


	  // magnetometer
	 //direction = read_magnetometer(&magX, &magY, &magZ,  0);
	// gps
	  read_gps(&line_ready, gps_line ,&gps_data, 1);



		 // temperature
	  read_temp(Temperature_buf, &ret, &val,&temp_c,0);

	  if (reset_flag){
		  reset_flag=0;
		  reset(&direction,&location,&gps_data);




	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_8B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB11 PB12 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BUTTON_RESET) {
	    reset_flag = 1;
  }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        char c = (char)rx_byte;

        // Re-arm interrupt immediately (takes < 2 microseconds)
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

        if (gps_index < sizeof(gps_buffer) - 1)
        {
            gps_buffer[gps_index++] = c;
            if (c == '\n') // End of NMEA sentence
            {
                gps_buffer[gps_index] = '\0';
                strcpy(gps_line, gps_buffer);
                line_ready = 1;
                gps_index = 0;
            }
        }
        else
        {
            gps_index = 0; // Prevent buffer overflow
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
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
#ifdef USE_FULL_ASSERT
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
