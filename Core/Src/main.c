/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Define necessary variables */
char inputPin[5] = "";
char correctPin[5] = "1234";
int pinAttempt = 0;
int incorrectAttempts = 0;
int threshold = 3;
int fireDetected = 0;
int motionDetected = 0;
int motionThreshold = 3;
int servoPosition = 0;
int abnormalSituation = 0;
unsigned long motionStartTime = 0;
#define MOTION_DETECTION_THRESHOLD 3     // Threshold for motion detection (e.g., distance)
#define ALERT_DURATION 25000             // Alert duration for abnormal situation (25 seconds)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Keypad_Init(void);
void Ultrasonic_Init(void);
void Temperature_Sensor_Init(void);
void Servo_Init(void);
void TriggerBuzzer(int times, int duration);
void FireDetection(void);
void MotionDetection(void);
void PasswordModule(void);
void TriggerAlert(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Keypad_Init(void) {
    // Initialize Keypad rows (PB4 to PB7)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
}

void Ultrasonic_Init(void) {
    // Initialize Ultrasonic sensor (Trigger on PB0, Echo on PB1)
    // Placeholder initialization for Ultrasonic sensor
}

void Temperature_Sensor_Init(void) {
    // Initialize Temperature sensor (Analog input on PB9)
    // Placeholder initialization for Temperature sensor
}

// In your initialization function
void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB clock

    // Configure the pin for fire detection (e.g., PB9)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on rising edge (or change to FALLING)
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Enable the interrupt in NVIC
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}


void Servo_Init(void) {
    // Enable the TIM1 peripheral clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Set the prescaler and auto-reload value for PWM frequency
    TIM1->PSC = 71;          // Prescaler (72 MHz / (71+1) = 1 MHz)
    TIM1->ARR = 1000 - 1;    // Auto-reload value for 1 kHz (1 MHz / 1000 = 1 kHz)

    // Configure the PWM mode in CCMR1 (PWM mode 1)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;               // Clear OC1M bits
    TIM1->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);   // Set OC1M to 0110 (PWM mode 1)

    // Set the compare value for 50% duty cycle (CCR1 = 500)
    TIM1->CCR1 = 500;  // Duty cycle = CCR1 / ARR = 500 / 1000 = 50%

    // Enable PWM output on channel 1 (PA8)
    TIM1->CCER |= TIM_CCER_CC1E;

    // Enable the timer to start generating PWM
    TIM1->CR1 |= TIM_CR1_CEN;
}

void TriggerBuzzer(int times, int duration) {
    for (int i = 0; i < times; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); // Turn on buzzer
        HAL_Delay(duration);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Turn off buzzer
        HAL_Delay(100); // Short delay between buzzes
    }
}

void FireDetection(void) {
    // Example: when fire is detected
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) > 50) {
        if (!fireDetected) {
            fireDetected = 1;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  // Turn on buzzer
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // Turn on LED
            // Disable other operations as needed
        }
    } else {
        // Clear the fire condition and re-enable other processes
        if (fireDetected) {
            fireDetected = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  // Turn off buzzer
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Turn off LED
            __enable_irq(); // Re-enable interrupts for other system operations
        }
    }
}

void MotionDetection(void) {
    // Monitor the ultrasonic sensor to detect motion
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) < MOTION_DETECTION_THRESHOLD) {
        if (!motionDetected) {
            motionDetected = 1;
            motionStartTime = HAL_GetTick();  // Get the current timestamp
        }
    } else {
        motionDetected = 0;
    }

    // If object has been detected for more than ALERT_DURATION (25 seconds), trigger alert
    if (motionDetected && (HAL_GetTick() - motionStartTime) > ALERT_DURATION) {
        if (!abnormalSituation) {
            abnormalSituation = 1;  // Mark abnormal situation
            TriggerAlert();
        }
    }

    // If no object is detected, reset the situation to normal
    if (!motionDetected && abnormalSituation) {
        abnormalSituation = 0;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Turn off LED
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  // Turn off buzzer
    }
}

void TriggerAlert(void) {
    // Trigger alerts: LED blinks, buzzer sounds, and abnormal situation message displayed

    // LED blinking
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // Turn on LED
        HAL_Delay(500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Turn off LED
        HAL_Delay(500);
    }

    // Buzzer sounds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  // Turn on buzzer
    HAL_Delay(3000);  // Buzzer on for 3 seconds
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  // Turn off buzzer
}

void PasswordModule(void) {
    // Handle user input for PIN verification
    char key = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);  // Example for keypad row 1 reading (PB4)

    // Process keypad input
    if (key == '#') {
        if (strcmp(inputPin, correctPin) == 0) {
            TriggerBuzzer(2, 500); // Correct password: buzzer triggers twice
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  // Servo position to 90 degrees
            HAL_Delay(5000);  // 5 seconds delay
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  // Servo position back to 0 degrees

            // Reset incorrect attempts on successful password entry
            incorrectAttempts = 0;
        } else {
            // Increment incorrect attempts
            incorrectAttempts++;
            // Check for lockout condition
              if (incorrectAttempts >= threshold) {
                  static unsigned long lockStartTime = 0;

                  if (lockStartTime == 0) {
                      lockStartTime = HAL_GetTick(); // Record lockout start time
                  }

                  // Check if 1 minute has passed since lockout started
                  if (HAL_GetTick() - lockStartTime < 60000) {  // 60,000 ms = 1 minute
                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);  // Turn on LED to indicate lock
                      return;  // Exit without processing further
                  } else {
                      // Reset lockout after 1 minute
                      incorrectAttempts = 0;
                      lockStartTime = 0;
                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);  // Turn off LED
                  }
              }
            TriggerBuzzer(1, 100);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);  // Buzzer ON continuously
            HAL_Delay(5000);  // Buzzer ON for 5 seconds
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);  // Buzzer OFF
        }
        memset(inputPin, 0, sizeof(inputPin));
        pinAttempt = 0;
    } else if (key >= '0' && key <= '9') {
        if (pinAttempt < 4) {  // Ensure PIN doesn't exceed length
            inputPin[pinAttempt++] = key;
            TriggerBuzzer(1, 100); // Trigger buzzer once for key press
        }
    } else if (key == '*') {
        memset(inputPin, 0, sizeof(inputPin));
        pinAttempt = 0;
    }
}

int main(void) {
    // Initialization functions
    HAL_Init();
    SystemClock_Config();
    Keypad_Init();
    Ultrasonic_Init();
    Temperature_Sensor_Init();
    Servo_Init();
    GPIO_Init(); // Initialize GPIO for interrupt

    while (1) {
        if (!fireDetected) {
            // Only run other tasks when there’s no fire detected
            MotionDetection();
            PasswordModule();
        }
        // Fire detection is handled by interrupt
    }
}

void EXTI9_5_IRQHandler(void)
{
    // Check if the interrupt was triggered by GPIO_PIN_9 (the fire detection pin)
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);  // Clear the interrupt flag

        // Trigger fire alarm
        FireDetection(); // Call your FireDetection function to handle the fire detection

        // Disable other system processes until fire condition is cleared
        __disable_irq();  // Disable global interrupts
        // Optionally stop other functionalities (motion, password, etc.)

        // You can implement a timeout here or a condition to reset the interrupt
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */

void Error_Handler(void) // blink an LED to indicate that an error has occurred
{
    // Disable interrupts to prevent further issues
    __disable_irq();

    // Optionally blink an LED on error (PA6 for an LED)
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);  // Toggle LED
        HAL_Delay(500);  // Delay for 500ms
    }
}


