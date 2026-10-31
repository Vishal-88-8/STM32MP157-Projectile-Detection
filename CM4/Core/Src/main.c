/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Cortex-M4 firmware - IR trigger capture, flash/hold-out
  *                   pulses, dead-time gating, shared ring buffer
  ******************************************************************************
  * Flow on a valid trigger edge (TIM5 input capture ISR):
  *   1. dead-time check (ignore if < 1 ms since last accepted trigger)
  *   2. raise FLASH_OUT (PE2) and, if enabled, HOLD-OUT pin
  *   3. arm TIM5 compare CH3 (flash, 70 us) / CH4 (hold-out, 1 ms) -> pins
  *      are dropped by the compare ISR, no delay loops
  *   4. push {trigger_id, T_event, T_k, pps_seq} into the SRAM3 ring buffer
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shared_ringbuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ======================= USER CONFIGURATION ================================
 * TIM5 clock from CubeMX: PLL3P = 24MHz/2*34.813/2 ~ 208.878 MHz, APB1/2, x2.
 * Verify in the Clock Configuration tab and edit if different.               */
#define TIM_CLK_HZ          208878000UL
#define US_TO_TICKS(us)     ((uint32_t)(((uint64_t)TIM_CLK_HZ * (us)) / 1000000ULL))

#define FLASH_PULSE_US      70U      /* Xenon flash pulse width            */
#define HOLDOUT_PULSE_US    1000U    /* Hold-Out pulse width               */
#define DEADTIME_US         1000U    /* ignore triggers after an accepted one */

/* Which TIM5 channel carries the IR trigger?
 *   2 -> trigger on TIM5_CH2 (PH11, CN7 pin 31), PPS on TIM5_CH1 (PA0)
 *   1 -> trigger on TIM5_CH1 (PA0),               PPS on TIM5_CH2 (PH11)   */
#define TRIG_CH             2

#define TRIG_POLARITY       TIM_INPUTCHANNELPOLARITY_RISING
#define PPS_POLARITY        TIM_INPUTCHANNELPOLARITY_FALLING
#define IC_FILTER           0U

/* Hold-Out output pin. Uncomment and set once you confirm which MCU pin sits
 * on CN7 pin 32/33 (and NOT PH11 - that is the TIM5_CH2 capture input).      */
//#define HOLDOUT_PORT      GPIOx
//#define HOLDOUT_PIN       GPIO_PIN_y

/* Optional Hold-In gate: polled input, active low, starts a dead-time.
 * Leave 0 unless a separate Hold-In line is really wired to this pin.        */
#define USE_HOLDIN          0
#define HOLDIN_PORT         GPIOG
#define HOLDIN_PIN          GPIO_PIN_15

/* Bench self-test: fires a software trigger every N ms (0 = off).
 * Handy with only the board connected; keep 0 during real runs.             */
#define SELFTEST_PERIOD_MS  0U
/* ========================================================================= */

/* 1 = records are published to the A7 only after the NEXT PPS edge (T_k+1)
 *     arrives, so the daemon can interpolate. Needs a real PPS input.
 * 0 = publish immediately (t_k1 = 0, VALID flag clear) - bench use without PPS */
#define PPS_REQUIRED        1

#if (TRIG_CH == 2)
#define TRIG_CHANNEL        TIM_CHANNEL_2
#define PPS_CHANNEL         TIM_CHANNEL_1
#define TRIG_ACTIVE         HAL_TIM_ACTIVE_CHANNEL_2
#define TRIG_EGR            TIM_EGR_CC2G
#else
#define TRIG_CHANNEL        TIM_CHANNEL_1
#define PPS_CHANNEL         TIM_CHANNEL_2
#define TRIG_ACTIVE         HAL_TIM_ACTIVE_CHANNEL_1
#define TRIG_EGR            TIM_EGR_CC1G
#endif

#define FLASH_ON()          (FLASH_OUT_GPIO_Port->BSRR = (uint32_t)FLASH_OUT_Pin)
#define FLASH_OFF()         (FLASH_OUT_GPIO_Port->BSRR = (uint32_t)FLASH_OUT_Pin << 16)
#ifdef HOLDOUT_PIN
#define HOLDOUT_ON()        (HOLDOUT_PORT->BSRR = (uint32_t)HOLDOUT_PIN)
#define HOLDOUT_OFF()       (HOLDOUT_PORT->BSRR = (uint32_t)HOLDOUT_PIN << 16)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
/* Shared SRAM3 ring buffer, physical base 0x10040000 */
volatile shared_ringbuffer_t * const shared_buf = (shared_ringbuffer_t *) SHARED_SRAM_BASE_ADDR;

static volatile uint32_t last_pps_tick   = 0;   /* T_k  : latest PPS capture   */
static volatile uint32_t pps_seq         = 0;
static volatile uint32_t trigger_id      = 0;
static volatile uint32_t last_accept     = 0;   /* timer tick of last accepted trigger */
static volatile uint8_t  have_last       = 0;
static volatile uint32_t dropped_count   = 0;   /* triggers rejected by dead-time */
static volatile uint32_t pend_w          = 0;   /* next slot to fill (>= published write_idx) */
static volatile uint8_t  have_pps        = 0;   /* at least one PPS edge seen */
static volatile uint8_t  test_pending    = 0;   /* next trigger is a software test */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void App_GPIO_Init(void);
static void App_ConfigCapture(void);
static void ring_push(uint32_t t_event, uint32_t type);
static void pps_complete(uint32_t t_pps);
static void on_trigger(uint32_t t_event);
void Trigger_Software_Test_Event(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Force a capture event on the trigger channel (no external signal needed) */
void Trigger_Software_Test_Event(void)
{
    test_pending = 1;
    TIM5->EGR |= TRIG_EGR;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Enable CP10 and CP11 full access for the Cortex-M4 FPU */
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  App_GPIO_Init();

  /* Initialize the shared ring buffer header for the Cortex-A7 reader */
  shared_buf->magic          = RINGBUFFER_MAGIC;
  shared_buf->write_idx      = 0;
  shared_buf->read_idx       = 0;
  shared_buf->overflow_count = 0;

  /* Apply trigger/PPS polarity, then start both captures with interrupts */
  App_ConfigCapture();
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if USE_HOLDIN
    {
      static uint8_t prev = 1;
      uint8_t now = (HAL_GPIO_ReadPin(HOLDIN_PORT, HOLDIN_PIN) == GPIO_PIN_SET);
      if (prev && !now)               /* falling edge -> start dead-time */
      {
        last_accept = TIM5->CNT;
        have_last   = 1;
      }
      prev = now;
    }
#endif
#if (SELFTEST_PERIOD_MS > 0U)
    {
      static uint32_t t_last = 0;
      if ((HAL_GetTick() - t_last) >= SELFTEST_PERIOD_MS)
      {
        t_last = HAL_GetTick();
        Trigger_Software_Test_Event();
      }
    }
#endif
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 6660;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_OUT_GPIO_Port, FLASH_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : FLASH_OUT_Pin */
  GPIO_InitStruct.Pin = FLASH_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLASH_OUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Extra GPIO (Hold-Out output, optional Hold-In input) kept out of the
 * generated code so CubeMX regeneration does not wipe it. */
static void App_GPIO_Init(void)
{
  GPIO_InitTypeDef gi = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

#ifdef HOLDOUT_PIN
  HAL_GPIO_WritePin(HOLDOUT_PORT, HOLDOUT_PIN, GPIO_PIN_RESET);
  gi.Pin   = HOLDOUT_PIN;
  gi.Mode  = GPIO_MODE_OUTPUT_PP;
  gi.Pull  = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(HOLDOUT_PORT, &gi);
#endif

#if USE_HOLDIN
  gi.Pin   = HOLDIN_PIN;
  gi.Mode  = GPIO_MODE_INPUT;
  gi.Pull  = GPIO_PULLUP;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HOLDIN_PORT, &gi);
#endif
  (void)gi;
}

/* Apply edge polarity per role (trigger vs PPS) */
static void App_ConfigCapture(void)
{
  TIM_IC_InitTypeDef ic = {0};
  ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic.ICPrescaler = TIM_ICPSC_DIV1;
  ic.ICFilter    = IC_FILTER;

  ic.ICPolarity = TRIG_POLARITY;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &ic, TRIG_CHANNEL) != HAL_OK) { Error_Handler(); }

  ic.ICPolarity = PPS_POLARITY;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &ic, PPS_CHANNEL) != HAL_OK) { Error_Handler(); }
}

/* Fill a slot with the event and the PPS edge BEFORE it (T_k). The record is
 * not visible to the A7 until pps_complete() adds T_k+1 and moves write_idx. */
static void ring_push(uint32_t t_event, uint32_t type)
{
  uint32_t w    = pend_w;
  uint32_t next = (w + 1U) % RINGBUFFER_CAPACITY;

  if (next == shared_buf->read_idx)
  {
    shared_buf->overflow_count++;
    return;
  }

  shared_buf->records[w].trigger_id    = trigger_id++;
  shared_buf->records[w].event_type    = type;
  shared_buf->records[w].t_event_ticks = t_event;
  shared_buf->records[w].t_k_ticks     = last_pps_tick;
  shared_buf->records[w].t_k1_ticks    = 0;
  shared_buf->records[w].pps_sequence  = pps_seq;
  shared_buf->records[w].flags         = 0;
  shared_buf->records[w].reserved      = 0;
  pend_w = next;

#if !PPS_REQUIRED
  __DMB();
  shared_buf->write_idx = pend_w;
#endif
}

/* New PPS edge: close all pending records with T_k+1, then publish them */
static void pps_complete(uint32_t t_pps)
{
  uint32_t i = shared_buf->write_idx;
  while (i != pend_w)
  {
    shared_buf->records[i].t_k1_ticks = t_pps;
    if (have_pps) { shared_buf->records[i].flags |= STATUS_FLAG_VALID; }
    i = (i + 1U) % RINGBUFFER_CAPACITY;
  }
  __DMB();                          /* data visible before index moves */
  shared_buf->write_idx = pend_w;

  last_pps_tick = t_pps;            /* becomes T_k for later events */
  pps_seq++;
  have_pps = 1;
}

static void on_trigger(uint32_t t_event)
{
  uint32_t type = test_pending ? EVENT_TYPE_TEST : EVENT_TYPE_IR_TRIGGER;
  test_pending = 0;

  /* Dead-time gate: ignore anything within DEADTIME_US of the last accepted one */
  if (have_last && ((int32_t)(t_event - last_accept) < (int32_t)US_TO_TICKS(DEADTIME_US)))
  {
    dropped_count++;
    return;
  }

  FLASH_ON();
#ifdef HOLDOUT_PIN
  HOLDOUT_ON();
#endif

  /* Arm compare channels: pins are dropped by the compare ISR below */
  uint32_t now = TIM5->CNT;
  TIM5->CCR3 = now + US_TO_TICKS(FLASH_PULSE_US);
  TIM5->SR   = ~TIM_SR_CC3IF;
  TIM5->DIER |= TIM_DIER_CC3IE;
#ifdef HOLDOUT_PIN
  TIM5->CCR4 = now + US_TO_TICKS(HOLDOUT_PULSE_US);
  TIM5->SR   = ~TIM_SR_CC4IF;
  TIM5->DIER |= TIM_DIER_CC4IE;
#endif

  last_accept = t_event;
  have_last   = 1;
  ring_push(t_event, type);
}

/* Input capture: trigger and PPS (called from HAL_TIM_IRQHandler) */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM5) { return; }

  if (htim->Channel == TRIG_ACTIVE)
  {
    uint32_t t = (TRIG_CH == 2) ? TIM5->CCR2 : TIM5->CCR1;
    on_trigger(t);
  }
  else
  {
    pps_complete((TRIG_CH == 2) ? TIM5->CCR1 : TIM5->CCR2);
  }
}

/* Output compare: end of the 70 us flash pulse / 1 ms hold-out pulse */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM5) { return; }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    FLASH_OFF();
    TIM5->DIER &= ~TIM_DIER_CC3IE;
  }
#ifdef HOLDOUT_PIN
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    HOLDOUT_OFF();
    TIM5->DIER &= ~TIM_DIER_CC4IE;
  }
#endif
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
