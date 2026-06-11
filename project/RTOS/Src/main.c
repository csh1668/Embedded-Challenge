#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"


// 전체 로직
// 우선순위 1: 정면에 장애물 감지 -> recovery_turn
// 우선순위 2: 0 방향 복귀가 가능한 경우 (측면 다시 트임 감지) -> 그쪽으로 진입+회전
// 우선순위 3: 한쪽 IR에 장애물 감지 -> 반대편으로 micro_steer
// 우선순위 4: 1~3 모두 아님 -> 직진

typedef enum {
    MODE_NORMAL,                // 챌린지 본 동작
    MODE_ROTATION_CALIBRATION,  // 90도 캘리브레이션 모드
    MODE_MOVEMENT_TEST,         // 간단한 주행 테스트
} Mode;
static Mode current_mode = MODE_NORMAL;

// IR 센서 or 초음파 센서 임계값 목록
#define IR_NEAR_THRESH         700U   // 한쪽 IR 값이 이 값을 초과하면 해당 방향에 장애물이 감지되었다고 판단
#define IR_CLOSE_THRESH        1000U  // 양쪽 IR 값이 모두 이 값을 초과하면 정면에 장애물이 있다고 판단
#define FRONT_HARD_STOP_CM     12U    // 정면 초음파 센서 값이 이 값 이하라면, 정면에 장애물이 있다고 판단
#define SIDE_ASYMMETRY_CM      20U    // 두 측면 초음파 센서 값의 차이가 이 값 초과라면, 더 트인 쪽으로 회전한다
#define SIDE_OPEN_THRESH       40U    // 측면 초음파 센서 값이 이 값 이상이면, 해당 방향이 트여있다고 판단
#define SIDE_NEAR_CM           20U    // 측면 초음파 센서 값이 이 값 이하이면, 벽이 가깝다고 판단 (동적 장애물 판정용)

// 모터 동작 값
#define TURN_90_TICKS              900U   // 90도 회전 엔코더 ticks (양 바퀴 평균, ROTATE_PWM_PULSE=17000 기준 캘리브레이션)
#define TURN_90_SAFETY_MS          10000U  // 엔코더 closed-loop 회전 fallback timeout (TARGET 미도달 시 강제 중단)
#define TURN_SETTLE_MS             100U   // 90도 회전 후 바퀴 안정에 사용하는 시간 - 그냥 회전해버리면 바퀴가 더 돌아버려서 90도 이상 회전하는 문제가 있었음
#define MICRO_STEER_MS             90U    // micro_steer 때 사용하는 시간 (FORWARD_PWM_PULSE 에서 동작)
#define OPENING_ADVANCE_MS         650U  // 0 방향 복귀 할때, 감지 후 추가로 더 전진하는 시간 (FORWARD_PWM_PULSE 에서 동작 — 속도 변경 시 재캘리브레이션 권장)
#define DYNAMIC_OBSTACLE_WAIT_MS   2500U  // 동적 장애물 추정 시 정지 대기 시간 (이 시간 후에도 장애물 잔존이면 정적 장애물로 간주)

// 태스크 별 주기 (ms)
#define SENSE_PERIOD_MS        100U
#define DRIVE_PERIOD_MS        50U
#define IR_PERIOD_MS           50U
#define DEBUG_PERIOD_MS        500U

// 초음파 outlier 필터에 사용되는 상수값들
// 초음파 센서의 하드웨어 오류로 인해 값 튐이 자주 발생하는데, 로봇 움직임에 악영향을 줄 수 있다. 소프트웨어 보정이 필요함.
// 사례 1) 너무 가까움 -> 측정 값이 갑자기 커짐
// 사례 2) 평시 상태 -> 측정 값이 갑자기 커짐 / 갑자기 작아짐
#define ULTRASONIC_OUTLIER_MAX     150U  // raw 값이 이 값을 초과하면 노이즈로 간주
#define SIDE_OUTLIER_STREAK        2U    // 측면 outlier N번 연속이면 트임으로 간주
#define SIDE_OPEN_SENTINEL         100U  // 측면 트임으로 간주할 때 고정할 값
#define FRONT_OUTLIER_STREAK       3U    // 전방 outlier N번 연속이고 이전 값이 close였다면 가깝다고 간주
#define FRONT_OUTLIER_CLOSE_GUARD  30U
#define FRONT_OUTLIER_FORCE_CM     5U    // 전방 close로 간주할 때 고정할 값

// 모터 동작 속도 (PWM Pulse)
// 전진은 빠르게 (cruise), 회전/조향은 느리게. (회전/조향도 빠르면 관성 때문에 실제 회전각이 불규칙해지는 문제가 확인됨)
// micro_steer 도 cruise 가속 후 너무 sharp 하게 휘므로 별도 낮은 PWM 사용.
#define FORWARD_PWM_PULSE      20000U  // 전진 cruise 속도
#define ROTATE_PWM_PULSE       17000U  // 90° 회전 시 사용
#define STEER_PWM_PULSE        14000U  // micro_steer 시 사용


/* =====================================================================
 *  하드웨어 값들
 * ===================================================================== */

TIM_HandleTypeDef    TimHandle1, TimHandle2, TimHandle3, TimHandle4;
TIM_IC_InitTypeDef   sICConfig;
TIM_OC_InitTypeDef   sConfig1, sConfig2, sConfig3;
uint32_t uwPrescalerValue = 0;

uint16_t motorInterrupt1 = 0;  // 좌측 바퀴 엔코더
uint16_t motorInterrupt2 = 0;  // 우측 바퀴 엔코더
uint8_t encoder_right = READY;
uint8_t encoder_left  = READY;

// 초음파 캡처 값
uint32_t uwIC2Value1 = 0, uwIC2Value2 = 0, uwDiffCapture1 = 0;
uint32_t uwIC2Value3 = 0, uwIC2Value4 = 0, uwDiffCapture2 = 0;
uint32_t uwIC2Value5 = 0, uwIC2Value6 = 0, uwDiffCapture3 = 0;

ADC_HandleTypeDef      AdcHandle1, AdcHandle2;
ADC_ChannelConfTypeDef adcConfig1, adcConfig2;
__IO uint32_t uhADCxLeft, uhADCxRight; // IR ADC 값

extern UART_HandleTypeDef UartHandle1, UartHandle2;


// 함수 프로토타입
static void SystemClock_Config(void);
static void EXTILine_Config(void);
static void Error_Handler(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);


#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&UartHandle1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}


/* =====================================================================
 *  로봇 상태
 * ===================================================================== */

typedef enum {
    BEHAVIOR_FORWARD  = 0,  // 전진 중
    BEHAVIOR_VEER_L,        // 좌측으로 micro steer 중
    BEHAVIOR_VEER_R,        // 우측으로 micro steer 중
    BEHAVIOR_RECOVERY,      // 복구 회전 중
    BEHAVIOR_DYNAMIC_WAIT,  // 동적 장애물 추정 → 정지 대기 중
} Behavior;

volatile Behavior current_behavior = BEHAVIOR_FORWARD;

// 현재 향해있는 방향: -1/0/+1 / 0=시작 방향
static int8_t heading = 0;

// 소프트웨어적으로 보정된 초음파 센서 값들 (cm)
volatile uint32_t front_cm = 100, left_cm = 100, right_cm = 100;
volatile uint32_t prev_front_cm = 100, prev_left_cm = 100, prev_right_cm = 100;


/* =====================================================================
 *  회전 / 조향 헬퍼
 * ===================================================================== */


static void set_motor_pwm(uint32_t pulse)
{
    TIM8->CCR1 = pulse;
    TIM8->CCR2 = pulse;
    TIM4->CCR1 = pulse;
    TIM4->CCR2 = pulse;
}

static uint32_t abs_diff16(uint16_t cur, uint16_t base)
{
    int16_t d = (int16_t)(cur - base);
    return (d < 0) ? (uint32_t)(-(int32_t)d) : (uint32_t)d;
}

static void rotate_until_ticks(void)
{
    uint16_t r0 = motorInterrupt1;
    uint16_t l0 = motorInterrupt2;
    uint32_t elapsed = 0;
    while (elapsed < TURN_90_SAFETY_MS) {
        uint32_t adr = abs_diff16(motorInterrupt1, r0);
        uint32_t adl = abs_diff16(motorInterrupt2, l0);
        uint32_t mag = (adr + adl) / 2U;
        if (mag >= TURN_90_TICKS) break;
        osDelay(5);
        elapsed += 5;
    }
}

static void turn_90_left(void)
{
    Motor_Stop();
    osDelay(TURN_SETTLE_MS);
    set_motor_pwm(ROTATE_PWM_PULSE);
    Motor_Left();
    rotate_until_ticks();
    Motor_Stop();
    set_motor_pwm(FORWARD_PWM_PULSE);
    osDelay(TURN_SETTLE_MS);
    if (heading > -1) heading--;
}

static void turn_90_right(void)
{
    Motor_Stop();
    osDelay(TURN_SETTLE_MS);
    set_motor_pwm(ROTATE_PWM_PULSE);
    Motor_Right();
    rotate_until_ticks();
    Motor_Stop();
    set_motor_pwm(FORWARD_PWM_PULSE);
    osDelay(TURN_SETTLE_MS);
    if (heading < 1) heading++;
}

// 정면에 장애물을 만나서 90도 회전을 해야 하는 경우
// 우선순위 1: L/R 차이가 크면 더 트인쪽으로 회전한다
// 우선순위 2: 둘이 비슷하고 heading != 0이라면 heading이 0이 될 방향으로 회전한다
// 우선순위 3: 둘이 비슷하고 heading == 0이라면 초음파 값 기준으로 더 트인 곳으로 회전한다
static void recovery_turn(void)
{
    int32_t diff = (int32_t)left_cm - (int32_t)right_cm;
    if (diff >  (int32_t)SIDE_ASYMMETRY_CM) { turn_90_left();  return; }
    if (diff < -(int32_t)SIDE_ASYMMETRY_CM) { turn_90_right(); return; }

    if (heading > 0)       turn_90_left();
    else if (heading < 0)  turn_90_right();
    else if (left_cm > right_cm)    turn_90_left();
    else                   turn_90_right();
}

// 미세 조향을 해야 하는 경우
// 한쪽 PWM 채널을 잠시 정지시켜서 그쪽으로 휘게 한다
static void micro_steer_right(void)
{
    set_motor_pwm(STEER_PWM_PULSE);
    HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
    osDelay(MICRO_STEER_MS);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
    set_motor_pwm(FORWARD_PWM_PULSE);
}

static void micro_steer_left(void)
{
    set_motor_pwm(STEER_PWM_PULSE);
    HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
    osDelay(MICRO_STEER_MS);
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    set_motor_pwm(FORWARD_PWM_PULSE);
}

// 정면에 장애물이 있는지 판정
// 1 조건: 정면 초음파가 가까움을 감지했다
// 2 조건: 양쪽 IR 센서가 매우 가까움을 감지했다
static bool is_front_emergency(void)
{
    bool front_close = (front_cm < FRONT_HARD_STOP_CM);
    bool L_close = (uhADCxLeft  > IR_CLOSE_THRESH);
    bool R_close = (uhADCxRight > IR_CLOSE_THRESH);
    return front_close || (L_close && R_close);
}

// 정면 장애물 감지를 폴링하면서 max_ms 만큼 전진. 그동안 측면 IR 감지 시 micro_steer 도 수행.
// 장애물이 감지되면 조기 정지한다.
// 호출 전에 Motor_Forward() 가 시작돼 있어야 함.
static void advance_watch_front(uint32_t max_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < max_ms) {
        uint32_t step = (max_ms - elapsed < DRIVE_PERIOD_MS) ? (max_ms - elapsed) : DRIVE_PERIOD_MS;
        osDelay(step);
        elapsed += step;
        if (is_front_emergency()) return;

        bool L_near = (uhADCxLeft  > IR_NEAR_THRESH);
        bool R_near = (uhADCxRight > IR_NEAR_THRESH);
        bool steered = false;
        if (L_near && !R_near)      { micro_steer_right(); steered = true; }
        else if (R_near && !L_near) { micro_steer_left();  steered = true; }
        if (steered) {
            elapsed = (elapsed > MICRO_STEER_MS / 2) ? (elapsed - MICRO_STEER_MS / 2) : 0;
        }
    }
}

static bool should_heading_recover(void)
{
    if (front_cm < FRONT_HARD_STOP_CM) return false;
    bool go_left = (heading > 0 && left_cm  >= SIDE_OPEN_THRESH);
    bool go_right = (heading < 0 && right_cm >= SIDE_OPEN_THRESH);
    if (!go_left && !go_right) return false;
    return true;
}


/* =====================================================================
 *  UW_task
 * ===================================================================== */

// 전방 초음파 센서 필터: outlier가 연속으로 발생했으며 이전 거리가 가까움이었다면 매우 가까움으로 간주한다
static void filter_front(uint32_t raw, uint8_t *streak,
                         volatile uint32_t *out, volatile uint32_t *prev)
{
    if (raw > ULTRASONIC_OUTLIER_MAX) {
        if (++(*streak) >= FRONT_OUTLIER_STREAK
            && *prev < FRONT_OUTLIER_CLOSE_GUARD)
            *out = FRONT_OUTLIER_FORCE_CM;
        else
            *out = *prev;
    } else {
        *streak = 0;
        *out  = raw;
        *prev = raw;
    }
}

// 측면 초음파 센서 필터: outlier가 연속으로 발생하면 트임으로 간주한다
static void filter_side(uint32_t raw, uint8_t *streak,
                        volatile uint32_t *out, volatile uint32_t *prev)
{
    if (raw > ULTRASONIC_OUTLIER_MAX) {
        if (++(*streak) >= SIDE_OUTLIER_STREAK)
            *out = SIDE_OPEN_SENTINEL;
        else
            *out = *prev;
    } else {
        *streak = 0;
        *out  = raw;
        *prev = raw;
    }
}

void UW_task(void)
{
    static uint8_t front_streak = 0, left_streak = 0, right_streak = 0;

    osDelay(500); // 시스템 안정화 대기
    for (;;)
    {
        filter_front(uwDiffCapture2 / 58, &front_streak, &front_cm, &prev_front_cm);
        filter_side(uwDiffCapture3 / 58, &left_streak,  &left_cm,  &prev_left_cm);
        filter_side(uwDiffCapture1 / 58, &right_streak, &right_cm, &prev_right_cm);

        osDelay(SENSE_PERIOD_MS);
    }
}


/* =====================================================================
 *  Drive_task
 * ===================================================================== */

// 현재 BEHAVIOR 상태에 따라 LED1~4를 켜서 표시한다
static void update_state_leds(void)
{
    BSP_LED_Off(LED1); BSP_LED_Off(LED2);
    BSP_LED_Off(LED3); BSP_LED_Off(LED4);
    switch (current_behavior) {
        case BEHAVIOR_FORWARD:   BSP_LED_On(LED1); break;
        case BEHAVIOR_VEER_L:    BSP_LED_On(LED2); break;
        case BEHAVIOR_VEER_R:    BSP_LED_On(LED3); break;
        case BEHAVIOR_RECOVERY:  BSP_LED_On(LED4); break;
        case BEHAVIOR_DYNAMIC_WAIT:
            BSP_LED_On(LED1); BSP_LED_On(LED2);
            BSP_LED_On(LED3); BSP_LED_On(LED4);
            break;
    }
}


void Drive_task(void)
{
    osDelay(1000);
    Motor_Forward();

    for (;;)
    {
        update_state_leds();

        bool L_near = (uhADCxLeft  > IR_NEAR_THRESH);
        bool R_near = (uhADCxRight > IR_NEAR_THRESH);

        // 우선순위 1: 정면에 장애물이 있다
        if (is_front_emergency()) {
            current_behavior = BEHAVIOR_RECOVERY;
            update_state_leds();
            Motor_Stop();
            recovery_turn();
            Motor_Forward();
            continue;
        }

        // 우선순위 2: 0 방향으로 복귀할 수 있다
        if (should_heading_recover()) {
            current_behavior = BEHAVIOR_RECOVERY;
            update_state_leds();
            Motor_Forward();
            advance_watch_front(OPENING_ADVANCE_MS);
            Motor_Stop();
            bool go_left  = (heading > 0 && left_cm  >= SIDE_OPEN_THRESH);
            if (go_left) turn_90_left();
            else         turn_90_right();
            Motor_Forward();
            continue;
        }

        // 우선순위 3: IR 센서 기반 미세조정이 필요하다
        if (L_near && R_near) {
            // 정면 초음파는 장애물이 감지되지 않았지만,
            // 양쪽 IR 센서 둘다 near라면 좁은 틈을 통과중이라고 판단하여 전진한다
            current_behavior = BEHAVIOR_FORWARD;
            Motor_Forward();
        } else if (L_near) {
            current_behavior = BEHAVIOR_VEER_R;
            Motor_Forward();
            micro_steer_right();
        } else if (R_near) {
            current_behavior = BEHAVIOR_VEER_L;
            Motor_Forward();
            micro_steer_left();
        } else {
            // 우선순위 4: 이상 없으므로 전진
            current_behavior = BEHAVIOR_FORWARD;
            Motor_Forward();
        }
        osDelay(DRIVE_PERIOD_MS);
    }
}


/* =====================================================================
 *  IR 태스크
 * ===================================================================== */

static uint32_t read_ir(ADC_HandleTypeDef *adc)
{
    HAL_ADC_Start(adc);
    HAL_ADC_PollForConversion(adc, 0xFF);
    uint32_t v = HAL_ADC_GetValue(adc);
    if      (v > 2000) v = 2000;
    else if (v <  100) v =  100;
    return v;
}

void IR_task(void)
{
    for (;;) {
        uhADCxLeft  = read_ir(&AdcHandle1);
        uhADCxRight = read_ir(&AdcHandle2);
        osDelay(IR_PERIOD_MS);
    }
}

/* =====================================================================
 *  디버그 태스크
 * ===================================================================== */

/* Tera Term 모니터링. raw 가 0/고정 = 초음파 인터럽트 미수신 (TIM10 CH1 점검). */
void Debug_task(void)
{
    osDelay(1200);
    printf("\r\n[Debug] behavior: 0=FWD 1=VEER_L 2=VEER_R 3=RECOVERY | F cm | L cm | R cm | IRL IRR | hd\r\n");
    for (;;) {
        printf("\r\n[%d] F=%lu | L=%lu | R=%lu | IRL=%lu IRR=%lu | hd=%d",
               (int)current_behavior,
               (unsigned long)front_cm,
               (unsigned long)left_cm, (unsigned long)right_cm,
               (unsigned long)uhADCxLeft, (unsigned long)uhADCxRight,
               (int)heading);
        osDelay(DEBUG_PERIOD_MS);
    }
}


/* =====================================================================
 *  캘리브레이션 / 테스트 모드
 * ===================================================================== */

// 엔코더 기반 90도 회전 캘리브레이션 후보값.
static const uint32_t calibration_target_ticks[] = {
    800U, 850U, 900U, 950U, 1000U
};

// 캘리브레이션 모드 (엔코더 기반 closed-loop)
void Calibration_task(void)
{
    const uint32_t n = sizeof(calibration_target_ticks)
                       / sizeof(calibration_target_ticks[0]);
    const uint32_t SAFETY_MS  = 10000U; // closed-loop fallback timeout (1300 ticks 도달까지 여유)
    const uint32_t POLL_MS    = 5U;

    BSP_LED_Off(LED1); BSP_LED_Off(LED2);
    BSP_LED_Off(LED3); BSP_LED_Off(LED4);

    osDelay(1000);
    set_motor_pwm(ROTATE_PWM_PULSE);
    printf("\r\n=== Encoder-based LEFT-turn calibration: %lu tests ===\r\n",
           (unsigned long)n);
    printf("PWM=%u (rotate). 1deg ~= 10 ticks. Right-turn baseline: 900 ticks = 90deg.\r\n",
           (unsigned)ROTATE_PWM_PULSE);
    printf("Record actual angle per target; choose the target whose result lands on 90.\r\n");
    printf("Starting in 4s.\r\n");
    osDelay(4000);

    for (uint32_t i = 0; i < n; i++) {
        uint32_t target = calibration_target_ticks[i];

        BSP_LED_On(LED1);
        printf("\r\n[%lu/%lu] target=%lu ticks (ready 1s)\r\n",
               (unsigned long)(i + 1), (unsigned long)n, (unsigned long)target);
        osDelay(1000);
        BSP_LED_Off(LED1);

        uint16_t r0 = motorInterrupt1, l0 = motorInterrupt2;

        BSP_LED_On(LED3);
        Motor_Stop();
        osDelay(TURN_SETTLE_MS);
        Motor_Left();

        // closed-loop: target ticks 도달 시 즉시 정지
        uint32_t elapsed = 0;
        uint32_t reach_ms = 0;
        bool reached = false;
        uint32_t mag_at_stop = 0;
        while (elapsed < SAFETY_MS) {
            uint32_t adr = abs_diff16(motorInterrupt1, r0);
            uint32_t adl = abs_diff16(motorInterrupt2, l0);
            uint32_t mag = (adr + adl) / 2U;
            if (mag >= target) {
                reach_ms = elapsed;
                mag_at_stop = mag;
                reached = true;
                break;
            }
            osDelay(POLL_MS);
            elapsed += POLL_MS;
        }

        Motor_Stop();
        BSP_LED_Off(LED3);

        // settle 후 최종 ticks (관성 누적량 포함)
        osDelay(TURN_SETTLE_MS);
        int16_t dr_final = (int16_t)(motorInterrupt1 - r0);
        int16_t dl_final = (int16_t)(motorInterrupt2 - l0);
        int32_t adr = (dr_final < 0) ? -(int32_t)dr_final : (int32_t)dr_final;
        int32_t adl = (dl_final < 0) ? -(int32_t)dl_final : (int32_t)dl_final;
        int32_t mag_final = (adr + adl) / 2;
        int32_t coast = mag_final - (int32_t)mag_at_stop;

        printf("Done target=%lu | %s @ %lums | stop_mag=%lu, final_mag=%ld (coast=+%ld) | ~%ld deg\r\n",
               (unsigned long)target,
               reached ? "reached" : "TIMEOUT",
               (unsigned long)reach_ms,
               (unsigned long)mag_at_stop,
               (long)mag_final, (long)coast,
               (long)(mag_final / 2));
        printf("Reposition within 3s.\r\n");

        BSP_LED_On(LED2);
        osDelay(3000);
        BSP_LED_Off(LED2);
    }

    printf("\r\n=== All tests done. ===\r\n");
    BSP_LED_On(LED1); BSP_LED_On(LED2);
    BSP_LED_On(LED3); BSP_LED_On(LED4);
    Motor_Stop();
    while (1) osDelay(1000);
}

// 주행 테스트 모드
// 90도 회전과 전진을 반복하면서 제대로 동작하는지 테스트한다
void MovementTest_task(void)
{
    BSP_LED_Off(LED1); BSP_LED_Off(LED2);
    BSP_LED_Off(LED3); BSP_LED_Off(LED4);

    osDelay(1000);
    printf("\r\n=== Movement test: F5s -> L90 -> F5s -> R90 -> F5s (3s) ===\r\n");
    osDelay(3000);

    printf("[1/5] Forward 5s\r\n");
    BSP_LED_On(LED1); Motor_Forward(); osDelay(5000); Motor_Stop(); BSP_LED_Off(LED1);
    osDelay(500);

    printf("[2/5] Rotate LEFT 90 (TURN_90_TICKS=%u)\r\n", (unsigned)TURN_90_TICKS);
    BSP_LED_On(LED2); turn_90_left(); BSP_LED_Off(LED2);
    osDelay(500);

    printf("[3/5] Forward 5s\r\n");
    BSP_LED_On(LED1); Motor_Forward(); osDelay(5000); Motor_Stop(); BSP_LED_Off(LED1);
    osDelay(500);

    printf("[4/5] Rotate RIGHT 90 (TURN_90_TICKS=%u)\r\n", (unsigned)TURN_90_TICKS);
    BSP_LED_On(LED3); turn_90_right(); BSP_LED_Off(LED3);
    osDelay(500);

    printf("[5/5] Forward 5s\r\n");
    BSP_LED_On(LED1); Motor_Forward(); osDelay(5000); Motor_Stop(); BSP_LED_Off(LED1);

    printf("\r\n=== Test complete. ===\r\n");
    BSP_LED_On(LED1); BSP_LED_On(LED2);
    BSP_LED_On(LED3); BSP_LED_On(LED4);
    Motor_Stop();
    while (1) osDelay(1000);
}


/* =====================================================================
 *  하드웨어 초기화 헬퍼
 * ===================================================================== */

static void motor_pwm_init(TIM_HandleTypeDef *th, TIM_OC_InitTypeDef *cfg,
                           TIM_TypeDef *inst)
{
    cfg->OCMode     = TIM_OCMODE_PWM1;
    cfg->OCPolarity = TIM_OCPOLARITY_HIGH;
    cfg->OCFastMode = TIM_OCFAST_DISABLE;
    cfg->Pulse      = FORWARD_PWM_PULSE;

    th->Instance           = inst;
    th->Init.Prescaler     = uwPrescalerValue;
    th->Init.Period        = 20000;
    th->Init.ClockDivision = 0;
    th->Init.CounterMode   = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(th);

    HAL_TIM_PWM_ConfigChannel(th, cfg, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(th, cfg, TIM_CHANNEL_2);
}

static void ir_adc_init(ADC_HandleTypeDef *adc, ADC_ChannelConfTypeDef *cfg,
                        ADC_TypeDef *inst, uint32_t channel)
{
    adc->Instance                    = inst;
    adc->Init.ClockPrescaler         = ADC_CLOCKPRESCALER_PCLK_DIV2;
    adc->Init.Resolution             = ADC_RESOLUTION12b;
    adc->Init.ScanConvMode           = DISABLE;
    adc->Init.ContinuousConvMode     = DISABLE;
    adc->Init.DiscontinuousConvMode  = DISABLE;
    adc->Init.NbrOfDiscConversion    = 0;
    adc->Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_NONE;
    adc->Init.ExternalTrigConv       = ADC_EXTERNALTRIGCONV_T1_CC1;
    adc->Init.DataAlign              = ADC_DATAALIGN_RIGHT;
    adc->Init.NbrOfConversion        = 1;
    adc->Init.DMAContinuousRequests  = DISABLE;
    adc->Init.EOCSelection           = DISABLE;
    HAL_ADC_Init(adc);

    cfg->Channel      = channel;
    cfg->Rank         = 1;
    cfg->SamplingTime = ADC_SAMPLETIME_480CYCLES;
    cfg->Offset       = 0;
    HAL_ADC_ConfigChannel(adc, cfg);
}


/* =====================================================================
 *  main 함수 진입부
 * ===================================================================== */

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    HAL_Init();
    SystemClock_Config();   /* 180 MHz */
    BSP_COM1_Init();

    BSP_LED_Init(LED1); BSP_LED_Init(LED2);
    BSP_LED_Init(LED3); BSP_LED_Init(LED4);


    /* ---- 모터 PWM ---- */
    uwPrescalerValue = (SystemCoreClock / 2) / 1000000;

    /* MC_EN (PB2) High → 모터 드라이버 ON */
    __GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_2;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

    motor_pwm_init(&TimHandle1, &sConfig1, TIM8);
    motor_pwm_init(&TimHandle2, &sConfig2, TIM4);
    EXTILine_Config();   /* 엔코더 EXTI */


    /* ---- 초음파: TIM3 IC (echo) + TIM10 PWM (trig) ---- */
    uwPrescalerValue = ((SystemCoreClock / 2) / 1000000) - 1;
    TimHandle3.Instance           = TIM3;
    TimHandle3.Init.Period        = 0xFFFF;
    TimHandle3.Init.Prescaler     = uwPrescalerValue;
    TimHandle3.Init.ClockDivision = 0;
    TimHandle3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    if (HAL_TIM_IC_Init(&TimHandle3) != HAL_OK) Error_Handler();

    sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_1);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_2);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_3);
    HAL_TIM_IC_ConfigChannel(&TimHandle3, &sICConfig, TIM_CHANNEL_4);

    /* CH2=우, CH3=전, CH4=좌 */
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&TimHandle3, TIM_CHANNEL_4);

    uwPrescalerValue = (SystemCoreClock / 2 / 131099) - 1;
    TimHandle4.Instance           = TIM10;
    TimHandle4.Init.Prescaler     = uwPrescalerValue;
    TimHandle4.Init.Period        = 0xFFFF;
    TimHandle4.Init.ClockDivision = 0;
    TimHandle4.Init.CounterMode   = TIM_COUNTERMODE_UP;
    HAL_TIM_PWM_Init(&TimHandle4);

    sConfig3.OCMode     = TIM_OCMODE_PWM1;
    sConfig3.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfig3.OCFastMode = TIM_OCFAST_DISABLE;
    sConfig3.Pulse      = 2;   /* TRIG 짧은 High */
    HAL_TIM_PWM_ConfigChannel(&TimHandle4, &sConfig3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle4, TIM_CHANNEL_1);


    /* ---- IR ADC (좌=ADC3 ch11, 우=ADC2 ch14) ---- */
    ir_adc_init(&AdcHandle1, &adcConfig1, ADC3, ADC_CHANNEL_11);
    ir_adc_init(&AdcHandle2, &adcConfig2, ADC2, ADC_CHANNEL_14);


    /* ---- 모드별 태스크 생성 ---- */
    switch (current_mode) {
        case MODE_ROTATION_CALIBRATION:
            xTaskCreate(Calibration_task,  "Cal",  1000, NULL, 1, NULL);
            break;
        case MODE_MOVEMENT_TEST:
            xTaskCreate(MovementTest_task, "MoveTest", 1000, NULL, 1, NULL);
            break;
        case MODE_NORMAL:
        default:
            /* UW/IR 우선순위 2 (감지), Drive/Debug 우선순위 1 (반응) */
            xTaskCreate(UW_task, "UW", 1000, NULL, 2, NULL);
            xTaskCreate(IR_task,    "IR",    1000, NULL, 2, NULL);
            xTaskCreate(Drive_task, "Drive", 1000, NULL, 1, NULL);
            xTaskCreate(Debug_task, "Debug", 1000, NULL, 1, NULL);
            break;
    }

    vTaskStartScheduler();
    for (;;) {} // 도달 안함
}


/* =====================================================================
 *  모터 제어
 * ===================================================================== */

void Motor_Forward(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Backward(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Left(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2);
}

void Motor_Right(void)
{
    HAL_TIM_PWM_Start(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_1);
}

void Motor_Stop(void)
{
    HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TimHandle1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TimHandle2, TIM_CHANNEL_2);
}


/* =====================================================================
 *  Clock / Error / EXTI / Callbacks
 * ===================================================================== */

/* SYSCLK = 180 MHz (HSE 25MHz, PLL M=25 N=360 P=2 Q=7, AHB/APB1/APB2 = 1/4/2). */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    __PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_ActivateOverDrive();   /* 180MHz 도달 필수 */

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                 | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1) {}
}
#endif

static void Error_Handler(void)
{
    BSP_LED_On(LED3);
    while (1) {}
}

static void EXTILine_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    __GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    __GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Pin  = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case GPIO_PIN_15:
            encoder_right = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
            if (encoder_right == 0)      { motorInterrupt1++; encoder_right = READY; }
            else if (encoder_right == 1) { motorInterrupt1--; encoder_right = READY; }
            break;
        case GPIO_PIN_4:
            encoder_left = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
            if (encoder_left == 0)      { motorInterrupt2++; encoder_left = READY; }
            else if (encoder_left == 1) { motorInterrupt2--; encoder_left = READY; }
            break;
    }
}

static void capture_echo(uint32_t ccer_mask,
                         uint32_t *first, uint32_t *second, uint32_t *diff,
                         TIM_HandleTypeDef *htim, uint32_t channel)
{
    if ((TIM3->CCER & ccer_mask) == 0) {
        *first = HAL_TIM_ReadCapturedValue(htim, channel);
        TIM3->CCER |= ccer_mask;
    } else {
        *second = HAL_TIM_ReadCapturedValue(htim, channel);
        if      (*second > *first) *diff = *second - *first;
        else if (*second < *first) *diff = (0xFFFF - *first) + *second;
        else                       *diff = 0;
        TIM3->CCER &= ~ccer_mask;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;

    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_2:   /* 우 */
            capture_echo(TIM_CCER_CC2P, &uwIC2Value1, &uwIC2Value2,
                         &uwDiffCapture1, htim, TIM_CHANNEL_2);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:   /* 전 */
            capture_echo(TIM_CCER_CC3P, &uwIC2Value3, &uwIC2Value4,
                         &uwDiffCapture2, htim, TIM_CHANNEL_3);
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:   /* 좌 */
            capture_echo(TIM_CCER_CC4P, &uwIC2Value5, &uwIC2Value6,
                         &uwDiffCapture3, htim, TIM_CHANNEL_4);
            break;
        default: break;
    }
}
