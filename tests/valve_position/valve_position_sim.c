#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define FULL_RANGE 4096LL

typedef enum
{
    POSITION_0   = 0,
    POSITION_25  = 25,
    POSITION_50  = 50,
    POSITION_75  = 75,
    POSITION_100 = 100

} position_percent_t;


static int64_t calculate_target(
    int64_t closed,
    int64_t open,
    position_percent_t percent)
{
    int64_t span = open - closed;

    return closed +
           (span * (int64_t)percent) / 100;
}


static void test_calculation(
    int64_t closed,
    int64_t open)
{
    printf("\n========================================\n");
    printf("Calibration: CLOSED=%lld OPEN=%lld\n",
           (long long)closed,
           (long long)open);
    printf("========================================\n");

    position_percent_t positions[] =
    {
        POSITION_0,
        POSITION_25,
        POSITION_50,
        POSITION_75,
        POSITION_100
    };

    for (unsigned i = 0; i < 5; ++i)
    {
        int64_t target =
            calculate_target(
                closed,
                open,
                positions[i]);

        printf("%3d%% -> target total angle = %lld\n",
               positions[i],
               (long long)target);
    }
}


static void simulate_movement(
    int64_t closed,
    int64_t open,
    position_percent_t target_percent)
{
    int64_t target =
        calculate_target(
            closed,
            open,
            target_percent);

    int64_t current = closed;

    printf("\n----------------------------------------\n");
    printf("Movement simulation: %d%%\n",
           target_percent);
    printf("Current = %lld\n",
           (long long)current);
    printf("Target  = %lld\n",
           (long long)target);
    printf("----------------------------------------\n");

    while (current != target)
    {
        if (current < target)
        {
            current += 256;

            if (current > target)
            {
                current = target;
            }

            printf("OPEN  -> current=%lld\n",
                   (long long)current);
        }
        else
        {
            current -= 256;

            if (current < target)
            {
                current = target;
            }

            printf("CLOSE -> current=%lld\n",
                   (long long)current);
        }
    }

    printf("STOP  -> target reached\n");
}


int main(void)
{
    printf("\n");
    printf("========================================\n");
    printf(" GSM-VALVE VALVE POSITION PC SIMULATOR\n");
    printf("========================================\n");

    /*
     * Normal calibration.
     */
    test_calculation(
        0,
        FULL_RANGE);

    /*
     * Simulate movement to 25%.
     */
    simulate_movement(
        0,
        FULL_RANGE,
        POSITION_25);

    /*
     * Simulate movement to 50%.
     */
    simulate_movement(
        0,
        FULL_RANGE,
        POSITION_50);

    /*
     * Simulate movement to 75%.
     */
    simulate_movement(
        0,
        FULL_RANGE,
        POSITION_75);

    /*
     * Simulate movement to OPEN.
     */
    simulate_movement(
        0,
        FULL_RANGE,
        POSITION_100);

    /*
     * Non-zero absolute calibration.
     */
    test_calculation(
        12000,
        16096);

    printf("\n========================================\n");
    printf(" SIMULATION COMPLETE\n");
    printf("========================================\n");

    return 0;
}