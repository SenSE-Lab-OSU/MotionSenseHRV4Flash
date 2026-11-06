#include <assert.h>
#include <stdint.h>

typedef float float32_t;
#define LOG_DBG(...) ((void)0)
/* DECLARATIONS */

static unsigned decisions;
static void restart(void) { /* RESET */ }
static int adaptation_due(void) { /* GATE */ return 1; return 0; }

static void poll(unsigned fifo_items)
{
    int number_of_samples = fifo_items / 4;
    uint32_t led1A[] = {100000}, led1B[] = {150000};
    uint32_t led2A[] = {200000}, led2B[] = {250000};
    /* ACQUISITION */
    /* MEANS */
    if (adaptation_due()) ++decisions;
}

static void assert_empty_unchanged(void)
{
    uint32_t counter = ppg_brightness_check_counter;
    float means[] = {runningMeanCh1a, runningMeanCh1b, runningMeanCh2a, runningMeanCh2b};
    unsigned before = decisions;
    for (unsigned items = 0; items < 4; ++items) {
        poll(items);
        assert(ppg_brightness_check_counter == counter);
        assert(runningMeanCh1a == means[0] && runningMeanCh1b == means[1]);
        assert(runningMeanCh2a == means[2] && runningMeanCh2b == means[3]);
        assert(decisions == before);
    }
}

static void assert_means(unsigned contributions)
{
    assert(runningMeanCh1a == 100000.0f * contributions / timeWindow);
    assert(runningMeanCh1b == 150000.0f * contributions / timeWindow);
    assert(runningMeanCh2a == 200000.0f * contributions / timeWindow);
    assert(runningMeanCh2b == 250000.0f * contributions / timeWindow);
}

int main(void)
{
    restart();
    for (unsigned cycle = 0; cycle < 4096; ++cycle) {
        unsigned before = decisions;
        for (uint32_t sample = 1; sample <= timeWindow; ++sample) {
            if (sample == timeWindow) assert_empty_unchanged();
            poll(4);
            assert(ppg_brightness_check_counter == sample);
            assert(adaptation_due() == (sample == timeWindow));
        }
        assert(decisions == before + 1);
        /* Existing rollover includes its sample in the next window. */
        assert_means(timeWindow + (cycle != 0));
        assert_empty_unchanged();
        poll(4);
        assert(ppg_brightness_check_counter == 0 && !adaptation_due());
        assert_means(1);
    }

    restart();
    assert_means(0);
    for (unsigned sample = 0; sample < timeWindow / 2; ++sample) poll(4);
    assert_means(timeWindow / 2);
    restart();
    assert(ppg_brightness_check_counter == 0);
    assert_means(0);
    assert_empty_unchanged();
    unsigned before = decisions;
    for (unsigned sample = 0; sample < timeWindow; ++sample) poll(4);
    assert_means(timeWindow);
    assert(decisions == before + 1);
    return 0;
}
