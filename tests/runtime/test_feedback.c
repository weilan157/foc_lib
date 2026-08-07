/*
 * test_feedback.c —— FeedbackBuffer（§4.3，必修 6）：Fast 写 → 读保持型快照
 */
#include "runtime/feedback.h"
#include "../test_assert.h"

int main(void)
{
    FeedbackBuffer fb;
    FastFeedback in;
    FastFeedback out;

    feedback_buffer_init(&fb);

    in.mech_angle_rad = 1.0f;
    in.mech_vel_radps = 2.0f;
    in.elec_angle_rad = 7.0f;
    in.quality        = FEEDBACK_OK;

    feedback_buffer_write(&fb, &in);
    CHECK(feedback_buffer_read(&fb, &out));        /* 任意消费方读 */
    CHECK_NEAR(out.mech_angle_rad, 1.0f, 1e-6f);
    CHECK_NEAR(out.elec_angle_rad, 7.0f, 1e-6f);
    CHECK(out.quality == FEEDBACK_OK);

    /* 多读者保持型：连续读一致 */
    CHECK(feedback_buffer_read(&fb, &out));
    CHECK_NEAR(out.mech_vel_radps, 2.0f, 1e-6f);

    TEST_REPORT();
}
