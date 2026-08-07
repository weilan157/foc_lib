/*
 * test_command_buffer.c —— CommandBuffer 无锁双缓冲 + 保持型读语义（§11）
 */
#include "runtime/command.h"
#include "../test_assert.h"

int main(void)
{
    CommandBuffer cb;

    command_buffer_init(&cb);

    /* 写 / 读：内容一致 */
    {
        MotorCommand cmd = { .target = 3.14f, .mode = CTRL_MODE_POSITION, .sequence = 1u };
        MotorCommand out;
        CHECK(command_buffer_write(&cb, &cmd) == 0);
        CHECK(command_buffer_read(&cb, &out));                 /* 第一次读 = 新命令 */
        CHECK_NEAR(out.target, 3.14f, 1e-6f);
        CHECK(out.mode == CTRL_MODE_POSITION);
        CHECK(out.sequence == 1u);
    }

    /* 保持型：无新写时再读 → out 仍为最近值，返回 false */
    {
        MotorCommand out;
        CHECK(!command_buffer_read(&cb, &out));                /* 自上次读无新命令 */
        CHECK_NEAR(out.target, 3.14f, 1e-6f);                  /* 最近值仍有效 */
        CHECK(out.sequence == 1u);
    }

    /* 覆盖写 → 新命令检测 + sequence 单调 */
    {
        MotorCommand cmd = { .target = -1.0f, .mode = CTRL_MODE_VELOCITY, .sequence = 2u };
        MotorCommand out;
        CHECK(command_buffer_write(&cb, &cmd) == 0);
        CHECK(command_buffer_read(&cb, &out));                 /* 新命令 */
        CHECK_NEAR(out.target, -1.0f, 1e-6f);
        CHECK(out.sequence == 2u);
    }

    /* set_mode / get_mode */
    {
        MotorCommand cmd = { .target = 1.0f, .mode = CTRL_MODE_VELOCITY, .sequence = 3u };
        MotorCommand out;
        (void)command_buffer_write(&cb, &cmd);
        CHECK(command_buffer_set_mode(&cb, CTRL_MODE_TORQUE) == 0);
        CHECK(command_buffer_get_mode(&cb) == CTRL_MODE_TORQUE);
        (void)command_buffer_read(&cb, &out);
        CHECK(out.mode == CTRL_MODE_TORQUE);                   /* set_mode 保留 target */
        CHECK_NEAR(out.target, 1.0f, 1e-6f);
    }

    TEST_REPORT();
}
