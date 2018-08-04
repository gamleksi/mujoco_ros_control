/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/4/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <gtest/gtest.h>
#include <mujoco.h>

/** @brief Implement non blocking mju_error */
MJAPI void mju_error(const char *msg) {
    std::cout << msg << std::endl;
}

class MuJoCo : public ::testing::Test {
public:

    static void SetUpTestCase() {
        const auto key_path = std::string(getenv("HOME")) + "/.mujoco/mjkey.txt";
        activated = mj_activate(key_path.c_str()) == 1;
    }

    static void TearDownTestCase() {
        mj_deactivate();
    }


protected:
    void SetUp() override {
        ASSERT_TRUE(activated); //test activated for each test because SetUpTestCase ignores ASSERT...
    }

public:
    static bool activated;
};

bool MuJoCo::activated = false;


TEST_F(MuJoCo, Version) {
    ASSERT_EQ(150, mj_version());
}

TEST_F(MuJoCo, LoadModel) {
    const auto model_path = std::string(getenv("HOME")) + "/.mujoco/mjpro150/model/humanoid.xml";

    char error[1000] = "";
    auto m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    ASSERT_TRUE(m);
    auto d = mj_makeData(m);
    ASSERT_TRUE(d);

    EXPECT_EQ(0.002, m->opt.timestep);

    mj_deleteModel(m);
    mj_deleteData(d);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
