#include <gtest/gtest.h>

#include <iostream>

#include "vc/core/util/Signals.hpp"

using namespace volcart;

static int freeIntVal{0};
static void freeFnIntSlot(int i) { freeIntVal = i; }

static float freeFloatVal{0};
static void freeFnFloatSlot(float f) { freeFloatVal = f; }

static void freeFnIntFloatSlot(int i, float f)
{
    freeIntVal = i;
    freeFloatVal = f;
}

static void freeFnIntRefSlot(int& i) { i = 1; }
static void freeFnIntPtrSlot(int* i) { freeFnIntRefSlot(*i); }

[[noreturn]] static void freeFnExit()
{
    std::cerr << "Success" << std::endl;
    std::exit(0);
}

struct Receiver {
    int intVal{0};
    float floatVal{0};

    void intSlot(int i) { intVal = i; }
    void floatSlot(float f) { floatVal = f; }

    static void StaticFreeIntSlot(int i) { freeIntVal = i; }
    static void StaticFreeFloatSlot(float f) { freeFloatVal = f; }
};

TEST(Signals, FreeFnConnection)
{
    Signal<float> signal;
    signal.connect(freeFnIntSlot);
    signal.connect(&freeFnFloatSlot);

    // Test send
    signal.send(1.5);
    EXPECT_EQ(freeIntVal, 1);
    EXPECT_EQ(freeFloatVal, 1.5);

    // Test () operator
    signal(2.5);
    EXPECT_EQ(freeIntVal, 2);
    EXPECT_EQ(freeFloatVal, 2.5);
}

TEST(Signals, MemberFnConnection)
{
    Signal<float> signal;
    Receiver r;
    signal.connect(&r, &Receiver::intSlot);
    signal.connect(&r, &Receiver::floatSlot);

    signal.send(1.5);
    EXPECT_EQ(r.intVal, 1);
    EXPECT_EQ(r.floatVal, 1.5);
}

TEST(Signals, LambdaFnConnection)
{
    float floatVal{0};
    int intVal{0};

    Signal<float> signal;
    signal.connect([&intVal](int v) { intVal = v; });
    signal.connect([&floatVal](float v) { floatVal = v; });

    signal.send(1.5);
    EXPECT_EQ(intVal, 1);
    EXPECT_EQ(floatVal, 1.5);
}

TEST(Signals, StaticMemberFn)
{
    Signal<float> signal;

    signal.connect(Receiver::StaticFreeIntSlot);
    signal.connect(Receiver::StaticFreeFloatSlot);

    signal.send(1.5);
    EXPECT_EQ(freeIntVal, 1);
    EXPECT_EQ(freeFloatVal, 1.5);
}

TEST(Signals, MultiParameterFn)
{
    Signal<int, float> signal;
    signal.connect(freeFnIntFloatSlot);
    signal.send(1, 1.5);
    EXPECT_EQ(freeIntVal, 1);
    EXPECT_EQ(freeFloatVal, 1.5);

    Signal<float, int> swapped;
    swapped.connect(freeFnIntFloatSlot);
    swapped.send(2.5, 2);
    EXPECT_EQ(freeIntVal, 2);
    EXPECT_EQ(freeFloatVal, 2);
}

TEST(Signals, NoParameterFn)
{
    Signal<> signal;
    signal.connect(freeFnExit);
    EXPECT_EXIT(signal(), ::testing::ExitedWithCode(0), "Success");
}

TEST(Signals, NoParameterSlot)
{
    Signal<int> signal;
    signal.connect(freeFnExit);
    EXPECT_EXIT(signal.send(1), ::testing::ExitedWithCode(0), "Success");
}

TEST(Signals, ReferenceParameterFn)
{
    Signal<int&> signal;
    signal.connect(freeFnIntRefSlot);
    int val{0};
    signal.send(val);
    EXPECT_EQ(val, 1);
}

TEST(Signals, PointerParameterFn)
{
    Signal<int*> signal;
    signal.connect(freeFnIntPtrSlot);

    int* val = new int(0);
    signal.send(val);
    EXPECT_EQ(*val, 1);
    delete val;
}