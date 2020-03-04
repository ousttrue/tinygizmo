#pragma once
#include <stdint.h>
#include <bitset>

namespace screenstate
{
enum MouseButtonFlags : uint32_t
{
    None,
    LeftDown = 0x01,
    RightDown = 0x02,
    MiddleDown = 0x04,
    WheelPlus = 0x08,
    WheelMinus = 0x10,
    CursorUpdate = 0x20,
};

// auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(state.time - lastState.time).count() * 0.001f;

struct ScreenState
{
    int16_t Width;
    int16_t Height;
    int16_t MouseX;
    int16_t MouseY;
    float ElapsedSeconds;
    float DeltaSeconds;
    MouseButtonFlags MouseFlag;
    std::bitset<128> KeyCode = {};

    bool Has(MouseButtonFlags flag) const
    {
        return (MouseFlag & flag) != 0;
    }

    bool MouseLeftDown() const { return Has(MouseButtonFlags::LeftDown); }
    bool MouseRightDown() const { return Has(MouseButtonFlags::RightDown); }
    bool MouseMiddleDown() const { return Has(MouseButtonFlags::MiddleDown); }
    int MouseWheel() const
    {
        if (Has(MouseButtonFlags::WheelPlus))
        {
            return 1;
        }
        else if (Has(MouseButtonFlags::WheelMinus))
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }

    void Set(MouseButtonFlags flag)
    {
        MouseFlag = (MouseButtonFlags)(MouseFlag | flag);
    }

    void Unset(MouseButtonFlags flag)
    {
        MouseFlag = (MouseButtonFlags)(MouseFlag & ~flag);
    }

    void Clear()
    {
        Unset((MouseButtonFlags)(MouseButtonFlags::WheelMinus | MouseButtonFlags::WheelPlus | MouseButtonFlags::CursorUpdate));
    }

    float AspectRatio() const
    {
        if (Height == 0)
        {
            return 1;
        }
        return (float)Width / Height;
    }

    bool HasCapture() const
    {
        return Has(MouseButtonFlags::LeftDown) || Has(MouseButtonFlags::RightDown) || Has(MouseButtonFlags::MiddleDown);
    }

    bool SameSize(const ScreenState &rhs) const
    {
        return Width == rhs.Width && Height == rhs.Height;
    }

    // float DeltaSeconds(const ScreenState &rhs) const
    // {
    //     auto delta = Time - rhs.Time;
    //     if (delta == 0)
    //     {
    //         return 0.001f;
    //     }
    //     return 0.001f * delta;
    // }
};
static_assert(sizeof(ScreenState::KeyCode) == 16, "KeyCode bytes");
static_assert(sizeof(ScreenState) == 40, "sizeof(WindowMouseState)");
} // namespace screenstate
