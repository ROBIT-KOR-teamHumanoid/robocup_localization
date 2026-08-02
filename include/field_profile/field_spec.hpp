#pragma once

#include <string>

// 필드 규격 정의 (2026 RoboCup 규정 기준)

namespace FieldSpecs
{

    struct Small_Field
    {
        static constexpr double A = 9.0;        // 필드 길이 (Field length)
        static constexpr double B = 6.0;        // 필드 너비 (Field width)
        static constexpr double C = 0.7;        // 골 깊이   (Goal depth)
        static constexpr double D = 1.8;        // 골 너비   (Goal width)
        static constexpr double E = 1.0;        // 골 에어리어 길이 (Goal area length)
        static constexpr double F = 3.0;        // 골 에어리어 너비 (Goal area width)
        static constexpr double G = 2.0;        // 페널티 에어리어 길이 (Penalty area length)
        static constexpr double H = 4.0;        // 페널티 에어리어 너비 (Penalty area width)
        static constexpr double I_DIST = 1.5;   // 페널티 마크 거리 (Penalty mark distance)
        static constexpr double J = 1.5;        // 센터 서클 지름 (Center circle diameter)
        static constexpr double K = 1.0;        // 보더 스트립 너비 (Border strip width, min)
        static constexpr double L_ARC = 0.0;    // 코너 아크 반지름 (Corner arc radius, none)
        static constexpr double LINE_W = 0.05;  // 라인 너비 (Line width)
        static constexpr double MARK_SZ = 0.10; // 페널티/센터 마크 크기 (Mark size)
    };

    struct Middle_Field
    {
        static constexpr double A = 14.0;
        static constexpr double B = 9.0;
        static constexpr double C = 1.2;
        static constexpr double D = 2.6;
        static constexpr double E = 1.0;
        static constexpr double F = 4.0;
        static constexpr double G = 3.0;
        static constexpr double H = 6.0;
        static constexpr double I_DIST = 2.0;
        static constexpr double J = 3.0;
        static constexpr double K = 1.0;
        static constexpr double L_ARC = 0.5;
        static constexpr double LINE_W = 0.09;
        static constexpr double MARK_SZ = 0.10;
    };

    struct Large_Field
    {
        static constexpr double A = 22.0;
        static constexpr double B = 14.0;
        static constexpr double C = 1.0;
        static constexpr double D = 2.6;
        static constexpr double E = 1.0;
        static constexpr double F = 5.0;
        static constexpr double G = 3.5;
        static constexpr double H = 7.0;
        static constexpr double I_DIST = 2.5;
        static constexpr double J = 4.0;
        static constexpr double K = 1.0;
        static constexpr double L_ARC = 1.0;
        static constexpr double LINE_W = 0.12;
        static constexpr double MARK_SZ = 0.15;
    };

    // 위 규격들을 담는 런타임 값 묶음.
    struct FieldSpec
    {
        double A, B, C, D, E, F, G, H;
        double I_DIST, J, K, L_ARC, LINE_W, MARK_SZ;
    };

    template <class Spec>
    constexpr FieldSpec makeSpec()
    {
        return FieldSpec{Spec::A, Spec::B, Spec::C, Spec::D,
                         Spec::E, Spec::F, Spec::G, Spec::H,
                         Spec::I_DIST, Spec::J, Spec::K, Spec::L_ARC,
                         Spec::LINE_W, Spec::MARK_SZ};
    }

    inline constexpr FieldSpec SMALL = makeSpec<Small_Field>();
    inline constexpr FieldSpec MIDDLE = makeSpec<Middle_Field>();
    inline constexpr FieldSpec LARGE = makeSpec<Large_Field>();

    inline const FieldSpec *fromName(const std::string &name)
    {
        if (name == "S" || name == "SMALL")
            return &SMALL;
        if (name == "M" || name == "MIDDLE")
            return &MIDDLE;
        if (name == "L" || name == "LARGE")
            return &LARGE;
        return nullptr;
    }

} // namespace FieldSpecs
