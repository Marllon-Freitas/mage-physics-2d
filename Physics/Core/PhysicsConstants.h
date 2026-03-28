#ifndef PHYSICS_CONSTANTS_H
#define PHYSICS_CONSTANTS_H

namespace mage {
    namespace math {
        constexpr float PI = 3.1415926535f;
        constexpr float TWO_PI = 6.2831853071f;
        constexpr float HALF_PI = 1.5707963267f;

        constexpr float DEG2RAD = PI / 180.0f;
        constexpr float RAD2DEG = 180.0f / PI;

        constexpr float EPSILON = 1e-4f;
        constexpr float EPSILON_SQ = 1e-8f;
    }

    namespace physics {
        // Conversão Mundo -> Tela
        constexpr int PIXELS_PER_METER = 50;

        // Tolerâncias de Simulação
        constexpr float STATIC_THRESHOLD = 0.005f;
        constexpr float SOLVER_EPSILON = 1e-9f;

        // Colisão & Manifold
        constexpr int   MAX_CONTACTS = 2;
        constexpr float CONTACT_PROXIMITY_THRESHOLD = 2.0f;

        // Solver & Constraints (Baumgarte & Sleep)
        constexpr float BAUMGARTE_BETA = 0.1f;
        constexpr float PENETRATION_SLOP = 0.5f;
        constexpr float SLEEP_IMPULSE_THRESHOLD = 0.5f;
    }
}

#endif